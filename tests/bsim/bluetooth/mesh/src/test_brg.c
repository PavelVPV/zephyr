/*
 * Copyright (c) 2024 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Subnet bridge test
 */

#include "mesh_test.h"
#include <zephyr/bluetooth/mesh.h>
#include "mesh/net.h"
#include "mesh/keys.h"
#include "bsim_args_runner.h"
#include "common/bt_str.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test_brg, LOG_LEVEL_INF);

#define WAIT_TIME 70 /*seconds*/

#define PROV_ADDR 0x0001
#define PROV_ADDR_START 0x0002
/* These addresses should be sequential.
 * Bridge address must be equal to PROV_ADDR_START.
 */
#define BRIDGE_ADDR 0x0002
#define DEVICE_ADDR_START 0x0003

static int REMOTE_NODES = 2;

static const uint8_t prov_dev_key[16] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xab,
					  0xcd, 0xef, 0x01, 0x23, 0x45, 0x67,
					  0x89, 0xab, 0xcd, 0xef };

static const uint8_t subnet_keys[][16] = {
	{ 0xaa, 0xbb, 0xcc },
	{ 0xdd, 0xee, 0xff },
	{ 0x11, 0x22, 0x33 },
};

static uint8_t prov_uuid[16] = { 0x6c, 0x69, 0x6e, 0x67, 0x61, 0xaa };
static uint8_t bridge_uuid[16] = { 0x6c, 0x69, 0x6e, 0x67, 0x61, 0xbb };
static uint8_t dev_uuid[16] = { 0x6c, 0x69, 0x6e, 0x67, 0x61, 0xcc };

static int test_ividx = 0x123456;

extern const struct bt_mesh_comp comp;
static bool provisioner_ready;

struct msg {
	uint8_t payload;
};

enum {
	MSG_TYPE_DATA = 0,
	MSG_TYPE_GET = 1,
	MSG_TYPE_STATUS = 2,
};

static struct msg recvd_msgs[10];
static uint8_t recvd_msgs_cnt;

BUILD_ASSERT((2 /* opcode */ + 1 /* type */ + 1 /* msgs cnt */ + sizeof(recvd_msgs) +
	      BT_MESH_MIC_SHORT) <= BT_MESH_RX_SDU_MAX,
	     "Status message does not fit into the maximum incoming SDU size.");
BUILD_ASSERT((2 /* opcode */ + 1 /* type */ + 1 /* msgs cnt */ + sizeof(recvd_msgs) +
	      BT_MESH_MIC_SHORT) <= BT_MESH_TX_SDU_MAX,
	     "Status message does not fit into the maximum outgoing SDU size.");

static K_SEM_DEFINE(prov_status_sem, 0, 1);
static K_SEM_DEFINE(prov_sem, 0, 1);

static void test_provisioner_init(void)
{
	bt_mesh_test_cfg_set(NULL, WAIT_TIME);
}

static void test_bridge_init(void)
{
	/* Bridge device must always be the second device. */
	ASSERT_EQUAL(1, get_device_nbr());

	bt_mesh_test_cfg_set(NULL, WAIT_TIME);
}

static void test_device_init(void)
{
	ASSERT_TRUE_MSG(get_device_nbr() >= 2, "Regular devices must be initialized after"
			"Provisioner and Bridge devices.");

	/* Regular devices addresses starts from address 0x0003.*/
	dev_uuid[6] = get_device_nbr() + 1;

	/* Regular devices are provisioned into subnets starting with idx 1. */
	dev_uuid[8] = get_device_nbr() - 1;

	bt_mesh_test_cfg_set(NULL, WAIT_TIME);
}

static void unprovisioned_beacon(uint8_t uuid[16], bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash)
{
	int err;

	/* Subnet may not be ready yet when provisioner receives a beacon. */
	if (!provisioner_ready) {
		LOG_INF("Provisioner is not ready yet");
		return;
	}

	LOG_INF("Received unprovisioned beacon, uuid %s", bt_hex(uuid, 16));

	if (!memcmp(uuid, bridge_uuid, 16)) {
		err = bt_mesh_provision_adv(uuid, 0, BRIDGE_ADDR, 0);
		if (!err) {
			LOG_INF("Provisioning bridge at address 0x%04x", BRIDGE_ADDR);
		}

		return;
	}

	/* UUID[6] - address to be used for provisioning.
	 * UUID[8] - subnet to be used for provisioning.
	 */
	uint16_t addr = uuid[6];
	int subnet_idx = uuid[8];

	err = bt_mesh_provision_adv(uuid, subnet_idx, addr, 0);
	if (!err) {
		LOG_INF("Provisioning device at address 0x%04x with NetKeyIdx 0x%04x", addr,
			subnet_idx);
	}
}

static void prov_node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
	LOG_INF("Device 0x%04x provisioned, NetKeyIdx 0x%04x", addr, net_idx);
	k_sem_give(&prov_sem);
}

static struct bt_mesh_prov provisioner_prov = {
	.uuid = prov_uuid,
	.unprovisioned_beacon = unprovisioned_beacon,
	.node_added = prov_node_added
};

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	LOG_INF("Device 0x%04x provisioning is complete, NetKeyIdx 0x%04x", addr, net_idx);
	k_sem_give(&prov_sem);
}

static struct bt_mesh_prov device_prov = {
	.uuid = dev_uuid,
	.complete = prov_complete,
};

static struct bt_mesh_prov bridge_prov = {
	.uuid = bridge_uuid,
	.complete = prov_complete,
};

static void provisioner_setup(void)
{
	uint8_t status;
	int err;

	bt_mesh_device_setup(&provisioner_prov, &comp);

	ASSERT_OK(bt_mesh_cdb_create(test_net_key));
	ASSERT_OK(bt_mesh_provision(test_net_key, 0, 0, test_ividx, PROV_ADDR, prov_dev_key));

	err = bt_mesh_cfg_cli_app_key_add(0, PROV_ADDR, 0, 0, test_app_key, &status);
	if (err || status) {
		FAIL("AppKey add failed (err %d, status %u)", err, status);
		return;
	}

	err = bt_mesh_cfg_cli_mod_app_bind(0, PROV_ADDR, PROV_ADDR, 0, TEST_MOD_ID, &status);
	if (err || status) {
		FAIL("Mod app bind failed (err %d, status %u)", err, status);
		return;
	}

	for (int i = 0; i < REMOTE_NODES; i++) {
		LOG_INF("Creating subnet idx %d", i);

		ASSERT_OK(bt_mesh_cfg_cli_net_key_add(0, PROV_ADDR, i + 1, subnet_keys[i], &status));
		if (status) {
			FAIL("NetKey add failed (status %u)", status);
			return;
		}

		struct bt_mesh_cdb_subnet *subnet = bt_mesh_cdb_subnet_alloc(i + 1);
		ASSERT_TRUE(subnet != NULL);

		ASSERT_OK(bt_mesh_cdb_subnet_key_import(subnet, 0, subnet_keys[i]));

		bt_mesh_cdb_subnet_store(subnet);
	}

	provisioner_ready = true;
}

static void bridge_entry_add(uint16_t dst, uint16_t net_idx)
{
	struct bt_mesh_bridging_table_entry entry;
	struct bt_mesh_bridging_table_status rsp;

	entry.directions = BT_MESH_BRG_CFG_DIR_TWOWAY;
	entry.net_idx1 = 0x0000;
	entry.net_idx2 = net_idx;
	entry.addr1 = PROV_ADDR;
	entry.addr2 = dst;

	ASSERT_OK(bt_mesh_brg_cfg_cli_bridging_table_add(0, BRIDGE_ADDR, &entry, &rsp));
	if (rsp.status ||
	    rsp.entry.directions != BT_MESH_BRG_CFG_DIR_TWOWAY ||
	    rsp.entry.net_idx1 != 0x0000 || rsp.entry.net_idx2 != net_idx ||
	    rsp.entry.addr1 != PROV_ADDR || rsp.entry.addr2 != dst) {
		FAIL("Bridging table add failed (status %u)", rsp.status);
		return;
	}
}

static void provisioner_bridge_configure(int remote_devices)
{
	uint8_t status;
	int err;

	for (int i = 0; i < REMOTE_NODES; i++) {
		err = bt_mesh_cfg_cli_net_key_add(0, BRIDGE_ADDR, i + 1, subnet_keys[i], &status);
		if (err || status) {
			FAIL("NetKey add failed (err %d, status %u)", err, status);
			return;
		}
	}

	for (int i = 0; i < remote_devices; i++) {
		/** FIXME: Needs better way of handling error */
		bridge_entry_add(DEVICE_ADDR_START + i, i + 1);
	}

	ASSERT_OK(bt_mesh_brg_cfg_cli_subnet_bridge_set(0, BRIDGE_ADDR,
							BT_MESH_SUBNET_BRIDGE_ENABLED, &status));
	if (status != BT_MESH_SUBNET_BRIDGE_ENABLED) {
		FAIL("Subnet bridge set failed (status %u)", status);
		return;
	}

	LOG_INF("Bridge configured");
}

static void provisioner_device_configure(uint16_t net_key_idx, uint16_t addr)
{
	int err;
	uint8_t status;

	err = bt_mesh_cfg_cli_app_key_add(net_key_idx, addr, net_key_idx, 0, test_app_key, &status);
	if (err || status) {
		FAIL("AppKey add failed (err %d, status %u)", err, status);
		return;
	}

	err = bt_mesh_cfg_cli_mod_app_bind(net_key_idx, addr, addr, 0, TEST_MOD_ID, &status);
	if (err || status) {
		FAIL("Mod app bind failed (err %d, status %u)", err, status);
		return;
	}

	LOG_INF("Device 0x%04x configured", addr);
}

static void provisioner_ra_cb(uint8_t *data, size_t length)
{
	uint8_t type = data[0];

	LOG_HEXDUMP_DBG(data, length, "Provisioner received message");

	ASSERT_TRUE_MSG(length > 1, "Too short message");
	ASSERT_EQUAL(type, MSG_TYPE_STATUS);

	recvd_msgs_cnt = data[1];
	ASSERT_EQUAL(recvd_msgs_cnt * sizeof(recvd_msgs[0]), length - 2);
	memcpy(recvd_msgs, &data[2], recvd_msgs_cnt * sizeof(recvd_msgs[0]));

	k_sem_give(&prov_status_sem);
}

static int send_data(uint16_t dst, uint8_t payload)
{
	uint8_t data[2] = {MSG_TYPE_DATA, payload};

	return bt_mesh_test_send_ra(dst, data, sizeof(data), NULL, NULL);
}

static int send_get(uint16_t dst)
{
	uint8_t data[1] = {MSG_TYPE_GET};

	return bt_mesh_test_send_ra(dst, data, sizeof(data), NULL, NULL);
}

static void device_ra_cb(uint8_t *data, size_t length)
{
	uint8_t type = data[0];

	LOG_HEXDUMP_DBG(data, length, "Device received message");

	switch (type) {
	case MSG_TYPE_DATA:
		ASSERT_EQUAL(2, length);
		ASSERT_TRUE_MSG(recvd_msgs_cnt < ARRAY_SIZE(recvd_msgs), "Too many messages");

		recvd_msgs[recvd_msgs_cnt].payload = data[1];
		recvd_msgs_cnt++;

		break;

	case MSG_TYPE_GET: {
		uint8_t test_data[1 /*type */ + 1 /* msgs cnt */ + sizeof(recvd_msgs)] = {
			MSG_TYPE_STATUS,
			recvd_msgs_cnt
		};

		memcpy(&test_data[2], recvd_msgs, recvd_msgs_cnt * sizeof(recvd_msgs[0]));

		ASSERT_OK(bt_mesh_test_send_ra(PROV_ADDR, test_data,
					       2 + recvd_msgs_cnt * sizeof(recvd_msgs[0]), NULL,
					       NULL));

		memset(recvd_msgs, 0, sizeof(recvd_msgs));
		recvd_msgs_cnt = 0;

		break;
	}

	case MSG_TYPE_STATUS:
		ASSERT_TRUE_MSG(false, "Unexpected message");
		break;
	}
}

static void test_provisioner_simple(void)
{
	/** FIXME: */
	REMOTE_NODES = 2;

	provisioner_setup();

	for (int i = 0; i < 1 /* bridge */ + REMOTE_NODES; i++) {
		LOG_INF("Waiting for a device to provision...");
		ASSERT_OK(k_sem_take(&prov_sem, K_SECONDS(40)));
	}

	LOG_INF("Configuring bridge...");
	provisioner_bridge_configure(REMOTE_NODES);

	LOG_INF("Configuring devices...");
	for (int i = 0; i < REMOTE_NODES; i++) {
		provisioner_device_configure(i + 1, DEVICE_ADDR_START + i);
	}

	bt_mesh_test_ra_cb_setup(provisioner_ra_cb);

	LOG_INF("Sending data...");
	const int msgs_cnt = 3;
	for (int i = 0; i < REMOTE_NODES; i++) {
		/* FIXME: must be struct msg */
		uint8_t payload = i | i << 4;

		for (int j = 0; j < msgs_cnt; j++) {
			ASSERT_OK(send_data(DEVICE_ADDR_START + i, payload + j));
		}
	}

	LOG_INF("Checking data...");
	for (int i = 0; i < REMOTE_NODES; i++) {
		uint8_t payload = i | i << 4;

		ASSERT_OK(send_get(DEVICE_ADDR_START + i));
		ASSERT_OK(k_sem_take(&prov_status_sem, K_SECONDS(5)));

		ASSERT_EQUAL(recvd_msgs_cnt, msgs_cnt);
		for (int j = 0; j < recvd_msgs_cnt; j++) {
			ASSERT_EQUAL(recvd_msgs[j].payload, payload + j);
		}
	}

	PASS();
}

static void test_bridge_simple(void)
{
	bt_mesh_device_setup(&bridge_prov, &comp);

	ASSERT_OK(bt_mesh_prov_enable(BT_MESH_PROV_ADV));

	LOG_INF("Waiting for being provisioned...");
	ASSERT_OK(k_sem_take(&prov_sem, K_SECONDS(40)));
	LOG_INF("Bridge is provisioned");

	PASS();
}

static void test_device_simple(void)
{
	bt_mesh_device_setup(&device_prov, &comp);

	ASSERT_OK(bt_mesh_prov_enable(BT_MESH_PROV_ADV));

	LOG_INF("Waiting for being provisioned...");
	ASSERT_OK(k_sem_take(&prov_sem, K_SECONDS(40)));
	LOG_INF("Node is provisioned");

	bt_mesh_test_ra_cb_setup(device_ra_cb);

	PASS();
}

#define TEST_CASE(role, name, description)                 \
	{                                                  \
		.test_id = "brg_" #role "_" #name,         \
		.test_post_init_f = test_##role##_init,    \
		.test_descr = description,                 \
		.test_tick_f = bt_mesh_test_timeout,       \
		.test_main_f = test_##role##_##name,       \
	}

static const struct bst_test_instance test_brg[] = {
	TEST_CASE(provisioner, simple, "Provisioner node"),
	TEST_CASE(bridge, simple, "Subnet Bridge node"),
	TEST_CASE(device, simple, "Simple mesh device"),

	BSTEST_END_MARKER
};

struct bst_test_list *test_brg_install(struct bst_test_list *tests)
{
	tests = bst_add_tests(tests, test_brg);
	return tests;
}
