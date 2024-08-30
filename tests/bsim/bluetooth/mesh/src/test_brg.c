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
// #define REMOTE_ADDR_START 0x0002
#define REMOTE_NODES 1
static const uint16_t subnet_key_idxs[] = {0x0001, 0x0002, 0x0003};
static const uint8_t subnet_keys[16] = { 0xaa, 0xbb, 0xcc };
static uint8_t prov_uuid[16] = { 0x6c, 0x69, 0x6e, 0x67, 0x61, 0xaa };
static uint16_t remote_addrs[] = {0x0002, 0x0003, 0x0004};
static uint8_t remote_uuids[][16] = {
	{0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
	{0x11, 0x12, 0x13, 0x14, 0x15, 0x16},
	{0x21, 0x22, 0x23, 0x24, 0x25, 0x26}
};
static uint8_t provisioned_uuids[REMOTE_NODES][16];
static int test_ividx = 0x123456;

extern const struct bt_mesh_comp comp;
static bool provisioner_ready;
static int remote_idx = 0;
static uint8_t test_flags;

static void unprovisioned_beacon(uint8_t uuid[16], bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash)
{
	/* Subnet may not be ready yet when provisioner receives a beacon. */
	if (!provisioner_ready) {
		LOG_INF("Provisioner is not ready yet");
		return;
	}
	for (int i = 0; i < REMOTE_NODES; i++) {
		if (memcmp(uuid, provisioned_uuids[i], 16) == 0) {
			LOG_INF("Device already provisioned");
			return;
		}
	}
	LOG_INF("Received unprovisioned beacon, uuid %s", bt_hex(uuid, 16));
	LOG_INF("Provisioning device 0x%04x with NetKeyIdx 0x%04x", remote_addrs[remote_idx], subnet_key_idxs[remote_idx]);
	ASSERT_OK(bt_mesh_provision_adv(uuid, subnet_key_idxs[remote_idx], remote_addrs[remote_idx], 0));
	memcpy(provisioned_uuids[remote_idx], uuid, 16);
	remote_idx++;
}

static struct k_sem prov_sem;

static void prov_node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
	LOG_INF("Device 0x%04x provisioned, NetKeyIdx 0x%04x", addr, net_idx);
	k_sem_give(&prov_sem);
}

static struct bt_mesh_prov brg_prov = {
	.uuid = prov_uuid,
	.unprovisioned_beacon = unprovisioned_beacon,
	.node_added = prov_node_added
};

static const struct bt_mesh_test_cfg brg_node_cfg = {
	.addr = 0x0001,
	.dev_key = {0x01},
};

static struct bt_mesh_test_cfg remote_cfg;

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	LOG_INF("Device 0x%04x provisioning is complete, NetKeyIdx 0x%04x", addr, net_idx);
	k_sem_give(&prov_sem);
}

static void remote_node_init(void)
{
	struct bt_mesh_prov prov = {
		.uuid = remote_uuids[bsim_args_get_global_device_nbr() - 1],
		.complete = prov_complete,
	};

	k_sem_init(&prov_sem, 0, 1);

	/* Guarantee that the devices addresses are unique
	 * for each device in the simulation.
	 */
	remote_cfg.addr = remote_addrs[bsim_args_get_global_device_nbr() - 1];
	remote_cfg.dev_key[0] = bsim_args_get_global_device_nbr();
	bt_mesh_test_cfg_set(&remote_cfg, WAIT_TIME);
	bt_mesh_device_setup(&prov, &comp);

	ASSERT_OK(bt_mesh_prov_enable(BT_MESH_PROV_ADV));

	LOG_INF("Waiting for being provisioned...");
	ASSERT_OK(k_sem_take(&prov_sem, K_SECONDS(40)));
	LOG_INF("Node provisioned...");
}

static void brg_conf(void)
{
	uint8_t status;
	int err;

	k_sem_init(&prov_sem, 0, 1);

	bt_mesh_test_cfg_set(&brg_node_cfg, WAIT_TIME);
	bt_mesh_device_setup(&brg_prov, &comp);

	ASSERT_OK(bt_mesh_cdb_create(test_net_key));
	ASSERT_OK(bt_mesh_provision(test_net_key, 0, test_flags, test_ividx, brg_node_cfg.addr,
				    brg_node_cfg.dev_key));

	err = bt_mesh_cfg_cli_app_key_add(0, cfg->addr, 0, 0, test_app_key, &status);
	if (err || status) {
		FAIL("AppKey add failed (err %d, status %u)", err, status);
		return;
	}

	err = bt_mesh_cfg_cli_mod_app_bind(0, cfg->addr, cfg->addr, 0, TEST_MOD_ID, &status);
	if (err || status) {
		FAIL("Mod app bind failed (err %d, status %u)", err, status);
		return;
	}

	for (int i = 0; i < REMOTE_NODES; i++) {
		LOG_INF("Creating subnet idx %d", i);
		ASSERT_OK(bt_mesh_subnet_add(subnet_key_idxs[i], &subnet_keys[i]));

		struct bt_mesh_cdb_subnet *subnet = bt_mesh_cdb_subnet_alloc(subnet_key_idxs[i]);
		ASSERT_TRUE(subnet != NULL);
		err = bt_mesh_cdb_subnet_key_import(subnet, 0, &subnet_keys[i]);
		if (err) {
			FAIL("Unable to import netkey (err: %d)", err);
		}
		LOG_WRN("Storing subnet idx %d", i);
		bt_mesh_cdb_subnet_store(subnet);
		LOG_WRN("Stored subnet idx %d", i);
		LOG_INF("Created subnet idx %d", i);
	}

	provisioner_ready = true;
}

static void test_rx_simple(void)
{
	int err;

	brg_conf();

	for (int i = 0; i < REMOTE_NODES; i++) {
		LOG_INF("Provisioning a remote device...");
		ASSERT_OK(k_sem_take(&prov_sem, K_SECONDS(40)));
	}

	for (int i = 0; i < 3 * REMOTE_NODES; i++) {
		err = bt_mesh_test_recv(1, cfg->addr, NULL, K_SECONDS(10));
		ASSERT_OK_MSG(err, "Failed receiving vector %d", i);
	}

	PASS();
}

static void test_tx_simple(void)
{
	int err;
	remote_node_init();

	for (int i = 0; i < 3; i++) {
		err = bt_mesh_test_send(brg_node_cfg.addr, NULL, 1, 0, K_SECONDS(10));
		ASSERT_OK_MSG(err, "Failed sending vector %d", i);
	}

	PASS();
}

#define TEST_CASE(role, name, description)                 \
	{                                                  \
		.test_id = "brg_" #role "_" #name,         \
		.test_descr = description,                 \
		.test_tick_f = bt_mesh_test_timeout,       \
		.test_main_f = test_##role##_##name,       \
	}

static const struct bst_test_instance test_brg[] = {
	TEST_CASE(tx, simple,        "BRG: send to unicast addr"),

	TEST_CASE(rx, simple,        "BRG: send to unicast addr"),
	BSTEST_END_MARKER};

struct bst_test_list *test_brg_install(struct bst_test_list *tests)
{
	tests = bst_add_tests(tests, test_brg);
	return tests;
}
