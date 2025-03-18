/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "babblekit/testcase.h"
#include "babblekit/flags.h"
#include "common.h"

#define TEST_CONN_PARAM BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 2, \
					 BT_GAP_MS_TO_CONN_TIMEOUT(32000))

static const struct bt_uuid *test_svc_uuid = TEST_SERVICE_UUID;

struct server {
	atomic_t flag_is_connected;
	atomic_t flag_is_encrypted;
	atomic_t flag_discover_complete;
	atomic_t flag_long_subscribed;
	struct bt_conn *conn;
	uint16_t long_chrc_handle;
	size_t num_notifications;
};

static struct server servers[CONFIG_BT_MAX_CONN];

static struct server *server_alloc(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(servers); i++) {
		if (servers[i].conn == NULL) {
			return &servers[i];
		}
	}

	return NULL;
}

static void server_free(struct server *server)
{
	server->conn = NULL;
}

static struct server *server_find(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(servers); i++) {
		if (servers[i].conn == conn) {
			return &servers[i];
		}
	}

	return NULL;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct server *server;

	server = server_alloc();
	__ASSERT(server != NULL, "No more servers available");

	server->conn = conn;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		TEST_FAIL("Failed to connect to %s (%u)", addr, err);
		return;
	}

	printk("Connected to %s\n", addr);

	SET_FLAG(server->flag_is_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct server* server;

	server = server_find(conn);
	__ASSERT(server != NULL, "Unknown disconnection");

	bt_addr_le_to_str(bt_conn_get_dst(server->conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(server->conn);
	server_free(server);

	UNSET_FLAG(server->flag_is_connected);
}

void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	struct server* server;

	server = server_find(conn);
	__ASSERT(server != NULL, "Security change for unknown connection");

	if (err) {
		TEST_FAIL("Encryption failure (%d)", err);
	} else if (level < BT_SECURITY_L2) {
		TEST_FAIL("Insufficient sec level (%d)", level);
	} else {
		SET_FLAG(server->flag_is_encrypted);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;
	struct bt_conn *stub_conn = NULL;

	/* We're only interested in connectable events */
	if (type != BT_HCI_ADV_IND && type != BT_HCI_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	printk("Stopping scan\n");
	err = bt_le_scan_stop();
	if (err != 0) {
		TEST_FAIL("Could not stop scan: %d");
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, TEST_CONN_PARAM,
				&stub_conn);
	if (err != 0) {
		TEST_FAIL("Could not connect to peer: %d", err);
	}
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	struct server* server;
	int err;

	server = server_find(conn);
	__ASSERT(server != NULL, "Discovery complete for unknown connection");

	if (attr == NULL) {
		if (server->long_chrc_handle == 0) {
			TEST_FAIL("Did not discover long_chrc (%x)", server->long_chrc_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		SET_FLAG(server->flag_discover_complete);

		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, TEST_SERVICE_UUID) == 0) {
		printk("Found test service\n");
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			TEST_FAIL("Discover failed (err %d)", err);
		}

		return BT_GATT_ITER_STOP;
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		const struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, TEST_CHRC_UUID) == 0) {
			printk("Found long_chrc\n");
			server->long_chrc_handle = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void gatt_discover(struct server *server, enum bt_att_chan_opt opt)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	printk("Discovering services and characteristics\n");

	discover_params.uuid = test_svc_uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;
	discover_params.chan_opt = opt;

	err = bt_gatt_discover(server->conn, &discover_params);
	if (err != 0) {
		TEST_FAIL("Discover failed(err %d)", err);
	}

	WAIT_FOR_FLAG(server->flag_discover_complete);
	printk("Discovery complete\n");
}

static void test_long_subscribed(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_subscribe_params *params)
{
	struct server* server;

	server = server_find(conn);
	__ASSERT(server != NULL, "Long subscribe unknown connection");

	if (err) {
		TEST_FAIL("Subscribe failed (err %d)", err);
	}

	SET_FLAG(server->flag_long_subscribed);

	if (!params) {
		printk("params NULL\n");
		return;
	}

	if (params->value_handle == server->long_chrc_handle) {
		if ((params->value & BT_GATT_CCC_NOTIFY) == 0) {
			printk("Subscribed to long characteristic for server %d\n", ARRAY_INDEX(servers, server));
		} else {
			printk("Unsubscribed from long characteristic for server %d\n", ARRAY_INDEX(servers, server));
		}
	} else {
		TEST_FAIL("Unknown handle %d", params->value_handle);
	}
}

uint8_t test_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data,
		    uint16_t length)
{
	struct server* server;

	server = server_find(conn);
	__ASSERT(server != NULL, "Received notification for unknown connection");

	printk("Received notification #%u with length %d from server %d\n", server->num_notifications++, length, ARRAY_INDEX(servers, server));

	if (server->num_notifications == NOTIFICATION_COUNT) {
		printk("All notifications received from server %d\n", ARRAY_INDEX(servers, server));
	}

	/* This causes ACL data drop in HCI IPC driver. */
	k_sleep(K_MSEC(1000));

	return BT_GATT_ITER_CONTINUE;
}

static struct bt_gatt_discover_params disc_params_long;
static struct bt_gatt_subscribe_params sub_params_long = {
	.notify = test_notify,
	.subscribe = test_long_subscribed,
	.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE,
	.disc_params = &disc_params_long,
	.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
	.value = BT_GATT_CCC_NOTIFY,
};

static void gatt_subscribe_long(struct server *server, enum bt_att_chan_opt opt)
{
	int err;

	UNSET_FLAG(server->flag_long_subscribed);
	sub_params_long.value_handle = server->long_chrc_handle;
	sub_params_long.chan_opt = opt;
	err = bt_gatt_subscribe(server->conn, &sub_params_long);
	if (err < 0) {
		TEST_FAIL("Failed to subscribe: %d", err);
	} else {
		printk("Subscribe request sent\n");
	}
}

static void gatt_unsubscribe_long(struct server *server, enum bt_att_chan_opt opt)
{
	int err;

	UNSET_FLAG(server->flag_long_subscribed);
	sub_params_long.value_handle = server->long_chrc_handle;
	sub_params_long.chan_opt = opt;
	err = bt_gatt_unsubscribe(server->conn, &sub_params_long);
	if (err < 0) {
		TEST_FAIL("Failed to unsubscribe: %d", err);
	} else {
		printk("Unsubscribe request sent\n");
	}
}

static void setup(void)
{
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		TEST_FAIL("Bluetooth discover failed (err %d)", err);
	}

	for (size_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct server *server;

		printk("Connection to %d device...\n", i);

		err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
		if (err != 0) {
			TEST_FAIL("Scanning failed to start (err %d)", err);
		}

		printk("Scanning successfully started\n");

		/* TODO: This code is bug-prone as server_alloc may not allocate in the same order
		 * as the loop.
		 */
		server = &servers[i];

		WAIT_FOR_FLAG(server->flag_is_connected);

		err = bt_conn_set_security(server->conn, BT_SECURITY_L2);
		if (err) {
			TEST_FAIL("Starting encryption procedure failed (%d)", err);
		}

		WAIT_FOR_FLAG(server->flag_is_encrypted);

		while (bt_eatt_count(server->conn) < CONFIG_BT_EATT_MAX) {
			k_sleep(K_MSEC(10));
		}

		printk("EATT connected\n");
	}
}

static void test_main_client(void)
{
	setup();

	for (size_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct server *server = &servers[i];

		gatt_discover(server, BT_ATT_CHAN_OPT_ENHANCED_ONLY);
	}

	for (size_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct server *server = &servers[i];

		gatt_subscribe_long(server, BT_ATT_CHAN_OPT_ENHANCED_ONLY);
		WAIT_FOR_FLAG(server->flag_long_subscribed);

		printk("Server %d subscribed\n", i);
	}

	printk("All servers subscribed\n");

	uint32_t num_notifications;

	do {
		k_sleep(K_MSEC(100));

		num_notifications = 0;
		for (size_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			struct server *server = &servers[i];

			num_notifications += server->num_notifications;
		}
	} while (num_notifications < NOTIFICATION_COUNT * ARRAY_SIZE(servers));

	printk("All notifications received\n");

	for (size_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct server *server = &servers[i];

//		if (server->conn == NULL) {
//			continue;
//		}

		gatt_unsubscribe_long(server, BT_ATT_CHAN_OPT_ENHANCED_ONLY);
		WAIT_FOR_FLAG(server->flag_long_subscribed);

		printk("Server %d unsubscribed\n", i);
	}

	TEST_PASS("GATT client Passed");
}

static const struct bst_test_instance test_vcs[] = {
	{
		.test_id = "gatt_client_enhanced_notif_stress",
		.test_main_f = test_main_client,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_gatt_client_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_vcs);
}
