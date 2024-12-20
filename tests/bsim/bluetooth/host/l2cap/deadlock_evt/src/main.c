/*
 * Copyright (c) 2024 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bstests.h"
#include "common.h"

#include "host/conn_internal.h"

#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#include "host/hci_core.h"

#define LOG_MODULE_NAME main
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static struct k_work_q dut_work_q;
static K_THREAD_STACK_DEFINE(dut_work_stack, 1024);

static void heavy_work_handler(struct k_work *work)
{
	LOG_DBG("Heavy work started");
	k_busy_wait(100 * 1000);
	LOG_DBG("Heavy work done");

	k_sleep(K_MSEC(1));
//	k_work_submit_to_queue(&dut_work_q, work);
}

static K_WORK_DEFINE(heavy_work, heavy_work_handler);

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
		    uint8_t adv_type, struct net_buf_simple *buf)
{
	//LOG_HEXDUMP_DBG(buf->data, buf->len, "Recvd adv report");

#if 1
	uint16_t tx_octets;
	uint16_t tx_time;
	int err;

	err = bt_hci_le_read_max_data_len(&tx_octets, &tx_time);
	if (err) {
		FAIL("Failed to read max data len (err %d)", err);
	}

	//LOG_WRN("Max data len: %u, max tx time: %u", tx_octets, tx_time);
#else
	struct bt_conn_le_tx_power power_level = {0};
	int err;
	err = bt_conn_le_get_tx_power_level(default_conn, &power_level);
	if (err) {
		FAIL("Failed to get tx power level (err %d)", err);
	}

	printf("Tx power level: %d\n", power_level.current_level);
#endif
}

#define BT_MESH_ADV_SCAN_UNIT(_ms) ((_ms) * 8 / 5)

int scan_enable(void)
{
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.interval = BT_MESH_ADV_SCAN_UNIT(30),
		.window = BT_MESH_ADV_SCAN_UNIT(30),
	};
	int err;

	LOG_DBG("");

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err && err != -EALREADY) {
		LOG_ERR("starting scan failed (err %d)", err);
		return err;
	}

	return 0;
}

static void test_dut_main(void)
{
	LOG_DBG("DUT deadlock started");
	int err;

	err = bt_enable(NULL);
	if (err) {
		FAIL("Can't enable Bluetooth (err %d)", err);
		return;
	}

	(void)settings_load();

	k_work_queue_start(&dut_work_q, dut_work_stack,
			   K_THREAD_STACK_SIZEOF(dut_work_stack),
			   K_PRIO_COOP(CONFIG_BT_RX_PRIO + 1), NULL);
	k_thread_name_set(&dut_work_q.thread, "DUT workq");

	err = scan_enable();
	if (err) {
		FAIL("Scanning failed to start (err %d)", err);
		return;
	}

	k_sleep(K_SECONDS(2));

	k_work_submit_to_queue(&dut_work_q, &heavy_work);

	k_sleep(K_SECONDS(60));

	PASS("DUT deadlock test passed\n");
}

#define ADV_INT_FAST_MS    20
#define ADV_WORKQ_NUM_THREADS 1

static struct k_work_q adv_work_q[ADV_WORKQ_NUM_THREADS];
static K_THREAD_STACK_ARRAY_DEFINE(adv_work_stack, ADV_WORKQ_NUM_THREADS, 1024);
static struct k_work adv_work[ADV_WORKQ_NUM_THREADS];
static struct bt_le_ext_adv *adv_sets[ADV_WORKQ_NUM_THREADS];

static void adv_sent_cb(struct bt_le_ext_adv *instance, struct bt_le_ext_adv_sent_info *info)
{
	int i;
	int err;
	for (i = 0; i < ADV_WORKQ_NUM_THREADS; i++) {
		if (adv_sets[i] == instance) {
			break;
		}
	}
	__ASSERT_NO_MSG(i < ADV_WORKQ_NUM_THREADS);
	err = k_work_submit_to_queue(&adv_work_q[i], &adv_work[i]);
	if (err < 0) {
		FAIL("Failed to submit adv work[%d] (err: %d)", i, err);
	}
}

static int send_adv(struct bt_le_ext_adv *instance)
{
	int err;
	NET_BUF_SIMPLE_DEFINE(buf, 100);
	net_buf_simple_init(&buf, 0);

	net_buf_simple_add(&buf, 60);

	struct bt_le_ext_adv_start_param start = {
		.num_events = 1,
	};

	struct bt_data ad;
	ad.type = BT_DATA_MESH_MESSAGE;
	ad.data_len = buf.len;
	ad.data = buf.data;

	err = bt_le_ext_adv_set_data(instance, &ad, 1, NULL, 0);
	if (err) {
		LOG_ERR("Failed to set adv data (err: %d)", err);
		FAIL("");
		return err;
	}

	err = bt_le_ext_adv_start(instance, &start);
	if (err) {
		LOG_ERR("Failed to start adv (err: %d)", err);
		FAIL("");
		return err;
	}

	return 0;
}

static void adv_work_handler(struct k_work *work)
{
	int i;
	int err;

	i = ARRAY_INDEX(adv_work, work);
	err = send_adv(adv_sets[i]);
	if (err) {
		LOG_ERR("Failed to send adv (err: %d) from adv[%d], resubmitting", err, i);
		err = k_work_submit_to_queue(&adv_work_q[i], work);
		if (err < 0) {
			FAIL("Failed to resubmit adv work[%d] (err: %d)", i, err);
		}
	}
}

static void advertisers_setup(void)
{
	static const struct bt_le_ext_adv_cb adv_cb = { .sent = adv_sent_cb };
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_EXT_ADV,
		.interval_min = BT_MESH_ADV_SCAN_UNIT(ADV_INT_FAST_MS),
		.interval_max = BT_MESH_ADV_SCAN_UNIT(ADV_INT_FAST_MS)
	};
	int err;
	for (int i = 0; i < ADV_WORKQ_NUM_THREADS; i++) {
		k_work_queue_start(&adv_work_q[i], adv_work_stack[i],
				   K_THREAD_STACK_SIZEOF(adv_work_stack[i]),
				   K_PRIO_COOP(1), NULL);
		k_thread_name_set(&adv_work_q[i].thread, "Test adv workq");
		k_work_init(&adv_work[i], adv_work_handler);
		err = bt_le_ext_adv_create(&adv_param, &adv_cb, &adv_sets[i]);
		if (err) {
			FAIL("Failed to create advertiser instance[%d] (err: %d)", i, err);
		}
	}
	for (int i = 0; i < ADV_WORKQ_NUM_THREADS; i++) {
		err = k_work_submit_to_queue(&adv_work_q[i], &adv_work[i]);
		if (err < 0) {
			FAIL("Failed to submit adv work[%d] (err: %d)", i, err);
		}
	}
}

static void test_tester_adv_main(void)
{
	int err;

	LOG_DBG("Tester deadlock started");

	err = bt_enable(NULL);
	ASSERT(err == 0, "Can't enable Bluetooth (err %d)\n", err);
	LOG_DBG("Central Bluetooth initialized.");

	(void)settings_load();

	advertisers_setup();

	PASS("Tester adv deadlock passed\n");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "dut",
		.test_descr = "DUT acting as peripheral",
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_dut_main
	},
	{
		.test_id = "tester_adv",
		.test_descr = "Tester acting as advertiser",
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_tester_adv_main
	},
	BSTEST_END_MARKER
};

struct bst_test_list *test_main_l2cap_deadlock_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}

bst_test_install_t test_installers[] = {
	test_main_l2cap_deadlock_install,
	NULL
};

int main(void)
{
	bst_main();
	return 0;
}
