/*
 * Copyright (c) 2024 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mesh_test.h"
#include "mesh/net.h"
#include "mesh/transport.h"
#include "mesh/va.h"
#include "mesh/adv.h"
#include <zephyr/sys/byteorder.h>

#include "mesh/crypto.h"

#define LOG_MODULE_NAME test_tx_stress

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "common/bt_str.h"

#define WAIT_TIME 70 /*seconds*/

#define ADV_INT_FAST_MS    20

#define ADV_WORKQ_NUM_THREADS (CONFIG_BT_BUF_CMD_TX_COUNT * 2)

static struct k_work_q adv_work_q[ADV_WORKQ_NUM_THREADS];
static K_THREAD_STACK_ARRAY_DEFINE(adv_work_stack, ADV_WORKQ_NUM_THREADS, 1024);
static struct k_work adv_work[ADV_WORKQ_NUM_THREADS];
static struct bt_le_ext_adv *adv_sets[ADV_WORKQ_NUM_THREADS];

static const struct bt_mesh_test_cfg tx_cfg = {
	.addr = 0x0001,
	.dev_key = { 0x01 },
};

static K_SEM_DEFINE(mesh_tx_sem, 0, 1);

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

	NET_BUF_SIMPLE_DEFINE(buf, 29);

	net_buf_simple_init(&buf, 0);

	const uint8_t dummy_data[] = "012345678";
	net_buf_simple_add_mem(&buf, dummy_data, 9);

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
		return err;
	}

	err = bt_le_ext_adv_start(instance, &start);
	if (err) {
		LOG_ERR("Failed to start adv (err: %d)", err);
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

static void tx_started(uint16_t dur, int err, void *data)
{
	if (err) {
		FAIL("Send start failed (%d)", err);
	}
}

static void tx_ended(int err, void *data)
{
	if (err) {
		FAIL("Send end failed (%d)", err);
	}

	k_sem_give(&mesh_tx_sem);
}

const struct bt_mesh_send_cb send_cb = {
	.start = tx_started,
	.end = tx_ended,
};

static void test_dut_concurrency(void)
{
	int err;

	bt_mesh_test_cfg_set(&tx_cfg, WAIT_TIME);
	bt_mesh_test_setup();

	advertisers_setup();

	for (int i = 0; i < 10000; i++) {
		err = bt_mesh_test_send_async(0x0123, NULL, 5, 0, &send_cb, NULL);
		if (err) {
			LOG_WRN("Failed to send message (%d). Probably filled out buffers", err);

			err = k_sem_take(&mesh_tx_sem, K_SECONDS(1));
			if (err) {
				FAIL("Failed to wait for a sent message (%d)", err);
			}
		}
	}

	PASS();
}

#define TEST_CASE(role, name, description)                                     \
	{                                                                      \
		.test_id = "tx_stress_" #role "_" #name,                       \
		.test_descr = description,                                     \
		.test_tick_f = bt_mesh_test_timeout,                           \
		.test_main_f = test_##role##_##name,                           \
	}

static const struct bst_test_instance test_connect[] = {
	TEST_CASE(dut, concurrency,        "TBA"),

	BSTEST_END_MARKER
};

struct bst_test_list *test_tx_stress_install(struct bst_test_list *tests)
{
	tests = bst_add_tests(tests, test_connect);
	return tests;
}
