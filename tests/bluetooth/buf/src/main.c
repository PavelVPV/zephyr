/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <errno.h>
#include <zephyr/tc_util.h>
#include <zephyr/ztest.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/sys/byteorder.h>

static enum bt_buf_type freed_buf_type;
static K_SEM_DEFINE(rx_sem, 0, 1);

void bt_buf_rx_freed_cb(enum bt_buf_type type)
{
	freed_buf_type = type;
	k_sem_give(&rx_sem);
}

ZTEST_SUITE(test_buf_data_api, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_buf_data_api, test_buf_freed_cb)
{
	struct net_buf *buf;
	int err;

	bt_buf_rx_freed_cb_set(bt_buf_rx_freed_cb);

	struct {
		enum bt_buf_type type;
		enum bt_buf_type exp_type_mask;
	} test_vector[] = {
#if defined(CONFIG_BT_HCI_RAW)
		{BT_BUF_ACL_IN, BT_BUF_ACL_IN | BT_BUF_EVT | BT_BUF_ISO_IN},
		{BT_BUF_EVT, BT_BUF_ACL_IN | BT_BUF_EVT | BT_BUF_ISO_IN},
		{BT_BUF_ISO_IN, BT_BUF_ACL_IN | BT_BUF_EVT | BT_BUF_ISO_IN},
#elif defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
		{BT_BUF_ACL_IN, BT_BUF_ACL_IN },
		{BT_BUF_EVT, BT_BUF_EVT },
		{BT_BUF_ISO_IN, BT_BUF_ISO_IN},
#else /* !defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL) */
	/* When ACL Flow Control is disabled, a single pool is used for events and
	 * acl data. This means that the callback will always notify about both
	 * types of buffers, BT_BUF_EVT and BT_BUF_ACL_IN.
	 */
		{BT_BUF_ACL_IN, BT_BUF_ACL_IN | BT_BUF_EVT },
		{BT_BUF_EVT, BT_BUF_EVT | BT_BUF_ACL_IN },
		{BT_BUF_ISO_IN, BT_BUF_ISO_IN},
#endif /* CONFIG_BT_HCI_RAW */
	};

	for (int i = 0; i < ARRAY_SIZE(test_vector); i++) {
		buf = bt_buf_get_rx(test_vector[i].type, K_NO_WAIT);
		zassert_not_null(buf, "Failed to get buffer");

		net_buf_unref(buf);
		/* The freed buf cb is called from net_buf_unref, therefore the semaphore should
		 * already by given.
		 */
		err = k_sem_take(&rx_sem, K_NO_WAIT);
		zassert_equal(err, 0, "Timeout while waiting for buffer");
		zassert_equal(test_vector[i].exp_type_mask, freed_buf_type,
			      "Unexpected buffer type");
	}
}

/** The test checks that after releasing a buffer allocated through bt_buf_get_rx the type_mask
 * returned through the bt_buf_rx_freed_cb_t correctly provides freed buffer
 * types.
 *
 * Test procedure:
 *
 * 1. For each bt_buf_type accepted by bt_buf_get_rx, pre-allocate buf and store it.
 * 2. Allocate rest buffer until pools for each bt_buf_type is empty.
 * 3. For each allocated bt_buf_type:
 * 3.1. Release the buffer
 * 3.2. Store the freed type_mask
 * 3.3. For each bt_buf_type in the freed type_mask:
 * 3.3.1. Allocate the buffer
 * 3.3.2. Check that it is not NULL
 * 3.3.3. Release the buffer
 * 3.4. Re-allocate the released buffer
 */
ZTEST(test_buf_data_api, test_buf_freed_cb_2)
{
	bt_buf_rx_freed_cb_set(bt_buf_rx_freed_cb);

	struct {
		enum bt_buf_type type;
		struct net_buf *buf;
	} test_vector[] = {
		{ BT_BUF_EVT, },
		{ BT_BUF_ACL_IN, },
		{ BT_BUF_ISO_IN, },
	};

	printk("Step 1: pre-allocating bufs\n");

	for (int i = 0; i < ARRAY_SIZE(test_vector); i++) {
		test_vector[i].buf = bt_buf_get_rx(test_vector[i].type, K_NO_WAIT);
		zassert_not_null(test_vector[i].buf, "Failed to get buf");
	}

	printk("Step 2: Emptying the pools\n");

	for (int i = 0; i < ARRAY_SIZE(test_vector); i++) {
		struct net_buf *buf;

		do {
			buf = bt_buf_get_rx(test_vector[i].type, K_NO_WAIT);
		} while (buf != NULL);
	}

	printk("Step 3: Verifying type_mask returned in the bt_buf_get_rx_freed_cb_t\n");

	for (int i = 0; i < ARRAY_SIZE(test_vector); i++) {
		enum bt_buf_type freed_buf_type_mask;

		net_buf_unref(test_vector[i].buf);

		printk("i: %d, freed_buf_type: %X\n", i, freed_buf_type);

		freed_buf_type_mask = freed_buf_type;

		for (int j = 0; j < NUM_BITS(freed_buf_type_mask); j++) {
			enum bt_buf_type tested_type = 1 << j;

			printk("j: %d, tested_type: %X\n", i, tested_type);

			if (freed_buf_type_mask & tested_type) {
				struct net_buf *buf;

				printk("type was freed, reallocating it\n");

				buf = bt_buf_get_rx(tested_type, K_NO_WAIT);
				zassert_not_null(buf, "Failed to get buf");

				net_buf_unref(buf);
			}
		}

		test_vector[i].buf = bt_buf_get_rx(test_vector[i].type, K_NO_WAIT);
		zassert_not_null(test_vector[i].buf);
	}
}
