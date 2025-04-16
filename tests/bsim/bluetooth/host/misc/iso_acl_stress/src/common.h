/*
 * Common functions and helpers for ISO tests
 *
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include <zephyr/sys/atomic.h>

#define TEST_SERVICE_UUID                                                                          \
	BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,      \
			    0x07, 0x08, 0x09, 0x00, 0x00)

#define TEST_CHRC_UUID                                                                             \
	BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,      \
			    0x07, 0x08, 0x09, 0xFF, 0x11)

#define COMMAND_COUNT 30

extern struct bt_conn *default_conn;
extern atomic_t flag_connected;
extern atomic_t flag_conn_updated;
