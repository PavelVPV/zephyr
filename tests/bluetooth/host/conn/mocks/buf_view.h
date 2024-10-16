/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/fff.h>

/* List of fakes used by this unit tester */
#define BUF_VIEW_MOCKS_FFF_FAKES_LIST(FAKE)                                                        \
		FAKE(bt_buf_has_view)

DECLARE_FAKE_VALUE_FUNC(bool, bt_buf_has_view, const struct net_buf *);
