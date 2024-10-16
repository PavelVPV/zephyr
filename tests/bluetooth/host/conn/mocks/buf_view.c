/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "buf_view.h"

DEFINE_FAKE_VALUE_FUNC(bool, bt_buf_has_view, const struct net_buf *);
