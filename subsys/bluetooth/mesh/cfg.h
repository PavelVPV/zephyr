/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "settings.h"

void bt_mesh_cfg_init(void);
void bt_mesh_cfg_pending_store(bt_mesh_settings_store_func store_func);

bool bt_mesh_fixed_group_match(uint16_t addr);
