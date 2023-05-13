/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct bt_mesh_va {
	uint16_t ref:15,
		 changed:1;
	uint16_t addr;
	uint8_t  uuid[16];
};

uint8_t bt_mesh_va_add(const uint8_t uuid[16], const struct bt_mesh_va **entry);
uint8_t bt_mesh_va_del(const uint8_t *uuid);
/* Finds va entry by user's UUID */
const struct bt_mesh_va *bt_mesh_va_get(const uint8_t *uuid);

/* Needed for storing va as indexes in persistent storage. */
const uint8_t *bt_mesh_va_get_uuid_by_idx(uint16_t idx);
uint16_t bt_mesh_va_get_idx_by_uuid(const uint8_t *uuid);

void bt_mesh_va_pending_store(void);
void bt_mesh_va_clear(void);
