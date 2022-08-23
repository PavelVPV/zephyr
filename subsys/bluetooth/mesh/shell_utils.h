/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/shell/shell.h>

bool shell_model_first_get(uint16_t id, struct bt_mesh_model **mod);

int shell_model_instance_set(const struct shell *shell, struct bt_mesh_model **mod,
			      uint16_t mod_id, uint8_t elem_idx);

int shell_model_instances_get_all(const struct shell *shell, uint16_t mod_id);

int shell_model_cmds_help(const struct shell *shell, size_t argc, char **argv);
