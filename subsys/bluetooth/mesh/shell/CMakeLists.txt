# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(shell.c)

zephyr_library_sources(shell_utils.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_HEALTH_CLI shell_health_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_CFG_CLI shell_config_cli.c)
