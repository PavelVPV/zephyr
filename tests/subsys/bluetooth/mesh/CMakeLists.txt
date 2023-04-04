#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bt_mesh_fuzzy_test)

FILE(GLOB app_sources src/*.c)

target_sources(app
  PRIVATE
  ${app_sources}
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/main.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/cfg.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/adv.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/beacon.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/net.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/subnet.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/app_keys.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/heartbeat.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/crypto.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/access.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/msg.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/rpl.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/settings.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/transport.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/cfg_srv.c
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh/health_srv.c

  ${ZEPHYR_BASE}/subsys/net/buf.c
  ${ZEPHYR_BASE}/subsys/net/buf_simple.c

  ${ZEPHYR_BASE}/../modules/crypto/tinycrypt/lib/source/ecc_platform_specific.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/subsys/bluetooth/mesh
  ${ZEPHYR_BASE}/subsys/bluetooth
  ${ZEPHYR_BASE}/../modules/crypto/tinycrypt/lib/include
  ${ZEPHYR_BASE}/include/zephyr/bluetooth
  ${ZEPHYR_BASE}/include/zephyr/settings
  )

target_compile_options(app
  PRIVATE
  -DCONFIG_BT_MESH
  -DCONFIG_BT_MESH_UNPROV_BEACON_INT=10
  -DCONFIG_BT_MESH_DEFAULT_TTL=3
  -DCONFIG_BT_MESH_NETWORK_TRANSMIT_COUNT=1
  -DCONFIG_BT_MESH_NETWORK_TRANSMIT_INTERVAL=10
  -DCONFIG_BT_MESH_LOG_LEVEL=0
  -DCONFIG_BT_MESH_SUBNET_COUNT=1
  -DCONFIG_BT_MESH_APP_KEY_COUNT=1
  -DCONFIG_BT_MESH_CRPL=32
  -DCONFIG_BT_MESH_IV_UPDATE_SEQ_LIMIT=10
  -DCONFIG_BT_MESH_IVU_DIVIDER=1
  -DCONFIG_BT_MESH_MSG_CACHE_SIZE=10
  -DCONFIG_BT_MESH_ADV_BUF_COUNT=10
  -DCONFIG_BT_MESH_LOOPBACK_BUFS=10
  -DCONFIG_BT_MESH_MODEL_KEY_COUNT=5
  -DCONFIG_BT_MESH_MODEL_GROUP_COUNT=5
  -DCONFIG_BT_MESH_TX_SEG_MSG_COUNT=32
  -DCONFIG_BT_MESH_RX_SEG_MSG_COUNT=32
  -DCONFIG_BT_MESH_SEG_BUFS=255
  -DCONFIG_BT_MESH_LABEL_COUNT=1
  -UCONFIG_BT_SETTINGS
  -DCONFIG_BT_LOG_LEVEL=0
  -DCONFIG_BT_ID_MAX=1
  -DCONFIG_BT_MESH_STORE_TIMEOUT=10
  -UCONFIG_LOG
  -DCONFIG_NET_BUF_LOG_LEVEL=0
)

zephyr_ld_options(
    ${LINKERFLAGPREFIX},--allow-multiple-definition
    )
