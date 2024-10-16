# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(conn)

include_directories(BEFORE
    ${ZEPHYR_BASE}/tests/bluetooth/host/conn
    ${ZEPHYR_BASE}/tests/bluetooth/host/conn/mocks/zephyr/include
)

add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/host host_mocks)
add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/host/conn/mocks mocks)

target_link_libraries(testbinary PRIVATE mocks host_mocks)

target_sources(testbinary
    PRIVATE
    src/main.c

    ${ZEPHYR_BASE}/subsys/bluetooth/host/conn.c
    ${ZEPHYR_BASE}/subsys/logging/log_minimal.c
    ${ZEPHYR_BASE}/lib/net_buf/buf.c
    ${ZEPHYR_BASE}/lib/net_buf/buf_simple.c
)

target_compile_options(testbinary
	PRIVATE
	-DCONFIG_BT_EXT_ADV=y
	-DCONFIG_BT_PER_ADV=y
	-DCONFIG_BT_PER_ADV_SYNC=y
	-DCONFIG_BT_CREATE_CONN_TIMEOUT=10000
	-DCONFIG_BT_CONN=y
	-DCONFIG_BT_MAX_CONN=1
	-DCONFIG_BT_CONN_TX_USER_DATA_SIZE=32
	-DCONFIG_BT_L2CAP_TX_MTU=23
	-DCONFIG_BT_CONN_TX_MAX=3
	-DCONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT=5000
	-DCONFIG_BT_L2CAP_TX_BUF_COUNT=3
	-DCONFIG_BT_ID_MAX=1
	-DCONFIG_BT_CENTRAL=y)
