#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

#set -x
# Compile all the applications needed by the bsim tests in these subfolders

#set -x #uncomment this line for debugging
set -ue
: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

export BOARD="nrf52_bsim"

source ${ZEPHYR_BASE}/tests/bsim/compile.source

cmake_extra_args="-DEXTRA_DTC_OVERLAY_FILE=${ZEPHYR_BASE}/tests/bsim/bluetooth/host/l2cap/deadlock/flash.overlay;${ZEPHYR_BASE}/tests/bsim/bluetooth/host/l2cap/deadlock/sdc.overlay" app=tests/bsim/bluetooth/host/l2cap/deadlock compile

wait_for_background_jobs
