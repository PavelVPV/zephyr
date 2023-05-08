#!/usr/bin/env bash
# Copyright 2021 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

#RunTest mesh_transport_va transport_rx_va_collision_find
RunTest mesh_transport_va_collision transport_tx_va_collision transport_rx_va_collision

conf=prj_mesh1d1_conf
