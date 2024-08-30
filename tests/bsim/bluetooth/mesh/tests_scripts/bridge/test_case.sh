#!/usr/bin/env bash
# Copyright 2024 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# *Test description*
#
# Test procedure:
RunTest mesh_brg_unicast \
	brg_rx_simple brg_tx_simple # brg_tx_simple brg_tx_simple

overlay=overlay_psa_conf
RunTest mesh_brg_unicast \
	brg_rx_simple brg_tx_simple # brg_tx_simple brg_tx_simple
