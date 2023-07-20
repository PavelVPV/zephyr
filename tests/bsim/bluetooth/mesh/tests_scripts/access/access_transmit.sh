#!/usr/bin/env bash
# Copyright 2021 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

RunTest mesh_access_pub_retr \
	access_tx_transmit access_rx_transmit

conf=prj_mesh1d1_conf
RunTest mesh_access_pub_retr_1d1 \
	access_tx_transmit access_rx_transmit

conf=prj_psa_conf
RunTest mesh_access_pub_retr_psa \
	access_tx_period access_rx_period
