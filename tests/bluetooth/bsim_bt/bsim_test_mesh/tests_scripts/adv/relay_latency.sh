#!/usr/bin/env bash
# Copyright 2021 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

RunTest mesh_relay_latency relay_latency_tester_send_timestamped_msgs \
	relay_latency_dut_relay_only \
	relay_latency_tester_recv_and_check_latency
