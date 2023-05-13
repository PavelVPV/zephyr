#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# Test receives on group and virtual addresses in the LPN
RunTest mesh_friendship_msg_va_collision \
	friendship_lpn_va_collision \
	friendship_friend_va_collision

conf=prj_mesh1d1_conf
RunTest mesh_friendship_msg_va_collision_1d1 \
	friendship_lpn_va_collision \
	friendship_friend_va_collision
