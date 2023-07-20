#!/usr/bin/env bash
# Copyright 2022 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# The test instance sequence must stay as it is due to addressing scheme
conf=prj_mesh1d1_conf
RunTest blob_caps_no_rsp \
	blob_cli_caps_no_rsp blob_srv_caps_no_rsp blob_srv_caps_no_rsp

conf=prj_psa_conf
RunTest blob_caps_no_rsp_psa \
	blob_cli_caps_no_rsp blob_srv_caps_no_rsp blob_srv_caps_no_rsp
