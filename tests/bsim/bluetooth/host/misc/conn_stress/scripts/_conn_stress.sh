#!/usr/bin/env bash
# Copyright (c) 2023 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

set -x

export BOARD="nrf5340bsim/nrf5340/cpuapp"

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="conn_stress"

test_path="tests_bsim_bluetooth_host_misc_conn_stress"
bsim_central_exe_name="bs_${BOARD_TS}_${test_path}_central_prj_conf"
bsim_peripheral_exe_name="bs_${BOARD_TS}_${test_path}_peripheral_prj_conf"

# terminate running simulations (if any)
${BSIM_COMPONENTS_PATH}/common/stop_bsim.sh $simulation_id

cd ${BSIM_OUT_PATH}/bin

bsim_args="-RealEncryption=1 -v=2 -s=${simulation_id}"
test_args="-argstest notify_size=220 conn_interval=32"

Execute ./bs_2G4_phy_v1 -v=2 -s=${simulation_id} -D=2 -sim_length=1000e6 &
Execute "./${bsim_central_exe_name}" ${bsim_args} -d=0 -rs=43 -testid=central ${test_args}
Execute "./${bsim_peripheral_exe_name}" ${bsim_args} -d=1 -rs=100 -testid=peripheral ${test_args}

wait_for_background_jobs
