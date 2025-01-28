: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

export BOARD="nrf5340bsim/nrf5340/cpuapp"

source ${ZEPHYR_BASE}/tests/bsim/compile.source

app=tests/bsim/bluetooth/host/misc/conn_stress/peripheral sysbuild=1 compile
app=tests/bsim/bluetooth/host/misc/conn_stress/central sysbuild=1 compile

wait_for_background_jobs
