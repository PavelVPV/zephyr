#!/usr/bin/env bash
# Copyright 2024 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# *Test description*
# This test is to verify the bridge feature of the mesh stack.
# 3 roles are used in this test: Provisioner, Subnet Bridge Device, and Device.
#
# Subnets topology*:
#        Provisioner
#             |
#         (subnet 0)
#            |
#      Subnet Bridge (bridges subnets 0 <-> 1, 0 <-> 2)
#        /         \
#    (subnet 1)  (subnet 2)
#       |          |
#     Device    Device
#
# (*) - All devices in the provisioner range
#
# Test procedure:
# 1. Provisioner configures itself and creates subnets equal to number of non-bridge devices.
# 2. Provisioner provisions and configures Subnet Bridge node to bridge the subnets.
# 3. Provisioner provisions and configures non-bridge devices for each subnet.
# 4. Provisioner sends a DATA message to each device encrypted with primary key.
# 5. Devices store the received messages.
# 6. Provisioner sends a GET message to each device encrypted with primary key.
# 7. Devices send the stored messages back to the provisioner through a message STATUS encrypted
# with the key of the subnets they are provisioned to.
# 8. Provisioner verifies that each device received messages.

RunTest mesh_brg_simple \
	brg_provisioner_simple brg_bridge_simple brg_device_simple brg_device_simple

#overlay=overlay_psa_conf
#RunTest mesh_brg_simple \
#	provisioner_node_simple bridge_simple device_simple device_simple
