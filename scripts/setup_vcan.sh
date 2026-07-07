#!/bin/bash

# This script must be executed, not sourced.
(return 0 2>/dev/null) && {
    echo "Error: This script must be run, not sourced."
    return 1 2>/dev/null || exit 1
}

# Make sure the script runs with super user privileges.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"
# Load the kernel module.
modprobe vcan
# Create the virtual CAN interface.
ip link add dev vcan0 type vcan
# Bring the virtual CAN interface online.
ip link set up vcan0
