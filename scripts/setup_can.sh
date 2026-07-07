#!/bin/bash
# This script must be executed, not sourced.
(return 0 2>/dev/null) && {
    echo "Error: This script must be run, not sourced."
    return 1 2>/dev/null || exit 1
}

# Make sure the script runs with super user privileges.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

# Set bitrate
ip link set can0 type can bitrate 1000000
# Set queue length
ip link set can0 txqueuelen 256
# Bring up CAN interface
ip link set can0 up