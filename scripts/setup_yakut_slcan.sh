#!/bin/bash

# This script must be sourced, not executed.
(return 0 2>/dev/null) || {
    echo "Error: This script must be sourced, not run."
    exit 1
}

if [[ ! -v CYPHAL_PATH ]]; then
    echo "CYPHAL_PATH is not set." >> /dev/stderr
    exit 1
elif [[ -z "$CYPHAL_PATH" ]]; then
    echo "CYPHAL_PATH is set to the empty string." >> /dev/stderr
    exit 1
else
    echo "CYPHAL_PATH = \"$CYPHAL_PATH\""
fi

# There is a difference between slcan and can, but for sake of simplicity, this sets SocketCAN interface as can0
if ! [ -e /sys/class/net/can0 ]; then
    sudo ./setup_slcan.sh --remove-all --basename can --speed-code 8 --baudrate 115200 /dev/serial/by-id/usb-Zubax*Babel*
fi
export UAVCAN__CAN__IFACE='socketcan:can0'

export UAVCAN__CAN__MTU=8
export UAVCAN__CAN__BITRATE='1000000 1000000'   # Arbitration and data segment bit rates.
export UAVCAN__NODE__ID=$(yakut accommodate)  # Pick an unoccupied node-ID automatically for this shell session.
echo "Auto-selected node-ID for this session: $UAVCAN__NODE__ID"
