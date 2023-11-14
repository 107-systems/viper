#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

CAN=can0
CAN_BITRATE=250000

GPIO_CAN0_STBY=160
GPIO_CAN1_STBY=161

if [ "$CAN" = "can0" ]; then
  GPIO_CAN_STBY=$GPIO_CAN0_STBY
elif [ "$CAN" = "can1" ]; then
  GPIO_CAN_STBY=$GPIO_CAN1_STBY
else
  echo "Error, invalid can interface specified"
  exit 1
fi

echo "Configuring $CAN for a bitrate of $CAN_BITRATE bits/s"

function finish
{
  ip link set $CAN down
  echo 1 > /sys/class/gpio/gpio$GPIO_CAN_STBY/value
  echo $GPIO_CAN_STBY > /sys/class/gpio/unexport
}
trap finish EXIT

echo $GPIO_CAN_STBY > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$GPIO_CAN_STBY/direction
echo 0 > /sys/class/gpio/gpio$GPIO_CAN_STBY/value

ip link set $CAN type can bitrate $CAN_BITRATE
ip link set $CAN up

sudo -u fio ifconfig $CAN

docker run -it \
   -u 0 \
   --privileged \
   --network host \
   pika_spark_docker_build_and_run
