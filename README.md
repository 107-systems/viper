<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
Viper-Firmware
==============

[![Compile Sketch status](https://github.com/107-systems/Viper-Firmware/workflows/Compile%20Sketch/badge.svg)](https://github.com/107-systems/Viper-Firmware/actions?workflow=Compile+Sketch)

This repository contains the Arduino based firmware for controlling the Viper MAV.

<p align="center">
  <a href="https://github.com/107-systems/Viper-Firmware"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="40%"></a>
</p>

## Repository Overview
#### :floppy_disk: Firmware
| Repository | Description |
|:-:|-|
| [`107-Arduino-MCP2515`](https://github.com/107-systems/107-Arduino-MCP2515) | Arduino library for interfacing with a MCP2515 CAN transceiver via **SPI** in order to transmit/receive CAN frames. |
| [`107-Arduino-BMP388`](https://github.com/107-systems/107-Arduino-BMP388) | Arduino library for interfacing with a BMP388 sensor via **SPI** in order retrieve barometric pressure for altitude estimation. |
| [`107-Arduino-TMF8801`](https://github.com/107-systems/107-Arduino-TMF8801) | Arduino library for interfacing with a TMF8801 sensor via **I2C** in order to determine distance to ground when landing. |
| [`107-Arduino-NMEA`](https://github.com/107-systems/107-Arduino-NMEA) | Arduino library for interfacing with a PA1010D GPS (MTK3333 chipset) sensor via **UART** in order to obtain GPS information during outdoor flight. |
| [`107-Arduino-FLIR-Lepton`](https://github.com/107-systems/107-Arduino-FLIR-Lepton) | Arduino library for interfacing with the FLIR Lepton sensor via **SPI**. |
| [`107-Arduino-UAVCAN`](https://github.com/107-systems/107-Arduino-UAVCAN) | Arduino library providing an Arduino-style abstraction of [libcanard](https://github.com/UAVCAN/libcanard) for interfacing with the Zubax [Orel 20](https://kb.zubax.com/display/MAINKB/Zubax+Orel+20) ESCs via [UAVCAN](https://uavcan.org/). |
| [`107-Arduino-BNO085`](https://github.com/107-systems/107-Arduino-BNO085) | Arduino library for interfacing with the BNO085 IMU. |
