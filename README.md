<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
Viper-Firmware
==============

[![Compile Sketch status](https://github.com/107-systems/Viper-Firmware/workflows/Compile%20Sketch/badge.svg)](https://github.com/107-systems/Viper-Firmware/actions?workflow=Compile+Sketch)

This repository contains the Arduino based firmware for controlling the Viper MAV.

<p align="center">
  <a href="https://github.com/107-systems/viper"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="40%"></a>
</p>

## How-to-build/upload
```bash
arduino-cli compile -b arduino:samd:nano_33_iot -v firmware/viper
arduino-cli upload -b arduino:samd:nano_33_iot -v firmware/viper -p /dev/ttyACM0
```
