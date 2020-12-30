/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "ViperFpgaUtil.h"

#include <Arduino.h>

#include "ViperConst.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void fpga_spi_select()
{
  digitalWrite(FPGA_CS_PIN, LOW);
}

void fpga_spi_deselect()
{
  digitalWrite(FPGA_CS_PIN, HIGH);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
