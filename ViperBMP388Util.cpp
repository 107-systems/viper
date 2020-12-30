/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "ViperBMP388Util.h"

#include <Arduino.h>

#include <ArduinoBMP388.h>

#include "ViperConst.h"

/**************************************************************************************
 * EXTERN DECLARATION
 **************************************************************************************/

extern ArduinoBMP388 bmp388;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void bmp388_spi_select()
{
  digitalWrite(BMP388_CS_PIN, LOW);
}

void bmp388_spi_deselect()
{
  digitalWrite(BMP388_CS_PIN, HIGH);
}

void bmp388_onExternalEvent()
{
  bmp388.onExternalEventHandler();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
