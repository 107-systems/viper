/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "ViperMCP2515Util.h"

#include <Arduino.h>

#include <ArduinoMCP2515.h>

#include "ViperConst.h"

/**************************************************************************************
 * EXTERN DECLARATION
 **************************************************************************************/

extern ArduinoMCP2515 mcp2515;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void mcp2515_spi_select()
{
  digitalWrite(MCP2515_CS_PIN, LOW);
}

void mcp2515_spi_deselect()
{
  digitalWrite(MCP2515_CS_PIN, HIGH);
}

void mcp2515_onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
