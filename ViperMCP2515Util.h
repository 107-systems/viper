/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

#ifndef VIPER_MCP2515_UTIL_H_
#define VIPER_MCP2515_UTIL_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void mcp2515_spi_select();
void mcp2515_spi_deselect();
void mcp2515_onExternalEvent();

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */

#endif /* VIPER_MCP2515_UTIL_H_ */
