/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

#ifndef VIPER_FPGA_UTIL_H_
#define VIPER_FPGA_UTIL_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void fpga_spi_select();
void fpga_spi_deselect();

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */

#endif /* VIPER_FPGA_UTIL_H_ */
