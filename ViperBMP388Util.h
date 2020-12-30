/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/Viper-Firmware/graphs/contributors
 */

#ifndef VIPER_BMP388_UTIL_H_
#define VIPER_BMP388_UTIL_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void bmp388_spi_select();
void bmp388_spi_deselect();
void bmp388_onExternalEvent();

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */

#endif /* VIPER_BMP388_UTIL_H_ */
