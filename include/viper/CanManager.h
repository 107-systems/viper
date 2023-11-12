/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <socketcan.h>

#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include <rclcpp/rclcpp.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class CanManager
{
public:
  typedef std::function<void(CanardFrame const &)> OnCanFrameReceivedFunc;

  CanManager(rclcpp::Logger const logger,
             std::string const & iface_name,
             OnCanFrameReceivedFunc on_can_frame_received);
  ~CanManager();


  bool transmit(CanardFrame const & frame);


private:
  rclcpp::Logger const _logger;
  std::string const IFACE_NAME;
  bool const IS_CAN_FD;
  int _socket_can_fd;
  OnCanFrameReceivedFunc _on_can_frame_received;

  std::atomic<bool> _rx_thread_active;
  std::thread _rx_thread;
  void rx_thread_func();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
