/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <cyphal++/cyphal++.h>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

#include "CanManager.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::s;
using mp_units::angular::unit_symbols::rad;

namespace viper
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  static size_t constexpr CYPHAL_O1HEAP_SIZE = (cyphal::Node::DEFAULT_O1HEAP_SIZE * 16);
  static size_t constexpr CYPHAL_TX_QUEUE_SIZE = 256;
  static size_t constexpr CYPHAL_RX_QUEUE_SIZE = 256;
  cyphal::Node::Heap<CYPHAL_O1HEAP_SIZE> _node_heap;
  cyphal::Node _node_hdl;
  std::mutex _node_mtx;
  std::chrono::steady_clock::time_point const _node_start;
  std::unique_ptr<CanManager> _can_mgr;
  static std::chrono::milliseconds constexpr NODE_LOOP_RATE{1};
  rclcpp::TimerBase::SharedPtr _node_loop_timer;

  cyphal::Publisher<uavcan::node::Heartbeat_1_0> _cyphal_heartbeat_pub;
  static std::chrono::milliseconds constexpr CYPHAL_HEARTBEAT_PERIOD{1000};
  rclcpp::TimerBase::SharedPtr _cyphal_heartbeat_timer;
  void init_cyphal_heartbeat();

  cyphal::NodeInfo _cyphal_node_info;
  void init_cyphal_node_info();

  CanardMicrosecond micros();

  rclcpp::QoS _teleop_qos_profile;
  rclcpp::SubscriptionOptions _teleop_sub_options;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _teleop_sub;
  quantity<m/s> _target_linear_velocity_x, _target_linear_velocity_y, _target_linear_velocity_z;
  quantity<rad/s> _target_angular_velocity_x, _target_angular_velocity_y, _target_angular_velocity_z;
  void init_teleop_sub();

  static uint16_t constexpr CYPHAL_DEMO_PORT_ID = 1234;
  cyphal::Publisher<uavcan::primitive::scalar::Integer8_1_0> _cyphal_demo_pub;

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
