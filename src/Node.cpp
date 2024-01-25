/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <viper/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("viper_node")
, _node_heap{}
, _node_hdl{_node_heap.data(),
            _node_heap.size(),
            [this] () { return micros(); },
            [this] (CanardFrame const & frame) { return _can_mgr->transmit(frame); },
            cyphal::Node::DEFAULT_NODE_ID,
            CYPHAL_TX_QUEUE_SIZE,
            CYPHAL_RX_QUEUE_SIZE,
            cyphal::Node::DEFAULT_MTU_SIZE}
, _node_mtx{}
, _node_start{std::chrono::steady_clock::now()}
, _teleop_qos_profile
{
  rclcpp::KeepLast(10),
  rmw_qos_profile_sensor_data
}
, _teleop_sub_options{}
, _teleop_sub{}
, _target_linear_velocity_x{0. * m/s}
, _target_linear_velocity_y{0. * m/s}
, _target_linear_velocity_z{0. * m/s}
, _target_angular_velocity_x{0. * rad/s}
, _target_angular_velocity_y{0. * rad/s}
, _target_angular_velocity_z{0. * rad/s}
{
  init_cyphal_heartbeat();
  init_cyphal_node_info();

  declare_parameter("can_iface", "can0");
  declare_parameter("can_node_id", 100);

  RCLCPP_INFO(get_logger(),
              "configuring CAN2233 bus:\n\tDevice: %s\n\tNode Id: %ld",
              get_parameter("can_iface").as_string().c_str(),
              get_parameter("can_node_id").as_int());

  _node_hdl.setNodeId(get_parameter("can_node_id").as_int());

  _can_mgr = std::make_unique<CanManager>(
    get_logger(),
    get_parameter("can_iface").as_string(),
    [this](CanardFrame const & frame)
    {
      std::lock_guard<std::mutex> lock(_node_mtx);
      _node_hdl.onCanFrameReceived(frame);
    });

  _node_loop_timer = create_wall_timer(NODE_LOOP_RATE,
                                       [this]()
                                       {
                                         std::lock_guard <std::mutex> lock(_node_mtx);
                                         _node_hdl.spinSome();
                                       });

  init_teleop_sub();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  _cyphal_demo_pub = _node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>(CYPHAL_DEMO_PORT_ID, 1*1000*1000UL);

  _setpoint_velocity_pub = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID, 1*1000*1000UL);

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_cyphal_heartbeat()
{
  _cyphal_heartbeat_pub = _node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);

  _cyphal_heartbeat_timer = create_wall_timer(CYPHAL_HEARTBEAT_PERIOD,
                                              [this]()
                                              {
                                                uavcan::node::Heartbeat_1_0 msg;

                                                auto const now = std::chrono::steady_clock::now();

                                                msg.uptime = std::chrono::duration_cast<std::chrono::seconds>(now - _node_start).count();
                                                msg.health.value = uavcan::node::Health_1_0::NOMINAL;
                                                msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
                                                msg.vendor_specific_status_code = 0;

                                                {
                                                  std::lock_guard <std::mutex> lock(_node_mtx);
                                                  _cyphal_heartbeat_pub->publish(msg);

                                                }
                                              });
}

void Node::init_cyphal_node_info()
{
  _cyphal_node_info = _node_hdl.create_node_info(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    std::array<uint8_t, 16>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
    /* saturated uint8[<=50] name */
    "107-systems.viper"
  );
}

CanardMicrosecond Node::micros()
{
  auto const now = std::chrono::steady_clock::now();
  auto const node_uptime = (now - _node_start);
  return std::chrono::duration_cast<std::chrono::microseconds>(node_uptime).count();
}

void Node::init_teleop_sub()
{
  declare_parameter("teleop_topic", "cmd_vel_head");
  declare_parameter("teleop_topic_deadline_ms", 100);
  declare_parameter("teleop_topic_liveliness_lease_duration", 1000);

  auto const teleop_topic = get_parameter("teleop_topic").as_string();
  auto const teleop_topic_deadline = std::chrono::milliseconds(get_parameter("teleop_topic_deadline_ms").as_int());
  auto const teleop_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("teleop_topic_liveliness_lease_duration").as_int());

  _teleop_qos_profile.deadline(teleop_topic_deadline);
  _teleop_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _teleop_qos_profile.liveliness_lease_duration(teleop_topic_liveliness_lease_duration);

  _teleop_sub_options.event_callbacks.deadline_callback =
    [this, teleop_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            teleop_topic.c_str(), event.total_count, event.total_count_change);

      _target_linear_velocity_x = 0. * m/s;
      _target_linear_velocity_y = 0. * m/s;
      _target_linear_velocity_z = 0. * m/s;

      _target_angular_velocity_x = 0. * rad/s;
      _target_angular_velocity_y = 0. * rad/s;
      _target_angular_velocity_z = 0. * rad/s;
    };

  _teleop_sub_options.event_callbacks.liveliness_callback =
    [this, teleop_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", teleop_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", teleop_topic.c_str());

        _target_linear_velocity_x = 0. * m/s;
        _target_linear_velocity_y = 0. * m/s;
        _target_linear_velocity_z = 0. * m/s;

        _target_angular_velocity_x = 0. * rad/s;
        _target_angular_velocity_y = 0. * rad/s;
        _target_angular_velocity_z = 0. * rad/s;
      }
    };

  _teleop_sub = create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic,
    _teleop_qos_profile,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _target_linear_velocity_x = static_cast<double>(msg->linear.x) * m/s;
      _target_linear_velocity_y = static_cast<double>(msg->linear.y) * m/s;
      _target_linear_velocity_z = static_cast<double>(msg->linear.z) * m/s;

      _target_angular_velocity_x = static_cast<double>(msg->angular.x) * rad/s;
      _target_angular_velocity_y = static_cast<double>(msg->angular.y) * rad/s;
      _target_angular_velocity_z = static_cast<double>(msg->angular.z) * rad/s;
    },
    _teleop_sub_options);
}

void Node::ctrl_loop()
{
  /* TODO: implement me ... */
  static int8_t demo_cnt = 0;
  uavcan::primitive::scalar::Integer8_1_0 const demo_msg{demo_cnt};
  _cyphal_demo_pub->publish(demo_msg);
  

  zubax::primitive::real16::Vector4_1_0 const motor_msg{10.0, 100.0, 10.0, 10.0};
  _setpoint_velocity_pub->publish(motor_msg);

  


//RCLCPP_INFO(get_logger(), "%s inusha bee.", get_name());/

  demo_cnt++;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
