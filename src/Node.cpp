/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <viper/Node.h>
#include <rcl_interfaces/msg/parameter_event.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("viper_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
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
, _teleop_qos_profile{rclcpp::KeepLast(10)}
, _teleop_sub_options{}
, _teleop_sub{}
, _target_linear_velocity{}  // Zero target linear velocity (PS3 controller)
, _target_angular_velocity{}  // Zero target angular velocity (PS3 controller)
, _armed(false)
, _joy_enable_button_index(0)
, _imu_qos_profile{rclcpp::KeepLast(10), rmw_qos_profile_sensor_data}
, _attitude_target(1.0f, 0.0f, 0.0f, 0.0f)  // Initialize to level attitude (identity quaternion)
, _yaw_target(NAN)
, _thrust_target(0.0f)
, _acro_mode(false)
{
  init_cyphal_heartbeat();

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

  // Initialize the state estimator object (estimator can be interchanged here)
  _estimator = std::make_unique<ExternalEstimator>();

  declare_parameter("enable_button", 0);
  _joy_enable_button_index = get_parameter("enable_button").as_int();

  _parameter_event_sub = create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events",
    rclcpp::QoS(10),
    [this](rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
      on_parameter_event(event);
    });

  init_teleop_sub();
  init_joy_sub();
  init_imu_sub();

  // Initialize attitude control parameters
  declare_control_parameters();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  _setpoint_velocity_pub = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID, 1*1000*1000UL);

  // Create ROS publisher for Gazebo motor commands (to be bridged to Ignition)
  auto const gazebo_topic = "motor_speed";
  _gazebo_motor_pub = create_publisher<actuator_msgs::msg::Actuators>(gazebo_topic, rclcpp::QoS(10));


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

CanardMicrosecond Node::micros()
{
  auto const now = std::chrono::steady_clock::now();
  auto const node_uptime = (now - _node_start);
  return std::chrono::duration_cast<std::chrono::microseconds>(node_uptime).count();
}

void Node::init_teleop_sub()
{
  // Declare default parameters for teleop topic and QoS settings
  declare_parameter("teleop_topic", "cmd_vel");
  declare_parameter("teleop_topic_deadline_ms", 100);
  declare_parameter("teleop_topic_liveliness_lease_duration", 1000);

  // Use parameters declared in viper-quad.py to configure subscriptions
  auto const teleop_topic = get_parameter("teleop_topic").as_string();
  auto const teleop_topic_deadline = std::chrono::milliseconds(get_parameter("teleop_topic_deadline_ms").as_int());
  auto const teleop_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("teleop_topic_liveliness_lease_duration").as_int());

  _teleop_qos_profile.deadline(teleop_topic_deadline);
  _teleop_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
  _teleop_qos_profile.liveliness_lease_duration(teleop_topic_liveliness_lease_duration);

  // Called when deadline missed, though currently set to 0ms, i.e. no deadline. Currently, this callback will never trigger
  _teleop_sub_options.event_callbacks.deadline_callback =
    [this, teleop_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            teleop_topic.c_str(), event.total_count, event.total_count_change);

      _target_linear_velocity.reset();
      _target_angular_velocity.reset();

      // Reset attitude and thrust targets to safe values
      _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
      _thrust_target = 0.0f;  // No thrust
    };

  // Called when teleop publisher stops asserting liveliness
  _teleop_sub_options.event_callbacks.liveliness_callback =
    [this, teleop_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        // At least one node is publishing to teleop topic
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", teleop_topic.c_str());
      }
      else
      {
        // No nodes publishing to teleop topic, i.e. joystick node not running / disconnected
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", teleop_topic.c_str());

        _target_linear_velocity.reset();
        _target_angular_velocity.reset();

        // Reset attitude and thrust targets to safe values
        _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
        _thrust_target = 0.0f;  // No thrust
      }
    };

  // Create subscription to Twist messages from joystick  
  _teleop_sub = create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic,
    _teleop_qos_profile,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      // Only process inputs if motors are enabled
      if (!_armed) {
        // Disarm motors when not enabled
        _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
        _thrust_target = 0.0f;  // No thrust
        return;
      }

      // Store velocity commands (could just use an assignment method in Vector class...)
      _target_linear_velocity.x = static_cast<double>(msg->linear.x);
      _target_linear_velocity.y = static_cast<double>(msg->linear.y);
      _target_linear_velocity.z = static_cast<double>(msg->linear.z);

      _target_angular_velocity.x = static_cast<double>(msg->angular.x);
      _target_angular_velocity.y = static_cast<double>(msg->angular.y);
      _target_angular_velocity.z = static_cast<double>(msg->angular.z);

      // Note: _attitude_target is computed in ctrl_loop() via update_attitude_target()

      // Map thrust: linear.z velocity (up/down) to throttle [0, 1]
      _thrust_target = std::clamp(0.5f + static_cast<float>(msg->linear.z) * 0.5f, 0.0f, 1.0f);
    },
    _teleop_sub_options);
}

void Node::init_joy_sub()
{
  _joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::QoS(10),
    [this](sensor_msgs::msg::Joy::SharedPtr const msg)
    {
      // Check enable button (whatever enable_button is set to in joystick_params.yaml)
      const int enable_button_index = _joy_enable_button_index;
      // Bounds check on button index
      if (enable_button_index >= 0 && msg->buttons.size() > static_cast<size_t>(enable_button_index)) {
        bool enable_pressed = (msg->buttons[enable_button_index] == 1);
        _armed = enable_pressed;

        // If deadman switch released, reset target velocities/attitude and reset controllers
        if (!enable_pressed) {
          // Targets from joystick
          _target_linear_velocity.reset();
          _target_angular_velocity.reset();

          // Targets used in ctrl loop
          _thrust_target = 0.0f;
          _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
          _yaw_target = NAN;

          _attitude_controller.reset();
          _rate_controller.reset();

          _estimator->reset(); // Flix would continue to perform estimation...
        }
      }
  });
}

void Node::init_imu_sub()
{
  declare_parameter("imu_topic", "imu");
  declare_parameter("imu_topic_deadline_ms", 100);
  declare_parameter("imu_topic_liveliness_lease_duration", 1000);

  auto const imu_topic = get_parameter("imu_topic").as_string();
  auto const imu_topic_deadline = std::chrono::milliseconds(get_parameter("imu_topic_deadline_ms").as_int());
  auto const imu_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("imu_topic_liveliness_lease_duration").as_int());

  _imu_qos_profile.deadline(imu_topic_deadline);
  // _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  // Likely will need to setup some sort of liveliness QOS later
  _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
  _imu_qos_profile.liveliness_lease_duration(imu_topic_liveliness_lease_duration);

  _imu_sub_options.event_callbacks.deadline_callback =
    [this, imu_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            imu_topic.c_str(), event.total_count, event.total_count_change);
    };

  _imu_sub_options.event_callbacks.liveliness_callback =
    [this, imu_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", imu_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", imu_topic.c_str());
      }
    };

  _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    _imu_qos_profile,
    [this](sensor_msgs::msg::Imu::SharedPtr const msg)
    {
      _estimator->process(*msg);
    },
    _imu_sub_options);
}

void Node::ctrl_loop()
{

  // Will not publish anything when enable button not pressed
  if (!_armed) {
    // Must explicitly tell the Gazebo plugin that motors are off
    if (_gazebo_motor_pub) {
      actuator_msgs::msg::Actuators gz_msg;
      gz_msg.velocity = {0.0, 0.0, 0.0, 0.0};
      // Timestamp when motor command is sent
      gz_msg.header.stamp = get_clock()->now();
      _gazebo_motor_pub->publish(gz_msg);
    }
    return;
  }
  
  // This only fetches latest estimate. The estimator is decoupled
  // and constantly running while new IMU data coming in 
  auto const & est = _estimator->estimate();
  if (!est.valid) return;

  // BNO085 ROS driver needs to be ported first before we can do anything with this
  // See how bad the measurement ages are before implementing first order forward projection
  // If measurement age really large, risks overcorrection, essentially adding energy into system
  auto const measurement_age = _estimator->measurement_age();
  (void) measurement_age;
  
  Quaternion attitude_current = est.orientation;
  Vector     rate_current     = est.angular_rate;

  // Update attitude target based on yaw integration and roll/pitch demands
  update_attitude_target(attitude_current);

  // === Cascaded PID Control Loop ===

  // Level 1: Attitude Controller or acro rate control
  Vector rate_target;
  if (_acro_mode) {
    // radians(180) is max rate
    rate_target = Vector(
        -static_cast<float>(_target_linear_velocity.y * radians(180)),
        static_cast<float>(_target_linear_velocity.x * radians(180)),
        -static_cast<float>(_target_angular_velocity.z * static_cast<float>(YAWRATE_MAX))
    );
  } else {
    rate_target = _attitude_controller.update(attitude_current,
          _attitude_target, _rates_extra);
  }

  // Level 2: Rate Controller
  // Inputs: target rates, current rates
  // Output: target torques
  Vector torque_target = _rate_controller.update(rate_target, rate_current);

  // Motor Mixer
  // Inputs: thrust, torques
  // Output: 4 normalized motor commands, i.e. [0, 1]
  auto motor_commands = _motor_mixer.mix(_thrust_target, torque_target);

  // Scale motor commands to appropriate range
  // WARNING!! This must be set to something smaller if running on actual hardware
  const float MOTOR_SCALE = 1000.0f;
  // We must offset the commands by at least 10 rad/s otherwise ESC will think they're stalled
  const float LOWER_STALL_LIMIT = 10.0f;
  
  zubax::primitive::real16::Vector4_1_0 motor_msg{
    motor_commands[0] * MOTOR_SCALE + LOWER_STALL_LIMIT,
    motor_commands[1] * MOTOR_SCALE + LOWER_STALL_LIMIT,
    motor_commands[2] * MOTOR_SCALE + LOWER_STALL_LIMIT,
    motor_commands[3] * MOTOR_SCALE + LOWER_STALL_LIMIT
  };

  // Publish Cyphal motor commands to all 4 motors
  _setpoint_velocity_pub->publish(motor_msg);

  // Publish to ROS topic as well so ros_gz_bridge can forward to Gazebo
  if (_gazebo_motor_pub) {
    actuator_msgs::msg::Actuators gz_msg;

    gz_msg.velocity = {
      static_cast<float>((motor_commands[0] * MOTOR_SCALE + LOWER_STALL_LIMIT)),
      static_cast<float>((motor_commands[1] * MOTOR_SCALE + LOWER_STALL_LIMIT)),
      static_cast<float>((motor_commands[2] * MOTOR_SCALE + LOWER_STALL_LIMIT)),
      static_cast<float>((motor_commands[3] * MOTOR_SCALE + LOWER_STALL_LIMIT)),
    };

    // Timestamp when motor command is sent
    gz_msg.header.stamp = get_clock()->now();
    _gazebo_motor_pub->publish(gz_msg);
  }

  // Log controller state
  if (false) {  // Manually set to true for verbose logging
    Vector attitude_euler = attitude_current.toEuler();
    RCLCPP_INFO(get_logger(),
      "Attitude (deg): roll=%.1f pitch=%.1f yaw=%.1f | "
      "Rates (rad/s): x=%.2f y=%.2f z=%.2f | "
      "Motors: [%.2f, %.2f, %.2f, %.2f]",
      attitude_euler.x * 180 / M_PI, attitude_euler.y * 180 / M_PI, attitude_euler.z * 180 / M_PI,
      rate_current.x, rate_current.y, rate_current.z,
      motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]
    );
  }
}


void Node::declare_control_parameters()
{
  // Attitude values
  declare_parameter("attitude_roll_p", 6.0);
  declare_parameter("attitude_roll_i", 0.01);
  declare_parameter("attitude_roll_d", 0.001);

  declare_parameter("attitude_pitch_p", 6.0);
  declare_parameter("attitude_pitch_i", 0.01);
  declare_parameter("attitude_pitch_d", 0.001);

  declare_parameter("attitude_yaw_p", 0.5);
  declare_parameter("attitude_yaw_i", 0.05);
  declare_parameter("attitude_yaw_d", 0.001);

  declare_parameter("attitude_roll_damping", 0.9);
  declare_parameter("attitude_pitch_damping", 0.9);
  declare_parameter("acro_mode", false);

  //Rate values
  declare_parameter("rate_roll_p", 0.05);
  declare_parameter("rate_roll_i", 0.2);
  declare_parameter("rate_roll_d", 0.001);

  declare_parameter("rate_pitch_p", 0.35);
  declare_parameter("rate_pitch_i", 0.4);
  declare_parameter("rate_pitch_d", 0.005);

  declare_parameter("rate_yaw_p", 0.5);
  declare_parameter("rate_yaw_i", 0.05);
  declare_parameter("rate_yaw_d", 0.001);

  declare_parameter("rate_integral_windup", 0.3);
  declare_parameter("rate_derivative_filter_alpha", 0.2);

  // Velocity to attitude mapping
  declare_parameter("max_tilt_angle", radians(15));

  // Load initial parameter values
  load_parameters();
}

void Node::load_parameters()
{
  _joy_enable_button_index = get_parameter("enable_button").as_int();

  // Update attitude controller gains
  _attitude_controller.set_gains(
    static_cast<float>(get_parameter("attitude_roll_p").as_double()),
    static_cast<float>(get_parameter("attitude_roll_i").as_double()),
    static_cast<float>(get_parameter("attitude_roll_d").as_double()),
    static_cast<float>(get_parameter("attitude_pitch_p").as_double()),
    static_cast<float>(get_parameter("attitude_pitch_i").as_double()),
    static_cast<float>(get_parameter("attitude_pitch_d").as_double()),
    static_cast<float>(get_parameter("attitude_yaw_p").as_double()),
    static_cast<float>(get_parameter("attitude_yaw_i").as_double()),
    static_cast<float>(get_parameter("attitude_yaw_d").as_double()),
    static_cast<float>(get_parameter("attitude_roll_damping").as_double()),
    static_cast<float>(get_parameter("attitude_pitch_damping").as_double())
  );

  // Update rate controller gains
  _rate_controller.set_gains(
    static_cast<float>(get_parameter("rate_roll_p").as_double()),
    static_cast<float>(get_parameter("rate_roll_i").as_double()),
    static_cast<float>(get_parameter("rate_roll_d").as_double()),
    
    static_cast<float>(get_parameter("rate_pitch_p").as_double()),
    static_cast<float>(get_parameter("rate_pitch_i").as_double()),
    static_cast<float>(get_parameter("rate_pitch_d").as_double()),
    
    static_cast<float>(get_parameter("rate_yaw_p").as_double()),
    static_cast<float>(get_parameter("rate_yaw_i").as_double()),
    static_cast<float>(get_parameter("rate_yaw_d").as_double())
  );

  // Update rate controller PID windup and filter settings
  _rate_controller.get_roll_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());
  _rate_controller.get_pitch_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());
  _rate_controller.get_yaw_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());

  _acro_mode = get_parameter("acro_mode").as_bool();

    // !TODO: Add parameter changes to alpha value. It's currently skipped now
}

void Node::update_attitude_target(const Quaternion& attitude_current)
{
  // Feedforward yaw: positive stick = clockwise in FLU => invert sign
  float controlYaw = static_cast<float>(_target_angular_velocity.z);
  // Ignore yaw deadzone
  if (std::abs(controlYaw) < 0.1f) controlYaw = 0.0f;
  // If drone not armed, yaw control non-zero (user push on yaw stick) or uninitialized, then set heading to current yaw
  if (!_armed || controlYaw != 0.0f || std::isnan(_yaw_target)) {
    _yaw_target = attitude_current.getYaw();
  }
  
  // Log yaw target
  // RCLCPP_INFO(get_logger(), "Yaw Target: %.3f | controlYaw: %.3f",_yaw_target * 180 / PI, controlYaw);

  // Feedforward yaw rate
  _rates_extra = Vector(0.0f, 0.0f, controlYaw * static_cast<float>(YAWRATE_MAX));

  // Compute roll/pitch targets from normalized stick inputs
  float max_tilt = static_cast<float>(get_parameter("max_tilt_angle").as_double());
  float pitch_target = 0.0f;
  float roll_target = 0.0f;
  if (std::abs(_target_linear_velocity.x) > 1e-3 || std::abs(_target_linear_velocity.y) > 1e-3) {
    float vz_max = 1.0f;  // Expected max velocity in m/s
    pitch_target = std::clamp(static_cast<float>(_target_linear_velocity.x) / vz_max, -1.0f, 1.0f) * max_tilt;
    roll_target = -std::clamp(static_cast<float>(_target_linear_velocity.y) / vz_max, -1.0f, 1.0f) * max_tilt;
  }

  // Update attitude target with integrated yaw and computed roll/pitch
  _attitude_target = Quaternion::fromEuler(Vector(roll_target, pitch_target, _yaw_target));
}

void Node::on_parameter_event(rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // Skip if not from this node
  if (event->node != "/viper/viper")
  {
    return;
  }

  auto const parameter_matches = [](const auto & param) -> bool
  {
    return param.name == "enable_button" ||
           param.name == "attitude_roll_p" ||
           param.name == "attitude_roll_i" ||
           param.name == "attitude_roll_d" ||
           param.name == "attitude_pitch_p" ||
           param.name == "attitude_pitch_i" ||
           param.name == "attitude_pitch_d" ||
           param.name == "attitude_yaw_p" ||
           param.name == "attitude_yaw_i" ||
           param.name == "attitude_yaw_d" ||
           param.name == "attitude_roll_damping" ||
           param.name == "attitude_pitch_damping" ||
           param.name == "acro_mode" ||
           param.name == "rate_roll_p" ||
           param.name == "rate_roll_i" ||
           param.name == "rate_roll_d" ||
           param.name == "rate_pitch_p" ||
           param.name == "rate_pitch_i" ||
           param.name == "rate_pitch_d" ||
           param.name == "rate_yaw_p" ||
           param.name == "rate_yaw_i" ||
           param.name == "rate_yaw_d" ||
           param.name == "rate_integral_windup" ||
           param.name == "rate_derivative_filter_alpha";
  };

    for (auto const & p : event->changed_parameters)
    {
      if (parameter_matches(p))
      {
        load_parameters();
        RCLCPP_INFO(get_logger(), "Parameter event loaded from %s", event->node.c_str());
        return;
      }
    }
  }  
}