<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `viper`
=====================
[![Build Status](https://github.com/107-systems/viper/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/viper/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/viper/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/viper/actions/workflows/spell-check.yml)

Generic ROS based drone flight stack for the [Pika Spark](https://pika-spark.io/).

<p align="center">
  <a href="https://github.com/107-systems/viper"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="40%"></a>
</p>

# Build Instructions
Note: Don't forget to install the [dependencies](https://github.com/107-systems/viper#install-dependencies) (see at the end of this file).  
Removed these dependencies for now...

- See the detailed [build instructions](docs/build.md).


# Running It
```bash
cd colcon_ws
. install/setup.bash
ros2 launch viper [launch-file.py]
```

## Launch file overview
* `viper-quad.py` - run the flight controller node directly on the Pika Spark hardware.
* `viper-sim.py` - run Gazebo simulation locally with the `viper` node, `joy`, `teleop_twist_joy`, and `ros_gz_bridge`. More details on Gazebo implementation [here](gazebo/README.md).
* `viper-cosim.py` - co-simulation setup that runs the joystick/Gazebo side locally while the flight controller remains on the Pika Spark.

## Selecting a CAN Interface
A CAN interface must be set up by running the relevant [script](scripts/) before the flight controller node is run. There are 2 options:
- `vcan`: Setup by running `setup_vcan.sh` in `scripts` directory
- `can`: This can be the physical interface on Pika Spark (`setup_can.sh`) or `slcan` (`source setup_yakut_slcan.sh`) on host PC. Scripts register under `can` for simplicity. For more details on which script to run, see [here](scripts/README.md).

  If you use the Zubax Babel as a USB-CAN device you need to run `. setup_yakut_slcan.sh` first.


# Interface Documentation
## Published Topics
| Default name  |                                    Type                                     | Description                                                                                     |
|:-------------:|:---------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------|
| `/viper/motor_speed` | [`actuator_msgs/Actuators`](http://docs.ros.org/en/api/actuator_msgs/html/msg/Actuators.html) | Motor command output published for Gazebo/ros_gz_bridge. Same as what is being sent over Cyphal                                       |

### Subscribed Topics
| Topic name    |                                     Type                                      | Description                                                                                     |
|:-------------:|:----------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------|
| `/viper/cmd_vel` | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | Teleop command input topic.|
| `/viper/joy`         | [`sensor_msgs/Joy`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) | Reads raw joystick input used to arm/disarm and read controller axes/buttons.                             |
| `/imu`         | [`sensor_msgs/Imu`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) | IMU measurements consumed by the estimator. External BNO085 ROS 2 node. |
| `/parameter_events` | [`rcl_interfaces/ParameterEvent`](http://docs.ros.org/en/api/rcl_interfaces/html/msg/ParameterEvent.html) | Parameter change notifications for dynamic reconfiguration. |


### Parameters
|                   Name                    | Default  | Description                                                                  |
|:-----------------------------------------:|:--------:|------------------------------------------------------------------------------|
|              `can_iface`                 | `can0`   | CAN interface device used by the flight controller.                          |
|             `can_node_id`                | `100`    | CAN node ID for the Pika Spark on the CAN bus.                              |
|             `enable_button`              | `0`      | Index of the joystick button used to arm/disarm the motors.                 |
|            `teleop_topic`                | `cmd_vel` | Topic name used for teleoperation Twist commands.                           |
|        `teleop_topic_deadline_ms`        |   100    | Deadline in milliseconds within which a teleop message is expected.          |
| `teleop_topic_liveliness_lease_duration` |   1000   | Liveliness lease duration for teleop command publishing.                     |
|               `imu_topic`                | `imu`    | Topic name used for IMU messages consumed by the estimator.                  |
|         `imu_topic_deadline_ms`          |   100    | Deadline in milliseconds within which an IMU message is expected.            |
| `imu_topic_liveliness_lease_duration`    |   1000   | Liveliness lease duration for IMU message publishing.                        |
|         `attitude_roll_p`                | `6.0`    | Attitude controller roll proportional gain.                                  |
|         `attitude_roll_i`                | `0.01`   | Attitude controller roll integral gain.                                       |
|         `attitude_roll_d`                | `0.001`  | Attitude controller roll derivative gain.                                     |
|        `attitude_pitch_p`                | `6.0`    | Attitude controller pitch proportional gain.                                 |
|        `attitude_pitch_i`                | `0.01`   | Attitude controller pitch integral gain.                                      |
|        `attitude_pitch_d`                | `0.001`  | Attitude controller pitch derivative gain.                                    |
|         `attitude_yaw_p`                 | `0.5`    | Attitude controller yaw proportional gain.                                     |
|         `attitude_yaw_i`                 | `0.05`   | Attitude controller yaw integral gain.                                         |
|         `attitude_yaw_d`                 | `0.001`  | Attitude controller yaw derivative gain.                                       |
|             `acro_mode`                  | `false`  | Enable acro-style rate control instead of attitude stabilization.             |
|          `rate_roll_p`                   | `0.05`   | Rate controller roll proportional gain.                                       |
|          `rate_roll_i`                   | `0.2`    | Rate controller roll integral gain.                                            |
|          `rate_roll_d`                   | `0.001`  | Rate controller roll derivative gain.                                          |
|         `rate_pitch_p`                   | `0.35`   | Rate controller pitch proportional gain.                                      |
|         `rate_pitch_i`                   | `0.4`    | Rate controller pitch integral gain.                                           |
|         `rate_pitch_d`                   | `0.005`  | Rate controller pitch derivative gain.                                         |
|          `rate_yaw_p`                    | `0.5`    | Rate controller yaw proportional gain.                                        |
|          `rate_yaw_i`                    | `0.05`   | Rate controller yaw integral gain.                                             |
|          `rate_yaw_d`                    | `0.001`  | Rate controller yaw derivative gain.                                           |
|      `rate_integral_windup`              | `0.3`    | Integral windup limit for rate controllers.                                   |
| `rate_derivative_filter_alpha`           | `0.2`    | Derivative filter alpha used by the rate controllers.                          |
|          `max_tilt_angle`               | `0.261799` | Maximum tilt angle in radians (default 15°).                                 |

### Install dependencies
Only needed if want to re-add `mp-units` dependency. It's removed for now.
* Install `gsl-lite`
```bash
git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `Catch2`
```bash
git clone https://github.com/catchorg/Catch2 && cd Catch2
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `fmt`
```bash
git clone https://github.com/fmtlib/fmt && cd fmt
mkdir build && cd build
cmake -DFMT_TEST=OFF ..
make -j8
sudo make install
```
