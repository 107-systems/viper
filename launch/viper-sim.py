"""
This script should be used for when running simulation entirely on host machine. Launches flight controller node
as well as Gazebo.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ns = 'viper'
    return LaunchDescription([
        # Launch the main flight controller ROS 2 node
        Node(
            package='viper',
            executable='viper_node',
            name='viper',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'can_iface': 'vcan0'},
                {'can_node_id': 100},
                {'teleop_topic': 'cmd_vel'},
                # There's a QoS profile mismatch between teleop_twist_joy_node and viper if these are not turned off
                {'teleop_topic_deadline_ms': 0},
                {'teleop_topic_liveliness_lease_duration': 0},
                {'imu_topic': '/imu'},
                {'imu_topic_deadline_ms': 0},
                {'imu_topic_liveliness_lease_duration': 0},
                PathJoinSubstitution([FindPackageShare('viper'), 'config', 'joystick_params.yaml'])
            ]
        ),
        # Joy node reads in inputs from PS3 controller
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{'device': '/dev/input/js0'}]
        ),
        # Teleop node converts the sensor_msg messages from joy node into geometry_msg Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([FindPackageShare('viper'), 'config', 'joystick_params.yaml'])]
        ),

        # Start Ignition Gazebo with the world for the simulation
        ExecuteProcess(
            cmd=['ign', 'gazebo', PathJoinSubstitution([FindPackageShare('viper'), 'gazebo', 'worlds', 'world.sdf'])],
            output='screen'
        ),
        
        # Bridge ROS topic to Ignition so motor commands reach the model. Also, bridges IMU in Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[
                {"config_file": PathJoinSubstitution([FindPackageShare('viper'), 'gazebo', 'config', 'bridge.yaml'])}
            ]
        ),

    ])
