"""
This script launches justthe flight controller node. Script to run when on Pika Spark.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ns = 'viper'
    return LaunchDescription([
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
                {'teleop_topic_deadline_ms': 0},
                {'teleop_topic_liveliness_lease_duration': 0},
                {'imu_topic': '/imu'},
                {'imu_topic_deadline_ms': 0},
                {'imu_topic_liveliness_lease_duration': 0},
                PathJoinSubstitution([FindPackageShare('viper'), 'config', 'joystick_params.yaml'])
            ]
        ),
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     namespace=ns,
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[{'device': '/dev/input/js0'}]
        # ),
        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy_node',
        #     namespace=ns,
        #     output='screen',
        #     emulate_tty=True,
        #     # YAML file is copied to the /install dir when building
        #     parameters=[PathJoinSubstitution([FindPackageShare('viper'), 'config', 'joystick_params.yaml'])]  
        # ),
        # Connect Pika Spark's running IMU driver that publishes on /imu
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'echo', 'imu'],
        #     output='screen',
        #     additional_env={'period': '0.1', 'msg': '[100, 100, 100, 100]', 'topic': '113:zubax.primitive.real16.Vector4'}
        # ),

        # Simulated 0 IMU
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '/imu', 'sensor_msgs/msg/Imu',
        #         '{header: {frame_id: "base_link"}, '
        #         'linear_acceleration: {x: 0.0, y: 0.0, z: 9.80665}, '
        #         'angular_velocity: {x: 0.0, y: 0.0, z: 0.0}, '
        #         'orientation: {x: 0.01, y: 0.02, z: 0.0, w: 1.0}}',
        #         '-r', '11'  # Publishes at 10Hz
        #     ],
        #     # output='screen'
        # ),

    ])
