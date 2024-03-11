from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='viper',
            executable='viper_node',
            name='viper',
            namespace='viper',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'can_iface': 'can0'},
                {'can_node_id': 100},
                {'teleop_topic': 'cmd_vel'},
                {'teleop_topic_deadline_ms': 100},
                {'teleop_topic_liveliness_lease_duration': 1000},
                {'imu_topic': 'imu'},
                {'imu_topic_deadline_ms': 100},
                {'imu_topic_liveliness_lease_duration': 1000},
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', 'imu'],
            output='screen'
        )
    ])
