from launch import LaunchDescription
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
        {'can_iface' : 'can0'},
        {'can_node_id' : 100},
      ]
    )
  ])
