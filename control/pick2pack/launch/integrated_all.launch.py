from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick2pack',
            executable='final_integrated_node',
            # name='integrated_master',
            namespace='dsr01',
            output='screen'
        )
    ])