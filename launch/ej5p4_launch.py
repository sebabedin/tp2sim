from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tp2sim',
            namespace='tp2sim',
            executable='dump_odom',
            name='dump_odom',
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
    ])