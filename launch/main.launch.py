from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_servo'),
                'launch',
                'servo_example.launch.py'
            ])
        ])
    )
    
    controller = Node(
        package='move_with_it',
        executable='controller',
        name='controller'
    )
    
    return LaunchDescription([
        moveit_servo,
        controller
    ])