import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('simple_fastlio_localization')
    rviz_config_path = os.path.join(package_path, 'rviz', 'loc.rviz')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Path to the map file'
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='0.0 0.0 0.0 0.0 0.0 0.0 1.0',
            description='Initial pose as x y z qx qy qz qw'
        ),

        # Launch RViz first
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # Add a delay of 3 seconds before launching the localization node
        TimerAction(
            period=3.0,  # Delay in seconds
            actions=[
                Node(
                    package='simple_fastlio_localization',
                    executable='localization_node',
                    name='localization_node',
                    output='screen',
                    parameters=[
                        {'map_file': LaunchConfiguration('map_file')},
                        {'initial_pose': LaunchConfiguration('initial_pose')}
                    ]
                )
            ]
        ),
    ])

