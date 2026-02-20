import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable

def generate_launch_description():

    package_name='beagle_slam_toolbox'

    rviz_enabled = LaunchConfiguration('launch_rviz', default='true')

    moved_path_enabled = LaunchConfiguration('moved_path', default='false')
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('beagle_bringup'),'launch','beagle_state_publisher_real.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'slam.rviz')],
        condition=IfCondition(rviz_enabled),
    )

    moved_path = Node(
        package='beagle_something_else',
        executable='moved_path_odom',
        condition=IfCondition(moved_path_enabled),
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                launch_arguments={'use_sim_time': 'false'}.items(),
                condition=UnlessCondition(EnvironmentVariable('OFFLINE_NAVIGATION_ACTIVE', default_value='false'))
             )
             

    # Launch them all!s
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=[os.path.join(get_package_share_directory(package_name),
                                        'config','real_slam_mapping.yaml'),''],
            description='SDF world file'
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('run_moved_path', default_value='false', description='Run Moved Path'),
        slam_toolbox,
        rviz,
        rsp,
        moved_path,
    ])
