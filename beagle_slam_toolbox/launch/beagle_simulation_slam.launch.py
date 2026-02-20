import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():

    package_name='beagle_slam_toolbox'

    rviz_enabled = LaunchConfiguration('launch_rviz', default='false')

    moved_path_enabled = LaunchConfiguration('moved_path', default='false')

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
                launch_arguments={'use_sim_time': 'true'}.items()
             )

    # Launch them all!s
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=[os.path.join(get_package_share_directory(package_name),
                                        'config','thin_maze_localization.yaml'),''], # 여기서 large_maze_mapping.yaml파일을 다른것으로 바꾸면 다른 형식의 런치파일로 사용할 수 있습니다.
            description='SDF world file'
        ),
        DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('run_moved_path', default_value='false', description='Run Moved Path'),
        slam_toolbox,
        rviz,
        moved_path
    ])
