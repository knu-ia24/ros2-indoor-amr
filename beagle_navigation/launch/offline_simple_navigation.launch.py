import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable



def generate_launch_description():

    rviz_enabled = LaunchConfiguration('launch_rviz', default='false')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('beagle_bringup'), 'config', 'simple_offline_bringup.rviz')],
        condition=IfCondition(rviz_enabled),
    )
    
    '''nav2 = Node(
    	package='nav2_bringup',
    	executable='bringup',
    	name='navigation2',
    	output='screen',
    	parameters=[os.path.join(get_package_share_directory('beagle_navigation'), 'params', 'nav2_params.yaml')]
	)'''


    

    planner = Node(
        package='beagle_navigation',
        executable='astar_path_planner',
    )

    tracker = Node(
        package='beagle_navigation',
        executable='pure_pursuit_path_tracker'
    )


    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz'),
        SetEnvironmentVariable('OFFLINE_NAVIGATION_ACTIVE', 'true'),
        #nav2,
        rviz,
        planner,
        tracker
    ])
