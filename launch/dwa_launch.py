import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to TurtleBot3 Gazebo launch
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # 2. Include the Gazebo World 
    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. Custom DWA Node
    dwa_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner_node',
        name='custom_dwa_planner',
        output='screen',
        parameters=[{'use_sim_time': True}] # Critical for Gazebo sync
    )

    # 4. RViz for Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_world,
        dwa_node,
        rviz_node
    ])
