import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import OpaqueFunction
from launch.actions import TimerAction

import yaml
import os



def generate_launch_description():

	
    traj_pkg = get_package_share_directory('traj')


    topics_list_path = os.path.join(
        traj_pkg, 'resource', 'rosbag_topics_list.yaml')


    # Load the YAML file dynamically
    def get_topics_from_yaml(path):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        return config['topics']
    
    topics = get_topics_from_yaml(topics_list_path)

    record_command = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + topics,
        output='screen'
    )

    return LaunchDescription(
        [
            record_command
        ]
	)    