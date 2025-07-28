
"""Launch nodes for the drone hardware setup."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    

    perception_launcher_path = get_package_share_directory('perception_launcher')

    perception_sim_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                                    perception_launcher_path,
                                                    'launch',
                                                    'drone.launch.py'
                                                    ])
                            )
                            # launch_arguments={'node_name': 'bar'}.items(),

                            # 'src/realsense-ros/realsense2_camera/examples/pointcloud/rs_d455_pointcloud_launch.py')
    )

    return LaunchDescription([
        px4_sim_launch,
        perception_sim_launch,
        
    ])
    
    