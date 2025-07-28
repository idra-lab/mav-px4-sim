
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

    # TODO: Add a static transform for the map frame or check if PX4 will deliver one

    slam_to_map_quat = [-0.5, 0.5, -0.5, -0.5]

    slam_map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_frame_publisher',
		arguments=[
			'0', '0', '0',                # translation x y z
            str(slam_to_map_quat[0]), str(slam_to_map_quat[1]), str(slam_to_map_quat[2]), str(slam_to_map_quat[3]),   # rotation in RPY (rad): -90°, 0°, -90°
            # '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
            # '1, 0, 0, 0',   # rotation in RPY (rad): -90°, 0°, -90°
			'slam_map', 
			'map',
			],
		output='screen'
	)

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

    pkg_traj = get_package_share_directory('traj')



    rviz2_slam = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [pkg_traj + '/resource/drone_hardware.rviz']],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )

    return LaunchDescription([
        slam_map_frame_node,
        perception_sim_launch,
        rviz2_slam
    ])
    
    