
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

from launch.actions import TimerAction

def add_all_actions(ld, actions):
	for action in actions:
		
		ld.add_action(action)


def generate_launch_description():

	# Setup the launch description
    ##############################################################################
    ld = LaunchDescription()
    
    # Paths to package directories
    ##############################################################################
    pkg_traj = get_package_share_directory('traj')
    perception_launcher_path = get_package_share_directory('perception_launcher')

	
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='true',
        description='Flag to enable hardware mode'
    )
	
    ld.add_action(hardware_arg)
    hardware = LaunchConfiguration('hardware')

    # TF actions
	
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

    px4_tf_node = Node(
		package='traj',
		executable='px4_tf',
		name='px4_tf',
		prefix='gnome-terminal --tab --',
		output='screen',
        parameters = [{'hardware': hardware}]
	)
	
    add_all_actions(ld, [px4_tf_node, slam_map_frame_node])
	


    


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
	
    add_all_actions(ld, [perception_sim_launch])



    # Visualization actions
	
    rviz2_slam = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [pkg_traj + '/resource/drone_hardware.rviz']],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
	
    visualizer_node = Node(
        package='traj',
        namespace='traj',
        executable='visualizer',
        name='visualizer',
        prefix='gnome-terminal --tab --',
        parameters=[{'hardware': hardware}]
    )
	
    add_all_actions(ld, [rviz2_slam, visualizer_node])
	

    # Trajectory utilities actions
    takeoff_node = Node(
		package='traj',
		executable='offboard_takeoff',
		name='offboard_takeoff',
		prefix='gnome-terminal --tab --',
		output='screen', 
		parameters = [{'altitude': 0.5, 'takeoff_speed': 0.5, 'hardware': True}]  # altitude in meters, takeoff_speed in m/s
	)

    takeoff_node_delay = TimerAction(
		period=15.0,  # delay in seconds
		actions=[takeoff_node]
	)


    traj_utilities_nodes = [
		takeoff_node_delay,
	]

    # add_all_actions(ld, traj_utilities_nodes)
	




    return ld
    
    