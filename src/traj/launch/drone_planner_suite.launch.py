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



def generate_launch_description():

	

	drone_path_follower_node = Node(
		package='traj',
		executable='drone_path_follower',
		name='drone_path_follower',
		prefix='gnome-terminal --tab --',
		output='screen'
	)

	uncertain_planner_pkg = get_package_share_directory('uncertain_planner')

	uncertain_planner = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(uncertain_planner_pkg, 'launch', 'drone_planner.launch.py')),
		
	)

	return LaunchDescription(
	[
	# drone_path_follower_node,
	uncertain_planner,
	]
	)    