import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable


def generate_launch_description():

	# TODO: Add launch arguments from terminal such as airframe name and world name and set PX4_GZ_MODEL_POSE to specify the spawn position
	
	# set_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value="/usr/share/gz/gz-sim8/"
    # )

	gazebo_plugin_path = '/opt/ros/humble/lib'

	set_plugin_path = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH',value= [EnvironmentVariable('GAZEBO_PLUGIN_PATH'), gazebo_plugin_path]),

	set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH'), ':/usr/share/gz/gz-sim8/'])
	

	gazebo_world_launch_arg = DeclareLaunchArgument(
		'world', default_value='warehouse.sdf'
	)
  
	
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	pkg_project_description = get_package_share_directory('drone_description')
	pkg_traj = get_package_share_directory('traj')
	# sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_mono_cam', 'model.sdf')
	# sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_depth', 'model.sdf')
	sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_realsense', 'model.sdf')
	# sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_base', 'model.sdf')
	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()

	drone_gazebo_dir = get_package_share_directory('drone_gazebo')

	airframe_value = LaunchConfiguration("airframe")
	ddsport_value = LaunchConfiguration("port")
	gazebo_world_value = LaunchConfiguration("world")
	


	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': PathJoinSubstitution([
				drone_gazebo_dir,
				'worlds',
				gazebo_world_value, 
			]),
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)

	# Bridge
	bridge = Node(
		package='ros_gz_image',
		executable='image_bridge',
		arguments=['camera', 'depth_camera', 'rgbd_camera/image', 'rgbd_camera/depth_image'],
		output='screen'
	)


	# Bridge ROS topics and Gazebo messages for establishing communication
	ros_gz_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		parameters=[{
			'config_file': os.path.join(pkg_traj, 'resource', 'traj_bridge.yaml'),
			# 'qos_overrides./tf_static.publisher.durability': 'transient_local',
		}],
		prefix='gnome-terminal --tab --',
		output='screen'
	)
	
	use_sim_time_setter = SetParameter(name='use_sim_time', value=True)

	
	
	
	depth_camera_pointcloud_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='depth_camera_pointcloud_publisher',
		arguments=['0', '0', '0', '0', '0', '0', 'drone', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'],
		output='screen'
	)


	return LaunchDescription(
	[
	# use_sim_time_setter,
	# uxrce_dds_synct_env,
	set_resource_path,
	set_plugin_path,
	gazebo_world_launch_arg,

	gz_sim,
	bridge,
	ros_gz_bridge, 

	
	]
	)    