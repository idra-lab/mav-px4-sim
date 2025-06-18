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

import xml.etree.ElementTree as ET

def obatin_real_time_factor(world_path):
	tree = ET.parse(world_path)
	root = tree.getroot()

	# Find the <real_time_factor> element
	physics = root.find('.//physics')
	if physics is not None:
		real_time_factor = physics.find('real_time_factor')
		if real_time_factor is not None:
			return real_time_factor.text
		else:
			print("No <real_time_factor> element found.")

def add_all_actions(ld, actions):
	for action in actions:
		
		ld.add_action(action)

def generate_launch_description():

	airframe = LaunchConfiguration("airframe")
	ddsport = LaunchConfiguration("ddsport")
	gz_world_file = LaunchConfiguration("gz_world_file")
	gz_world = LaunchConfiguration("gz_world")

	ld = LaunchDescription()


	airframe_launch_arg = DeclareLaunchArgument(
		'airframe', default_value='gz_x500_realsense'
	)

	gazebo_world_launch_arg = DeclareLaunchArgument(
		'gz_world_file', default_value='easy_forest.sdf'
	)
	
	gazebo_name_launch_arg = DeclareLaunchArgument(
		'gz_world', default_value='easy_forest'
	)
  
	ddsport_launch_arg = DeclareLaunchArgument(
		'ddsport', default_value='8888'
	)

	launch_args = [airframe_launch_arg,
					gazebo_world_launch_arg,
					gazebo_name_launch_arg,
					ddsport_launch_arg]
	
	add_all_actions(ld, launch_args)

	# TODO: Add launch arguments from terminal such as airframe name and world name and set PX4_GZ_MODEL_POSE to specify the spawn position
	
	# set_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value="/usr/share/gz/gz-sim8/"
    # )

	set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH'), ':/usr/share/gz/gz-sim8/'])


	set_plugin_path = SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH',value=[EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH'), ':/opt/ros/humble/lib'])

	# remove_gps = SetEnvironmentVariable(name='EKF2_GPS_CTRL', value=[EnvironmentVariable('EKF2_GPS_CTRL'), '0'])
	
	set_pose = SetEnvironmentVariable(
		name='PX4_GZ_MODEL_POSE',
		value='0 0 0.01 0 0 1.57'
		# value='0 0 0 0 0 0'
	)

	uxrce_dds_synct_env = SetEnvironmentVariable(
		'UXRCE_DDS_SYNCT', '0'
	)

	set_sim_speed = SetEnvironmentVariable(
		name='PX4_SIM_SPEED_FACTOR',
		value='1'
	)

	px4_sim_model_env = SetEnvironmentVariable(
		'PX4_SIM_MODEL', airframe
	)
	gz_standalone_env = SetEnvironmentVariable(
		'PX4_GZ_STANDALONE', "1"
	)



	env_vars_set = [
		set_resource_path,
		set_plugin_path,
		# remove_gps,
		set_pose, 
		uxrce_dds_synct_env, 
		set_sim_speed, 
		px4_sim_model_env]
	
	# ld.add_action(env_vars_set)
	add_all_actions(ld, env_vars_set)

	

	

	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	pkg_project_description = get_package_share_directory('drone_description')
	pkg_traj = get_package_share_directory('traj')

	sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_realsense', 'model.sdf')

	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()

	drone_gazebo_dir = get_package_share_directory('drone_gazebo')
	px4_src_dir =  os.path.expanduser('~/PX4-Autopilot')

	

	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': [PathJoinSubstitution([
				drone_gazebo_dir,
				'worlds',
				gz_world_file, 
			]),
			' -r'],
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)

	ld.add_action(gz_sim)

	# RTF = obatin_real_time_factor(os.path.join(drone_gazebo_dir, 'worlds', gazebo_world_launch_arg))

	
	
	# # Start the simulation 
	# sim_start = ExecuteProcess(
	# 	cmd=[
	# 		'gz', 'service', '-s', 
	# 		'/world/easy_forest/control',  
	# 		'--reqtype', 'gz.msgs.WorldControl', 
	# 		'--reptype', 'gz.msgs.Boolean', 
	# 		'--timeout', '2000', 
	# 		'--req', 'pause: false'
	# 	],
	# 	output='screen'
	# )

	""" 
	
	BRIDGE
	  
	 """

	# Bridge
	image_bridge = Node(
		package='ros_gz_image',
		executable='image_bridge',
		arguments=['rgbd_camera/image', 'rgbd_camera/depth_image'],
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

	BRIDGING_NODES = [
		image_bridge,
		ros_gz_bridge
	 ]
	
	add_all_actions(ld, BRIDGING_NODES)


	
	use_sim_time_setter = SetParameter(name='use_sim_time', value=True)

	

	px4_sim_cmd = ExecuteProcess(
		
		cmd=[
			'gnome-terminal',
			'--',
			"build/px4_sitl_default/bin/px4",
			"-d",
			"-s",
			"etc/init.d-posix/rcS",
			"build/px4_sitl_default/etc"
		],
		cwd = px4_src_dir,
		output="screen"
	)

	QGC_cmd = ExecuteProcess(
		cmd = ['gnome-terminal', '--', './QGroundControl.AppImage'	],
		cwd =  os.path.expanduser('~'),
		output='screen'
	)

	dds_cmd = ExecuteProcess(
		cmd=['gnome-terminal', '--', "MicroXRCEAgent", "udp4", "-p", ddsport],
		cwd=os.getcwd(),
		output='screen'
	)

	px4_software_launch = [
		px4_sim_cmd,
		QGC_cmd,
		dds_cmd,
	]

	# ld.add_action(px4_software_launch)
	add_all_actions(ld, px4_software_launch)



	""" 
	
	
	Trafo Nodes
	 
	
	"""

	camera_optical_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',                # translation x y z
            '0','0','0',    # rotation in RPY (rad): 90°, 0°, 90°
            'camera_color_optical_frame', # child frame
            'x500_realsense/realsense_d435/base_link/realsense_d435',               # parent frame
        ]
    )

	slam_map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_frame_publisher',
		arguments=[
			'0', '0', '0',                # translation x y z
            '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
			'slam_map', 
			'map'
			],
		output='screen'
	)


	px4_tf_node = Node(
		package='traj',
		executable='px4_tf',
		name='px4_tf',
		prefix='gnome-terminal --tab --',
		output='screen'
	)

	pointcloud_trafo_node = Node(
		package='tf_pcl_pub', 
		executable='tf_pcl_pub_node',
		name= 'pointcloud_trafo_node',
		output='screen'
	)
	
	ground_truth_node = Node(
		package='ground_truth',
		executable='drone_ground_truth',
		name='drone_ground_truth',
		output='screen'
	)

	trafo_nodes = [
		px4_tf_node,
		slam_map_frame_node,
		pointcloud_trafo_node,
		camera_optical_frame_tf,
		ground_truth_node
	]

	# ld.add_action(trafo_nodes)
	add_all_actions(ld, trafo_nodes)
	
	""" 

	RVIZ Visualization Nodes
	
	
	"""
	visualizer_node = Node(
            package='traj',
            namespace='traj',
            executable='visualizer',
            name='visualizer',
            prefix='gnome-terminal --tab --',
        )

	rviz2_node = Node(
		package='rviz2',
		namespace='',
		executable='rviz2',
		name='rviz2',
		prefix='gnome-terminal --tab --',
		arguments=['-d', [os.path.join(pkg_traj, 'resource/visualize.rviz')]]
	)

	rviz_visualization_nodes = [
		visualizer_node,
		rviz2_node
	]

	# ld.add_action(rviz_visualization_nodes)
	add_all_actions(ld, rviz_visualization_nodes)

	""" 

	TRAJECTORY utilities
	 
	
	"""

	takeoff_node = Node(
		package='traj',
		executable='offboard_takeoff',
		name='offboard_takeoff',
		prefix='gnome-terminal --tab --',
		output='screen', 
		parameters = [{'altitude': 1.0}]
	)

	traj_utilities_nodes = [
		takeoff_node,
	]
	# ld.add_action(traj_utilities_nodes)
	add_all_actions(ld, traj_utilities_nodes)

	
	return ld