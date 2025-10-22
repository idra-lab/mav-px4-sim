from  nav_manager.launcher_lib import *



def generate_launch_description():
	

	ld = LaunchDescription()

	benchmark_dir = get_package_share_directory('benchmarking')

	airframe = LaunchConfiguration("airframe")
	gz_world_file = LaunchConfiguration("gz_world_file")

	airframe_launch_arg = DeclareLaunchArgument(
		'airframe', default_value='gz_x500_realsense'
	)

	gazebo_world_launch_arg = DeclareLaunchArgument(
		'gz_world_file', default_value='benchmarking.sdf'
	)
	
	gazebo_name_launch_arg = DeclareLaunchArgument(
		'gz_world', default_value='benchmarking'
	)
  
	ddsport_launch_arg = DeclareLaunchArgument(
		'ddsport', default_value='8888'
	)

	launch_args = [airframe_launch_arg,
					gazebo_world_launch_arg,
					gazebo_name_launch_arg,
					ddsport_launch_arg]
	
	add_all_actions(ld, launch_args)


	set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH'), ':/usr/share/gz/gz-sim8/'])

	
	set_plugin_path = SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH',value=[EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH'), ':/opt/ros/humble/lib'])


	print("Drone Pose from config: ", drone_config_instance.pose_str)
	set_pose = SetEnvironmentVariable(
		name='PX4_GZ_MODEL_POSE',
		# value='0 0 0.01 0 0 1.57'
		value= drone_config_instance.pose_str,
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

	set_headless_env = SetEnvironmentVariable(
		name='HEADLESS',
		value='1'
	)



	env_vars_set = [
		set_resource_path,
		set_plugin_path,
		# remove_gps,
		set_pose, 
		uxrce_dds_synct_env, 
		set_sim_speed, 
		px4_sim_model_env, 
		set_headless_env]
	
	# ld.add_action(env_vars_set)
	add_all_actions(ld, env_vars_set)

	
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	pkg_project_description = get_package_share_directory('drone_description')
	pkg_traj = get_package_share_directory('traj')

	sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_realsense', 'model.sdf')

	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()

	drone_gazebo_dir = get_package_share_directory('drone_gazebo')

	

	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': [PathJoinSubstitution([
				drone_gazebo_dir,
				'worlds',
				gz_world_file, 
			]),
			' -r', 
			'-s'
			],
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)

	ld.add_action(gz_sim)

	""" 
	
	BRIDGE
	  
	 """

	nav_manager_dir = get_package_share_directory('nav_manager')
	

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
			'config_file': os.path.join(nav_manager_dir, 'config', 'ros_gz_bridge.yaml'),
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

	  

	
	rotated_map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='drone_pose_correction_frame_publisher',
		arguments=[
			'0', '0', '0',                # translation x y z
			str(drone_config_instance.drone_quat[0]), str(drone_config_instance.drone_quat[1]), str(drone_config_instance.drone_quat[2]), str(drone_config_instance.drone_quat[3]),  # rotation in RPY (rad): -90°, 0°, -90°
			# '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
			# '1, 0, 0, 0',   # rotation in RPY (rad): -90°, 0°, -90°
			'map', 
			'initial_pose_map',
			],
		output='screen'
	)


	slam_map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_frame_publisher',
		arguments=[
			'0', '0', '0',                # translation x y z
			str(drone_config_instance.map_to_slam[0]), str(drone_config_instance.map_to_slam[1]), str(drone_config_instance.map_to_slam[2]), str(drone_config_instance.map_to_slam[3]),   # rotation in RPY (rad): -90°, 0°, -90°
			# '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
			# '1, 0, 0, 0',   # rotation in RPY (rad): -90°, 0°, -90°
			'initial_pose_map',
			'slam_map', 
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
		ground_truth_node, 
		rotated_map_frame_node
	]

	# ld.add_action(trafo_nodes)
	add_all_actions(ld, trafo_nodes)

	"""

	RVIZ Visualization Nodes


	"""

	visualization_filename = LaunchConfiguration("visualization_filename")

	visualization_filename_launch_arg = DeclareLaunchArgument(
		'visualization_filename', default_value='visualize.rviz'
	)

	

	
	
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
		arguments=['-d', os.path.join(benchmark_dir, 'config', 'cube_benchmark', 'config_rviz.rviz')]

	)

	rviz_visualization_nodes = [
		visualizer_node,
		rviz2_node
	]

	# ld.add_action(rviz_visualization_nodes)
	add_all_actions(ld, rviz_visualization_nodes)


	ddsport = LaunchConfiguration("ddsport")

	ddsport_launch_arg = DeclareLaunchArgument(
		'ddsport', default_value='8888'
	)

	px4_source_dir = LaunchConfiguration('px4_source_dir')

	px4_source_dir_launch_arg = DeclareLaunchArgument(
		'px4_source_dir', default_value=os.path.expanduser('~/PX4-Autopilot')
	)

	qground_exe_path = LaunchConfiguration('qground_exe_path')

	qground_exe_path_launch_arg = DeclareLaunchArgument(
		'qground_exe_path', default_value=os.path.expanduser('~/QGroundControl.AppImage')
	)

	add_all_actions(ld, [ddsport_launch_arg, px4_source_dir_launch_arg, qground_exe_path_launch_arg])

	print('PX4 Source Directory: ', px4_source_dir)

	# px4_src_dir =  os.path.expanduser('~/PX4-Autopilot')


	px4_sim_cmd = ExecuteProcess(
		
		cmd=[
			# 'gnome-terminal',
			# '--',
			"build/px4_sitl_default/bin/px4",
			"-d",
			"-s",
			"etc/init.d-posix/rcS",
			"build/px4_sitl_default/etc"
		],
		cwd = px4_source_dir,
		output="screen"
	)

	QGC_cmd = ExecuteProcess(
		cmd = ['gnome-terminal', '--', qground_exe_path	],
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


	delay_duration = LaunchConfiguration('delay_duration', default='5.0')

	delay_duration_launch_arg = DeclareLaunchArgument(
		'delay_duration', default_value='5.0'
	)

	altitude = LaunchConfiguration('altitude', default='3.0')
	altitude_launch_arg = DeclareLaunchArgument(
		'altitude', default_value='3.0'
	)

	""" 

	TRAJECTORY utilities
	 
	
	"""

	takeoff_node = Node(
		package='traj',
		executable='offboard_takeoff',
		name='offboard_takeoff',
		prefix='gnome-terminal --tab --',
		output='screen', 
		parameters = [{'altitude': LaunchConfiguration('altitude')}]
	)

	takeoff_node_delay = TimerAction(
		period=delay_duration,  # delay in seconds
		actions=[takeoff_node]
	)


	traj_utilities_nodes = [
		delay_duration_launch_arg, 
		altitude_launch_arg,
		takeoff_node_delay,
	]

	add_all_actions(ld, traj_utilities_nodes)

	# keep a reference to the original include and replace px4_launch with an event-driven launcher
	# px4_delay_launch = TimerAction(
	# 	period=10.0,  # delay in seconds
	# 	actions=[px4_launch]
	# )

	

	# ld.add_action(px4_delay_launch)

	perception_launcher_path = get_package_share_directory('perception_launcher')

	orbslam3_path = get_package_share_directory('orbslam3')

	perception_sim_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                                    perception_launcher_path,
                                                    'launch',
                                                    'fake_slam.launch.py'
                                                    ])
                            ),

                            # 'src/realsense-ros/realsense2_camera/examples/pointcloud/rs_d455_pointcloud_launch.py')
    )

	# perception_sim_launch = Node(
    #         package='orbslam3',
    #         executable='simulation_rgbd',
    #         name='orb_slam3',
    #         output='screen',
    #         # prefix='gnome-terminal --tab --',
    #         arguments=[
    #             orbslam3_path+ '/vocabulary/ORBvoc.txt',
    #             orbslam3_path+ '/config/simulation/simulation_rgbd.yaml'
    #         ]
    # )

	ld.add_action(perception_sim_launch)

	"""
	Launch planner
	"""

	benchmark_plan = Node(
		package='benchmarking',
		executable='full_smoother_benchmarking_node',
		name='full_smoother_benchmarking_node',
		prefix='gnome-terminal --tab --',
		output='screen', 
		parameters=[
			{
				'configuration_filename': os.path.join(benchmark_dir, 'config', 'benchmark_config.yaml'), 
				'map_path': os.path.join(benchmark_dir, 'resource/maps/cube_benchmark', 'map_creator_file.yaml')

			}
		],

	)

	ld.add_action(benchmark_plan)	

	traj_follower_node = Node(
		package='traj',
		executable='drone_trajectory_follower',
		name='drone_trajectory_follower',
		prefix='gnome-terminal --tab --',
		output='screen', 
	)

	ld.add_action(traj_follower_node)



	return ld