from  nav_manager.launcher_lib import *



def generate_launch_description():

	ld = LaunchDescription()

	# op_fun_read_yaml = OpaqueFunction(function=read_yaml_config)

	# ld.add_action(op_fun_read_yaml)
	
	airframe = LaunchConfiguration("airframe")
	gz_world_file = LaunchConfiguration("gz_world_file")

	airframe_launch_arg = DeclareLaunchArgument(
		'airframe', default_value='gz_x500_realsense'
	)

	gazebo_world_launch_arg = DeclareLaunchArgument(
		'gz_world_file', default_value='medium_forest.sdf'
	)
	
	gazebo_name_launch_arg = DeclareLaunchArgument(
		'gz_world', default_value='medium_forest'
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


	return ld

	