from  nav_manager.launcher_lib import *



def generate_launch_description():

	ld = LaunchDescription()

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

	return ld

