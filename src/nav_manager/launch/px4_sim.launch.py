from  nav_manager.launcher_lib import *


def generate_launch_description():
    

	ld = LaunchDescription()

	gz_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                                    FindPackageShare('nav_manager'),
                                                    'gz_env.launch.py'
                                                    ])
                            )
    )

	bridge_launch = IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
								PathJoinSubstitution([
													FindPackageShare('nav_manager'),
													'bridging_nodes.launch.py'
													])
							)
	)

	

	trafo_nodes = IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
								PathJoinSubstitution([
													FindPackageShare('nav_manager'),
													'trafo_nodes.launch.py'
													])
							)
	)

	visualization_launch = IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
								PathJoinSubstitution([
													FindPackageShare('nav_manager'),
													'visualization.launch.py'
													])
							)
	)

	px4_launch = IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
								PathJoinSubstitution([
													FindPackageShare('nav_manager'),
													'px4_commands.launch.py'
													])
							)
	)

	takeoff_launch = IncludeLaunchDescription(
							PythonLaunchDescriptionSource(
								PathJoinSubstitution([
													FindPackageShare('traj'),
													'takeoff_delay.launch.py'
													])
							)
	)

	# keep a reference to the original include and replace px4_launch with an event-driven launcher
	px4_delay_launch = TimerAction(
		period=10.0,  # delay in seconds
		actions=[px4_launch]
	)

	

	ld.add_action(gz_launch)
	ld.add_action(bridge_launch)
	ld.add_action(trafo_nodes)
	ld.add_action(visualization_launch)
	ld.add_action(px4_delay_launch)
	ld.add_action(takeoff_launch)

	return ld