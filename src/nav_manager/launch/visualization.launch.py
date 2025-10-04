from  nav_manager.launcher_lib import *


def generate_launch_description():

	ld = LaunchDescription()

	visualization_filename = LaunchConfiguration("visualization_filename")

	visualization_filename_launch_arg = DeclareLaunchArgument(
		'visualization_filename', default_value='visualize.rviz'
	)

	nav_manager_dir = get_package_share_directory('nav_manager')

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
		arguments=['-d', os.path.join(nav_manager_dir, 'config', 'visualize.rviz')]

	)

	rviz_visualization_nodes = [
		visualizer_node,
		rviz2_node
	]

	# ld.add_action(rviz_visualization_nodes)
	add_all_actions(ld, rviz_visualization_nodes)


	
	return ld