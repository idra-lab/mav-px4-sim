from  nav_manager.launcher_lib import *



def generate_launch_description():

	ld = LaunchDescription()

	""" 
	
	
	Trafo Nodes
	 
	
	"""

	drone_qx = LaunchConfiguration('qx')
	drone_qy = LaunchConfiguration('qy')
	drone_qz = LaunchConfiguration('qz')
	drone_qw = LaunchConfiguration('qw')

	drone_qx_arg = DeclareLaunchArgument(
		'qx',
		default_value='0',
		description='Initial pose quaternion x'
	)
	drone_qy_arg = DeclareLaunchArgument(
		'qy',
		default_value='0',
		description='Initial pose quaternion y'
	)
	drone_qz_arg = DeclareLaunchArgument(
		'qz',
		default_value='0',
		description='Initial pose quaternion z'
	)
	drone_qw_arg = DeclareLaunchArgument(
		'qw',
		default_value='1',
		description='Initial pose quaternion w'
	)

	
	add_all_actions(ld, [drone_qx_arg, drone_qy_arg, drone_qz_arg, drone_qw_arg])

	
	slam_map_quat = [-0.5, 0.5, -0.5, -0.5]

	

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
            drone_qw, drone_qx, drone_qy, drone_qz,   # rotation in RPY (rad): -90°, 0°, -90°
            # '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
            # '1, 0, 0, 0',   # rotation in RPY (rad): -90°, 0°, -90°
			'map', 
			'initial_pose_map',
			],
		output='screen'
	)

	map_to_slam = quaternion_inverse(slam_map_quat)

	slam_map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_frame_publisher',
		arguments=[
			'0', '0', '0',                # translation x y z
            str(map_to_slam[0]), str(map_to_slam[1]), str(map_to_slam[2]), str(map_to_slam[3]),   # rotation in RPY (rad): -90°, 0°, -90°
            # '-0.5', '0.5', '-0.5', '-0.5',   # rotation in RPY (rad): -90°, 0°, -90°
            # '1, 0, 0, 0',   # rotation in RPY (rad): -90°, 0°, -90°
			'initial_pose_map',
			'slam_map', 
			],
		output='screen'
	)

	# inverse_drone_quat = quaternion_inverse(pose_quaternion)

	# total_rot = quaternion_multiply(
	# 	slam_map_quat,
	# 	inverse_drone_quat
	# )


	
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
	
	
	
	return ld