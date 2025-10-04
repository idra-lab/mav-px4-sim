
from  nav_manager.launcher_lib import *

def generate_launch_description():

	nav_manager_dir = get_package_share_directory('nav_manager')
	
	""" 
	
	BRIDGE
	  
	 """
	
	ld = LaunchDescription()

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
	return ld