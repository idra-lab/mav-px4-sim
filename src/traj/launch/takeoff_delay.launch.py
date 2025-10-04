
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch.actions import TimerAction

def generate_launch_description():    

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


    return LaunchDescription(traj_utilities_nodes)