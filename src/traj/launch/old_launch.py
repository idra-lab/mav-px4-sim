from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sim_package = get_package_share_directory('sim')
    px4_autopilot_dir = os.path.expanduser('~/PX4-Autopilot')
    home_dir = os.path.expanduser('~')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gz_x500'],
            cwd=px4_autopilot_dir,
            output='log'
        ),
        ExecuteProcess(
            cmd=['./QGroundControl.AppImage'],
            cwd=home_dir,
            output='log'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_node',
            output='log',
            arguments=['udp4', '--port', '8888']
        ),
        
        # ExecuteProcess(
        #     cmd=['python3', os.path.join(sim_package, 'scripts', 'takeoff.py')],
        #     output='log'
        # )

        # Node(
        #     package='sim',
        #     executable='takeoff_command',
        #     name='takeoff_command_node',
        #     output='log'
        # )
        # Node(
        #     package='gazebo_ros',
        #     executable='gzserver',
        #     name='gazebo_server',
        #     output='log',
        #     arguments=['-s', 'libgazebo_ros_factory.so']
        # ),
        # Node(
        #     package='gazebo_ros',
        #     executable='gzclient',
        #     name='gazebo_client',
        #     output='log'
        # ),
        # Node(
        #     package='px4',
        #     executable='px4',
        #     name='px4_node',
        #     output='log'
        # ),
        # Node(
        #     package='px4_offboard',
        #     executable='offboard_position_control',
        #     name='offboard_position_control_node',
        #     output='log'
        # )
    ])

if __name__ == '__main__':
    generate_launch_description()