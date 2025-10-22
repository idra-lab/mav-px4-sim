from  nav_manager.launcher_lib import *


def generate_launch_description():

    gazebo_world = LaunchConfiguration("gz_world_file")

    gazebo_world_launch_arg = DeclareLaunchArgument(
		'gz_world_file', default_value='benchmarking.sdf'
	)

    px4_sim_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                                    FindPackageShare('nav_manager'),
                                                    'px4_sim.launch.py'
                                                    ])
					
                            ),
                            launch_arguments={
                                							'gz_world_file': gazebo_world
															}.items(),
                            

                            # 'src/realsense-ros/realsense2_camera/examples/pointcloud/rs_d455_pointcloud_launch.py')
    )



    perception_launcher_path = get_package_share_directory('perception_launcher')

    perception_sim_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                                    perception_launcher_path,
                                                    'launch',
                                                    'simulation.launch.py'
                                                    ])
                            )
                            # launch_arguments={'node_name': 'bar'}.items(),

                            # 'src/realsense-ros/realsense2_camera/examples/pointcloud/rs_d455_pointcloud_launch.py')
    )

    return LaunchDescription([
        gazebo_world_launch_arg,
        px4_sim_launch,
        perception_sim_launch,
        
    ])
    
    