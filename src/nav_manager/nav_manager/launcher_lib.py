import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import OpaqueFunction

from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_inverse
from launch.actions import TimerAction
import sys
import inspect
import re

from launch_ros.parameter_descriptions import ParameterValue
from rcl_interfaces.msg import ParameterType

from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler

import yaml
from launch.event_handlers import OnExecutionComplete, OnProcessExit




import xml.etree.ElementTree as ET

def obatin_real_time_factor(world_path):
	tree = ET.parse(world_path)
	root = tree.getroot()

	# Find the <real_time_factor> element
	physics = root.find('.//physics')
	if physics is not None:
		real_time_factor = physics.find('real_time_factor')
		if real_time_factor is not None:
			return real_time_factor.text
		else:
			print("No <real_time_factor> element found.")

def add_all_actions(ld, actions):
	for action in actions:
		
		ld.add_action(action)

def collect_imported_names():
    """
    Return a sorted list of names in this module that were imported from other modules.
    Suitable to assign to __all__ (e.g. __all__ = collect_imported_names()).
    """
    current_mod = sys.modules.get(__name__)
    imported = []
    for name, val in list(globals().items()):

        # print(name,"       ", val, "\n")
        if name.startswith('_'):
            continue
        # modules themselves
        # if inspect.ismodule(val):
        #     imported.append(name)
        #     continue
        # # objects coming from a different module
        # try:
        #     obj_mod = inspect.getmodule(val)
        # except Exception:
        #     obj_mod = None
        # if obj_mod is not None and obj_mod is not current_mod:
        #     imported.append(name)
		
        imported.append(name)
    return sorted(set(imported))


def set_drone_pose(pose_string: str): 
    d_pos = [float(x) for x in pose_string.split()[:3]]
    d_ang = [float(x) for x in pose_string.split()[3:]]
    d_ang[2] -= 1.57  # Adjust yaw to match the expected orientation

    d_q = quaternion_from_euler(

        d_ang[0],
        d_ang[1],
        d_ang[2]
    )

    return d_pos, d_ang, d_q

pose_str = '-10 0 0.01 0 0 0.0'

drone_position, drone_angles, drone_quat = set_drone_pose(pose_str)


# drone_position = [float(x) for x in pose_str.split()[:3]]
# drone_angles = [float(x) for x in pose_str.split()[3:]]
# drone_angles[2] -= 1.57  # Adjust yaw to match the expected orientation

# drone_quat = quaternion_from_euler(

#     drone_angles[0],
#     drone_angles[1],
#     drone_angles[2]
# )

slam_map_quat = [-0.5, 0.5, -0.5, -0.5]

map_to_slam = quaternion_inverse(slam_map_quat)


class DroneConfig: 
	
    def __init__(self):
        self.pose_str = pose_str
        self.drone_position = drone_position
        self.drone_angles = drone_angles
        self.drone_quat = drone_quat
        self.slam_map_quat = slam_map_quat
        self.map_to_slam = map_to_slam

    def read_launch_configuration(self, path: str): 
        with open(path) as stream:
            try:
                data = yaml.safe_load(stream)

                pose = data["env"]["initial_pose"]

                self.pose_str = ' '.join([str(x) for x in pose])

                self.drone_position, self.drone_angles, self.drone_quat = set_drone_pose(self.pose_str)

                self.slam_map_quat = data["env"]["slam_map_quat"]

                self.map_to_slam = quaternion_inverse(self.slam_map_quat)


            except yaml.YAMLError as exc:
                print(exc)
            


drone_config_instance = DroneConfig()

local_config_path = LaunchConfiguration("config_path", default=os.path.join(get_package_share_directory('benchmarking'), 'config', 'cube_benchmark/launch_config.yaml'))



def read_yaml_config(context, *args, **kwargs):


    
    # local_config_path = LaunchConfiguration("config_path", default=os.path.join(get_package_share_directory('benchmarking'), 'config', 'cube_benchmark/launch_config.yaml')).perform(context)
    config_path_str = local_config_path.perform(context)
    print(f"Loading config from: {config_path_str}")
    drone_config_instance.read_launch_configuration(config_path_str)

    print(f"DroneConfig.param = {drone_config_instance.drone_position}")

    return []
        

__all__ = collect_imported_names()
# __all__ = [
# 	'os', 