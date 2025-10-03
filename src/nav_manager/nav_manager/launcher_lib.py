import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
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


# if __name__ == '__main__':
#     # This allows the launch file to be run directly with Python
#     print(collect_imported_names())


__all__ = collect_imported_names()
# __all__ = [
# 	'os', 