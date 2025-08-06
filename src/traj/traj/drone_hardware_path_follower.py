#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tf_transformations import euler_from_quaternion, quaternion_matrix,  rotation_matrix

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

from geometry_msgs.msg import PoseStamped, Point

import sys

from collections import deque

from scipy.spatial.transform import Rotation as R


from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException, TransformStamped

def homogeneous_transform_matrix_pose(pose: PoseStamped):
    """
    Convert a PoseStamped message to a homogeneous transformation matrix.
    """
    translation = np.array([[pose.pose.position.x],
                            [pose.pose.position.y],
                            [pose.pose.position.z]])
    
    rotation = quaternion_matrix([pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z,
                                  pose.pose.orientation.w])
    
    rotation[0:3, 3] = translation.flatten()  # Set translation in the last column
    
    return rotation

def homogeneous_transform_matrix_tf(tf_trafo: TransformStamped):
    """
    Convert a TransformStamped message to a homogeneous transformation matrix.
    """
    translation = np.array([[tf_trafo.transform.translation.x],
                            [tf_trafo.transform.translation.y],
                            [tf_trafo.transform.translation.z]])

    rotation = quaternion_matrix([tf_trafo.transform.rotation.x,
                                  tf_trafo.transform.rotation.y,
                                  tf_trafo.transform.rotation.z,
                                  tf_trafo.transform.rotation.w])

    rotation[0:3, 3] = translation.flatten()  # Set translation in the last column

    return rotation


class SE3Filter:
    def __init__(self, max_len=20):
        self.transforms = deque(maxlen=max_len)
        

    def push(self, T):
        self.transforms.append(T)

    def get_smoothed(self):
        if not self.transforms:
            return np.eye(4)

        # Average translation
        translations = np.array([T[:3, 3] for T in self.transforms])
        # print(translations)
        t_avg = np.mean(translations, axis=0)

        # print(t_avg)

        # Average rotation (using scipy)
        rotations = R.from_matrix([T[:3, :3] for T in self.transforms])
        # print(rotations)
        r_avg = rotations.mean().as_matrix()
        # print(r_avg)


        T_avg = np.eye(4)
        T_avg[:3, :3] = r_avg
        T_avg[:3, 3] = t_avg
        return T_avg
    

class DronePathFollower(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'map').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_transform_set = False

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.tf_timer)

        self.transform = None
        self.slam_map_to_rotated_map = None
        self.camera_slam_to_drone_camera = None

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # PATH MANAGER
        self.path_subscriber = self.create_subscription(Path, '/drone_path', self.path_callback, qos_profile)

        self.path = Path()

        self.publish_setpoints_flag = False

        self.path_start_time = self.get_clock().now().nanoseconds

        self.index = 0

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.se3_filter = SE3Filter(max_len=20)
    
    def tf_timer(self):

        # We immediately check if we need to stop the drone
        try: 
            self.drone_to_camera_color = self.tf_buffer.lookup_transform(
                'camera_color_frame',
                'drone',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform camera_slam to drone: {ex}')
            return
        
        self.r_drone_to_camera_color = quaternion_matrix([   
                                                            self.drone_to_camera_color.transform.rotation.x,
                                                            self.drone_to_camera_color.transform.rotation.y,
                                                            self.drone_to_camera_color.transform.rotation.z,
                                                            self.drone_to_camera_color.transform.rotation.w
                                                            ])
        
        self.t_drone_to_camera_color = np.array([[self.drone_to_camera_color.transform.translation.x],
                                                 [self.drone_to_camera_color.transform.translation.y],
                                                 [self.drone_to_camera_color.transform.translation.z]])
        
        self.T_drone_to_camera_color = homogeneous_transform_matrix_tf(self.drone_to_camera_color)

        if (np.linalg.norm(self.t_drone_to_camera_color) > 1.0): 
            self.set_hold_mode()
            rclpy.shutdown()
            sys.exit()


        if not self.static_transform_set:
            # Store frame names in variables that will be used to
            # compute transformations
         
            try: 
                self.slam_map_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    'slam_map',
                    rclpy.time.Time())
                

                
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform slam_map to map: {ex}')
                return
            
            self.r_slam_map_to_map = quaternion_matrix([   
                                                            self.slam_map_to_map.transform.rotation.x,
                                                            self.slam_map_to_map.transform.rotation.y,
                                                            self.slam_map_to_map.transform.rotation.z,
                                                            self.slam_map_to_map.transform.rotation.w
                                                            ])
            
            self.T_slam_map_to_map = homogeneous_transform_matrix_tf(self.slam_map_to_map)

            self.T_map_to_slam_map = np.linalg.inv(self.T_slam_map_to_map)


            

            self.camera_color_optical_to_camera_color = self.tf_buffer.lookup_transform(
                'camera_color_frame',
                'camera_color_optical_frame',
                rclpy.time.Time())
            

            self.T_camera_color_optical_to_camera_color = homogeneous_transform_matrix_tf(self.camera_color_optical_to_camera_color)

            self.static_transform_set = True


        

        try: 
            self.drone_to_map = self.tf_buffer.lookup_transform(
                "map",
                "drone",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to drone: {ex}')
            return
        
        # We need first a new frame representing the drone in map but with optical orientation
        self.T_drone_to_map = homogeneous_transform_matrix_tf(self.drone_to_map)

        
        """
        # In theory we can directly take this from the realsense sdf model on the drone
        self.T_drone_optical_to_map = np.eye(4)
        self.T_drone_optical_to_map[0:3, 0:3] = self.T_map_to_slam_map[0:3, 0:3] @ self.T_drone_to_map[0:3, 0:3]
        self.T_drone_optical_to_map[0:3, 3] = self.T_drone_to_map[0:3, 3]

        # We made up a new frame and we can transform this into the slam map frame
        # We need to compute the difference between the drone optical frame and the tracked camera optical color frame
        self.T_drone_optical_to_slam_map = np.dot(self.T_map_to_slam_map, self.T_drone_optical_to_map) 
        
        """

        try: 
            self.drone_optical_to_slam_map = self.tf_buffer.lookup_transform(
                "slam_map",
                "x500_realsense/realsense_d435/base_link/realsense_d435",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform slam_map to drone_optical_frame: {ex}')
            return

        self.T_drone_optical_to_slam_map = homogeneous_transform_matrix_tf(self.drone_optical_to_slam_map)


        try: 
            self.slam_map_to_camera_color_optical = self.tf_buffer.lookup_transform(
                "camera_color_optical_frame",
                "slam_map",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform slam_map to camera_color_optical_frame: {ex}')
            return
        
        self.T_slam_map_to_camera_color_optical = homogeneous_transform_matrix_tf(self.slam_map_to_camera_color_optical)
        

        try: 
            self.drone_optical_to_camera_color_optical = self.tf_buffer.lookup_transform(
                "camera_color_optical_frame",
                "x500_realsense/realsense_d435/base_link/realsense_d435",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform camera_color_optical_frame to drone_optical_frame: {ex}')
            return
        
        self.T_drone_optical_to_camera_color_optical = homogeneous_transform_matrix_tf(self.drone_optical_to_camera_color_optical)

        # self.T_drone_optical_to_camera_color_optical = np.dot(self.T_slam_map_to_camera_color_optical, self.T_drone_optical_to_slam_map)

        self.se3_filter.push(self.T_drone_optical_to_camera_color_optical)

        self.filtered_T_drone_optical_to_camera_color_optical = self.se3_filter.get_smoothed()

        
                                                
        
        

    def set_hold_mode(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.param7 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.param7 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
        
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

        # if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and self.publish_setpoints_flag:
        #     # print("NAV_STATE: OFFBOARD")
        #     self.set_offboard_mode()



    def path_callback(self, msg):
        self.path = msg
        self.path_start_time = self.get_clock().now().nanoseconds
        self.index = 1
        self.publish_setpoints_flag = True
        self.get_logger().info("Path Received")

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

    def cmdloop_callback(self):

        # Publish offboard control modes
        if self.publish_setpoints_flag:

            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position=True
            offboard_msg.velocity=False
            offboard_msg.acceleration=False
            self.publisher_offboard_mode.publish(offboard_msg)

            
            # if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            if True:

                # 1- Check time elapsed since path start in seconds
                elapsed_time = (self.get_clock().now().nanoseconds - self.path_start_time) / 1e9

                # 2- Check if the segment index is valid
                if self.index < len(self.path.poses)-1:
                    
                    # 3- Check if the time elapsed is greater than the time of the segment
                    if elapsed_time > self.path.poses[self.index].header.stamp.sec + self.path.poses[self.index].header.stamp.nanosec / 1e9:
                        
                        # 4- Increment the index
                        self.index += 1

                        if self.index == len(self.path.poses) -1:
                            return
                        
                    
                    T_setpoint_optical_to_slam_map = homogeneous_transform_matrix_pose(self.path.poses[self.index])

                    # filtered_T_drone_optical_to_camera_color_optical = self.se3_filter.get_smoothed()

                    T_drone_optical_setpoint_to_slam_map = np.dot(T_setpoint_optical_to_slam_map, self.T_drone_optical_to_camera_color_optical)

                    # T_setpoint_optical_to_map = np.dot(self.T_slam_map_to_map, T_setpoint_optical_to_slam_map)

                    # T_setpoint_cam_to_slam_map = self.camera_color_optical_to_camera_color @ T_setpoint_optical_to_slam_map

                    # T_setpoint_cam_to_map = np.dot(self.T_slam_map_to_map, T_setpoint_cam_to_slam_map)

                    # T_offset_setpoint_to_map = np.dot(T_setpoint_cam_to_map, self.T_drone_to_camera_color)

                    # print(T_offset_setpoint_to_map)

                    # setpoint = self.path.poses[self.index]

                    # x = setpoint.pose.position.x
                    # y = setpoint.pose.position.y
                    # z = setpoint.pose.position.z 


                    # r =  self.r_slam_map_to_map

                    # pos_slam =  np.array([[x], [y], [z]])


                    # rotated_pos = np.dot(r[0:3, 0:3], pos_slam)

                    # # rotated_pos = rotated_pos + self.t_drone_to_camera_color
                    

                    # R_i = quaternion_matrix([   
                    #                          setpoint.pose.orientation.x,
                    #                          setpoint.pose.orientation.y,
                    #                          setpoint.pose.orientation.z,
                    #                          setpoint.pose.orientation.w,          
                    #                         ])
                    
                    R_i = T_drone_optical_setpoint_to_slam_map[0:3, 0:3]
                    pos = T_drone_optical_setpoint_to_slam_map[0:3, 3]
                    # R_i = np.dot( r , R_i )
                    z_i = np.array(R_i[0:3, 2])

                    print("Z_i: ", z_i)

                    # # x_i = np.array(T_offset_setpoint_to_map[0:3, 0])

                    # TODO: There is a mistake here
                    yaw_i = np.arctan2(z_i[0], z_i[2])
                    # yaw_i = 0.0

                    # # pos = T_offset_setpoint_to_map[0:3, 3]
                    # pos = rotated_pos

                    x, y, z = pos

                    print("Yaw: ", yaw_i)


                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position[0] = z
                    trajectory_msg.position[1] = x
                    trajectory_msg.position[2] = y
                    trajectory_msg.yaw = yaw_i

                    # trajectory_msg.position[0] = pos[0]
                    # trajectory_msg.position[1] = -pos[1]
                    # trajectory_msg.position[2] = -pos[2]
                    # trajectory_msg.yaw = -yaw_i

                    self.publisher_trajectory.publish(trajectory_msg)


                else:
                    # 2B- Stop publishing setpoints
                    self.publish_setpoints_flag = False
                    self.index = 0
                    self.get_logger().info("Path Completed")



def main(args=None):
    rclpy.init(args=args)

    drone_path_follower = DronePathFollower()

    rclpy.spin(drone_path_follower)

    drone_path_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
