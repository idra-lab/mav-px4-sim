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

from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

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
    
    def tf_timer(self):
        if not self.static_transform_set:
            # Store frame names in variables that will be used to
            # compute transformations


            # Look up for the transformation between target_frame and turtle2 frames
            # and send velocity commands for turtle2 to reach target_frame
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    'map',
                    'initial_pose_map',
                    rclpy.time.Time())
                
                self.static_transform_set = True
            except TransformException as ex:
                self.get_logger().info(
                    # f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    f'Could not transform initial_pose_map to map: {ex}')
                return
            
            try: 
                self.initial_pose_to_slam = self.tf_buffer.lookup_transform(
                    'slam_map',
                    'initial_pose_map',
                    rclpy.time.Time())
                
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform initial_pose_map to slam_map: {ex}')
                return
            
            print("Transform:\n", self.transform)
            
            r_initial_pose_in_map = quaternion_matrix([     
                                                            self.transform.transform.rotation.x,
                                                            self.transform.transform.rotation.y,
                                                            self.transform.transform.rotation.z,
                                                            self.transform.transform.rotation.w
                                                            ])
            
            r_initial_pose_to_slam = quaternion_matrix([   
                                                            self.initial_pose_to_slam.transform.rotation.x,
                                                            self.initial_pose_to_slam.transform.rotation.y,
                                                            self.initial_pose_to_slam.transform.rotation.z,
                                                            self.initial_pose_to_slam.transform.rotation.w
                                                            ])
            
            print("r_initial_pose_in_map:\n", r_initial_pose_in_map)
            print("r_initial_pose_to_slam:\n", r_initial_pose_to_slam)

            
            self.rotated_map_in_slam_frame = r_initial_pose_to_slam @ r_initial_pose_in_map @ np.linalg.inv(r_initial_pose_to_slam)
        
            print("rotated_map_in_slam_frame:\n", self.rotated_map_in_slam_frame)


        try: 
            self.slam_cam_to_drone_cam = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame',
                'x500_realsense/realsense_d435/base_link/realsense_d435',
                rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform initial_pose_map to slam_map: {ex}')
            return
            

            

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

        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and self.publish_setpoints_flag:
            # print("NAV_STATE: OFFBOARD")
            self.set_offboard_mode()

    def path_callback(self, msg):
        self.path = msg
        self.path_start_time = self.get_clock().now().nanoseconds
        self.index = 1
        self.publish_setpoints_flag = True
        self.get_logger().info("Path Received")

    def cmdloop_callback(self):

        # Publish offboard control modes
        if self.publish_setpoints_flag:

            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position=True
            offboard_msg.velocity=False
            offboard_msg.acceleration=False
            self.publisher_offboard_mode.publish(offboard_msg)

                
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

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
                    
                    

                    # # 5- select the two setpoints at the end of the current segment
                    # setpoint_i = self.path.poses[self.index]
                    # setpoint_f = self.path.poses[self.index + 1]

                    # t_i = setpoint_i.header.stamp.sec + setpoint_i.header.stamp.nanosec / 1e9
                    # t_f = setpoint_f.header.stamp.sec + setpoint_f.header.stamp.nanosec / 1e9

                    # # self.get_logger().info("t_i: " + str(t_i) + " t_f: " + str(t_f) + " elapsed_time: " + str(elapsed_time))
                    

                    # # 6 - Compute the time elapsed in the current segment
                    # current_segement_time = elapsed_time - (setpoint_i.header.stamp.sec + setpoint_i.header.stamp.nanosec / 1e9)
                    


                    # # 7- Compute a linear interpolation between the two setpoints
                    # x = (setpoint_f.pose.position.x - setpoint_i.pose.position.x) / (t_f - t_i) * current_segement_time + setpoint_i.pose.position.x 
                    # y = (setpoint_f.pose.position.y - setpoint_i.pose.position.y) / (t_f - t_i) * current_segement_time + setpoint_i.pose.position.y
                    # z = (setpoint_f.pose.position.z - setpoint_i.pose.position.z) / (t_f - t_i) * current_segement_time + setpoint_i.pose.position.z


                    # r =  self.rotated_map_in_slam_frame
                    
                    # # print("rotated map in slam frame:\n", self.rotated_map_in_slam_frame)
                    # # print("pos_slam:\n", pos_slam)
                    # # print("rotated_pos: ", rotated_pos)

                    # pos_slam =  np.array([[x], [y], [z]])
                    # rotated_pos = np.dot(r[0:3, 0:3], pos_slam)
                    # x = rotated_pos[0, 0]
                    # y = rotated_pos[1, 0]
                    # z = rotated_pos[2, 0]
                    
                    # R_i = quaternion_matrix([   
                    #                             setpoint_i.pose.orientation.x,
                    #                             setpoint_i.pose.orientation.y,
                    #                             setpoint_i.pose.orientation.z,
                    #                             setpoint_i.pose.orientation.w,          
                    #                                         ])
                    
                    # R_f = quaternion_matrix([   
                    #                             setpoint_f.pose.orientation.x,
                    #                             setpoint_f.pose.orientation.y,
                    #                             setpoint_f.pose.orientation.z,
                    #                             setpoint_f.pose.orientation.w,          
                    #                                         ])
                    
                    # R_i = np.dot( r, R_i)
                    # R_f = np.dot( r, R_f)
                    
                    # z_i = np.array(R_i[0:3, 2])
                    # z_f = np.array(R_f[0:3, 2])



                    # yaw_i = np.arctan2(z_i[0], z_i[2])
                    # yaw_f = np.arctan2(z_f[0], z_f[2])

                    # # _, pitch_i, _= euler_from_quaternion([  setpoint_i.pose.orientation.x,
                    # #                                         setpoint_i.pose.orientation.y,
                    # #                                         setpoint_i.pose.orientation.z,
                    # #                                         setpoint_i.pose.orientation.w])
                    
                    # # _, pitch_f, _= euler_from_quaternion([  setpoint_f.pose.orientation.x,
                    # #                                         setpoint_f.pose.orientation.y,
                    # #                                         setpoint_f.pose.orientation.z,
                    # #                                         setpoint_f.pose.orientation.w])

                    # # Pitch in camera frame is yaw in NED frame

                    # # 8- Compute a linear interpolation between the pitch angles
                    # # pitch = (pitch_f - pitch_i) / (t_f - t_i) * current_segement_time + pitch_i
                    # yaw = (yaw_f - yaw_i) / (t_f - t_i) * current_segement_time + yaw_i


                    # # 9- Handle the conversion from Camera to NED frame
                    # trajectory_msg = TrajectorySetpoint()
                    # trajectory_msg.position[0] = z
                    # trajectory_msg.position[1] = x
                    # trajectory_msg.position[2] = y
                    # trajectory_msg.yaw = yaw

                    setpoint = self.path.poses[self.index]

                    x = setpoint.pose.position.x
                    y = setpoint.pose.position.y
                    z = setpoint.pose.position.z 

                    t_slam_cam_to_drone_cam = [self.slam_cam_to_drone_cam.transform.translation.x, 
                                                self.slam_cam_to_drone_cam.transform.translation.y, 
                                                self.slam_cam_to_drone_cam.transform.translation.z]
                    
                    x += t_slam_cam_to_drone_cam[0]
                    y += t_slam_cam_to_drone_cam[1]
                    z += t_slam_cam_to_drone_cam[2]

                    r =  self.rotated_map_in_slam_frame

                    pos_slam =  np.array([[x], [y], [z]])
                    rotated_pos = np.dot(r[0:3, 0:3], pos_slam)
                    
                    x = rotated_pos[0, 0]
                    y = rotated_pos[1, 0]
                    z = rotated_pos[2, 0]

                    R_i = quaternion_matrix([   
                                             setpoint.pose.orientation.x,
                                             setpoint.pose.orientation.y,
                                             setpoint.pose.orientation.z,
                                             setpoint.pose.orientation.w,          
                                            ])
                    
                    R_i = np.dot( r , R_i )
                    z_i = np.array(R_i[0:3, 2])

                    yaw_i = np.arctan2(z_i[0], z_i[2])


                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position[0] = z
                    trajectory_msg.position[1] = x
                    trajectory_msg.position[2] = y
                    trajectory_msg.yaw = yaw_i

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
