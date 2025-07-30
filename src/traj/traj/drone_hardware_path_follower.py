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
            
            self.static_transform_set = True
            

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

            # TODO: Change this back
            if True:
            # if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

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
                    
                    

                    setpoint = self.path.poses[self.index]

                    x = setpoint.pose.position.x
                    y = setpoint.pose.position.y
                    z = setpoint.pose.position.z 


                    r =  self.r_slam_map_to_map

                    pos_slam =  np.array([[x], [y], [z]])
                    rotated_pos = np.dot(r[0:3, 0:3], pos_slam)
                    
                    # x = rotated_pos[0, 0]
                    # y = rotated_pos[1, 0]
                    # z = rotated_pos[2, 0]

                    R_i = quaternion_matrix([   
                                             setpoint.pose.orientation.x,
                                             setpoint.pose.orientation.y,
                                             setpoint.pose.orientation.z,
                                             setpoint.pose.orientation.w,          
                                            ])
                    
                    R_i = np.dot( r , R_i )
                    x_i = np.array(R_i[0:3, 2])

                    yaw_i = np.arctan2(x_i[1], x_i[0])


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
