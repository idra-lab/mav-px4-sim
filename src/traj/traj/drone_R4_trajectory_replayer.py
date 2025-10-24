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

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import yaml


from scipy.interpolate import BSpline

def rosTime2Sec(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9


class R4TrajSpline: 
    def __init__(self, filename: str): 

        traj_yaml = None

        with open(filename, 'r') as f:

            traj_yaml = yaml.safe_load(f)

        self.rotate_frames = traj_yaml.get('rotate_frames', False)

            
        self.knots = np.array(traj_yaml['knots'])
        self.degree = traj_yaml['order'] 
        self.duration = self.knots[-self.degree -1]   # scale to total time duration

        self.knots = self.knots / self.duration

        # self.duration *= 2.0

        # self.knots = np.linspace(0.0, 1.0, len(self.knots))

        
        points_field = traj_yaml['pos_pts']
        self.ctrl_pts = []
        for i in range(len(points_field)):
            if self.rotate_frames:
                # Apply rotation to each control point
                self.ctrl_pts.append([points_field[i]['y'],
                                      -points_field[i]['x'],
                                      points_field[i]['z']])
            else:
                self.ctrl_pts.append([points_field[i]['x'],
                                      points_field[i]['y'],
                                      points_field[i]['z']])


        self.ctrl_pts = np.reshape(self.ctrl_pts, (-1, 3))
        print("CTRL PTS RAW: ", self.ctrl_pts)

        if self.rotate_frames:
            self.yaw_pts = np.pi - np.array(traj_yaml['yaw_pts'])
            # clamp yaw values to at most pi/2
            self.yaw_pts = np.clip(self.yaw_pts, np.pi/2, None)
        else:
            self.yaw_pts = np.array(traj_yaml['yaw_pts'])


        self.trans_spl = BSpline(self.knots, self.ctrl_pts, self.degree)
        self.yaw_spl = BSpline(self.knots, self.yaw_pts, self.degree)

    def setStartTime(self, start_time: float):
        
        self.start_time = start_time
        self.end_time = start_time + self.duration

        

    def evaluate(self, traj_time: float):

        t = (traj_time - self.start_time)/self.duration

        t = t if t <= 1.0 else 1.0

        position = self.trans_spl(t)
        velocity = self.trans_spl.derivative(1)(t)
        acceleration = self.trans_spl.derivative(2)(t)

        yaw = self.yaw_spl(t)
        dyaw = self.yaw_spl.derivative(1)(t)
        ddyaw = self.yaw_spl.derivative(2)(t)

        


        return position, velocity, acceleration, yaw, dyaw, ddyaw
    
    def printSplineRviz(self, publisher, frame_id='map'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        num_points = 100
        for i in range(num_points + 1):
            t = self.start_time + i * (self.end_time - self.start_time) / num_points
            p, _, _, _, _, _ = self.evaluate(t)
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            marker.points.append(point)

        publisher.publish(marker)



class DroneR4TrajectoryReplayer(Node):

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
        

        self.traj_file = self.declare_parameter(
          'trajectory_file', 'path/to/your/trajectory.yaml').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_transform_set = False

        

        # trajectory MANAGER

        self.trajectory = R4TrajSpline(self.traj_file)

        


        self.publish_setpoints_flag = True

        self.trajectory_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.trajectory.setStartTime(self.trajectory_start_time)

        self.traj_publisher = self.create_publisher(Marker, 'apace_trajectory_marker', qos_profile)

        self.trajectory.printSplineRviz(self.traj_publisher, frame_id="initial_pose_map")


        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED


        # Call on_timer function every second
        self.timer_tf = self.create_timer(1.0, self.tf_timer)

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

                # 1- Check time elapsed since trajectory start in seconds
                now = self.get_clock().now().nanoseconds * 1e-9
                
                


                p, v, a, yaw, dyaw, ddyaw = self.trajectory.evaluate(now)

                print("Position: ", p)

                yaw = float(yaw)  # Adjust yaw
                # yaw = 0.0  # Adjust yaw

                # print("Yaw: ", yaw)
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position[0] = p[1]
                trajectory_msg.position[1] = p[0]
                trajectory_msg.position[2] = -p[2]
                trajectory_msg.yaw = yaw

                self.publisher_trajectory.publish(trajectory_msg)



def main(args=None):
    rclpy.init(args=args)

    drone_r4_trajectory_replayer = DroneR4TrajectoryReplayer()

    rclpy.spin(drone_r4_trajectory_replayer)

    drone_r4_trajectory_replayer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# ros2 run traj drone_R4_trajectory_replayer --ros-args -p trajectory_file:=src/benchmarking/resource/trajectories/cube_benchmark/apace/benchmark.yaml 
