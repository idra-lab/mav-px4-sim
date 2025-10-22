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

from trajectory_msgs.msg import SE3Trajectory, SE3Spline, RotationSpline, TranslationSpline

from scipy.interpolate import BSpline

def rosTime2Sec(ros_time):
    return ros_time.sec + ros_time.nanosec * 1e-9


def hat_op(r): 
    A = np.zeros((3,3))
 
    A[0][1] = - r[2]
    A[0][2] = r[1]
    A[1][0] = r[2]
    A[1][2] = -r[0]
    A[2][0] = -r[1]
    A[2][1] = r[0]

    return A

def exponential_map(r):
 
 A = hat_op(r)
 
 n = np.linalg.norm(r)

 A2 = np.matmul(A, A)
 

 if(n == 0): 
    return np.identity(3)
 else: 
    result = np.identity(3) + (np.sin(n) / n)* A + ((1 - np.cos(n))/ (n*n)) * A2; 
    

 return result

def rightJacobian(r: np.array): 
    result = np.eye(3)

    n = np.linalg.norm(r)

    if (n>0): 
        A = hat_op(r)
        result = result -  (1 - np.cos(n))/(pow(n, 2)) * A + (n - np.sin(n))/(pow(n,3)) * np.dot(A,A)

    return result

def get_omega(r, dr): 
    A = rightJacobian(r)

    return np.dot(A,dr)
         
def accelerationConstant(r: np.array, dr: np.array): 
    n = np.linalg.norm(r)

    result = np.zeros((3))

    if n!=0: 

        rxdr =   np.cross(r,dr)
        result = (n - np.sin(n))/(np.power(n, 3)) * dr
        result = np.cross(result, rxdr)
        temp = (2*np.cos(n) + n*np.sin(n)-2)/(np.power(n,4)) * r 
        dtemp = np.dot(temp, dr)
        result =  result -dtemp*rxdr

        temp = (3*np.sin(n) - n*np.cos(n)-2*n)/(np.power(n, 5)) *r; 
        dtemp = np.dot(temp, dr)
        result += dtemp*np.cross(r, rxdr) 

    return result


def get_omegaDot(r: np.array, dr: np.array, ddr:np.array): 
    C = accelerationConstant(r,dr)
    J = rightJacobian(r)

    return np.dot(J,ddr) + C

class TrajSpline: 
    def __init__(self, spline: SE3Spline): 

        self.trans_knots = np.array(spline.translation_spline.knots)
        self.rot_knots = np.array(spline.rotation_spline.knots)

        self.start_time = rosTime2Sec(spline.start_time)
        self.end_time = rosTime2Sec(spline.end_time)

        self.duration = self.end_time - self.start_time

        self.trans_deg = spline.translation_spline.degree
        self.rot_deg = spline.rotation_spline.degree

        self.trans_ctrls = []


        for i in range(len(spline.translation_spline.pos_pts)):
            self.trans_ctrls.append(np.array([spline.translation_spline.pos_pts[i].x,
                                           spline.translation_spline.pos_pts[i].y,
                                           spline.translation_spline.pos_pts[i].z]))
        self.trans_ctrls = np.reshape(self.trans_ctrls, (-1, 3))

        print("Translation control points:\n", self.trans_ctrls)

        self.rot_ctrls = []
        for i in range(len(spline.rotation_spline.rot_pts)):
            self.rot_ctrls.append(np.array([spline.rotation_spline.rot_pts[i].x,
                                           spline.rotation_spline.rot_pts[i].y,
                                           spline.rotation_spline.rot_pts[i].z]))
        self.rot_ctrls = np.reshape(self.rot_ctrls, (-1, 3))

        print("Rotation control points:\n", self.rot_ctrls)

        self.init_orient = quaternion_matrix([
                                                spline.rotation_spline.initial_orientation.w,
                                                spline.rotation_spline.initial_orientation.x,
                                                spline.rotation_spline.initial_orientation.y,
                                                spline.rotation_spline.initial_orientation.z
                                            ])

        self.trans_spl = BSpline(self.trans_knots, self.trans_ctrls, self.trans_deg)
        self.rot_spl = BSpline(self.rot_knots, self.rot_ctrls, self.rot_deg)

        # TODO: We might build a yaw spline here instead of the full rotation spline

    def evaluate(self, traj_time: float):

        t = (traj_time - self.start_time)/self.duration
        position = self.trans_spl(t)
        velocity = self.trans_spl.derivative(1)(t)
        acceleration = self.trans_spl.derivative(2)(t)

        # TODO: Take care of R_CW
        r = self.rot_spl(t)
        dr = self.rot_spl.derivative(1)(t)
        ddr = self.rot_spl.derivative(2)(t)

        R_CW = self.init_orient[0:3, 0:3] @ exponential_map(r)

        R_WC = R_CW.T

        # R = np.transpose(R)
        omega = get_omega(r, dr)
        omegaDot = get_omegaDot(r, dr, ddr)


        return position, velocity, acceleration, R_WC, omega, omegaDot

class DroneTrajectoryFollower(Node):

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

        # trajectory MANAGER
        self.trajectory_subscriber = self.create_subscription(SE3Trajectory, '/drone_se3_trajectory', self.trajectory_callback, qos_profile)

        self.trajectory = SE3Trajectory()

        self.current_spline = None

        self.publish_setpoints_flag = False

        self.trajectory_start_time = self.get_clock().now().nanoseconds

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

    def trajectory_callback(self, msg):
        self.trajectory = msg
        self.index = 0

        self.current_spline = TrajSpline(self.trajectory.splines[0])
        self.trajectory_start_time = self.get_clock().now().nanoseconds 
        self.publish_setpoints_flag = True
        self.get_logger().info("trajectory Received")

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
                elapsed_time = (self.get_clock().now().nanoseconds - self.trajectory_start_time) / 1e9

                # 2- Check if the segment index is valid
                if self.index < len(self.trajectory.splines):


                    
                    # 3- Check if the time elapsed is greater than the time of the segment
                    if elapsed_time > self.current_spline.end_time:
                        
                        # 4- Increment the index
                        self.index += 1
                        self.current_spline = TrajSpline(self.trajectory.splines[self.index])

                        if self.index == len(self.trajectory.splines) :
                            return
                    
                    pos, vel, acc, R, omega, omegaDot = self.current_spline.evaluate(elapsed_time)
                    

                    t_slam_cam_to_drone_cam = np.array([self.slam_cam_to_drone_cam.transform.translation.x, 
                                            self.slam_cam_to_drone_cam.transform.translation.y, 
                                            self.slam_cam_to_drone_cam.transform.translation.z])
                    

                    pos += t_slam_cam_to_drone_cam
                    


                    r =  self.rotated_map_in_slam_frame
                    
                    

                    pos_slam =  np.array([[pos[0]], [pos[1]], [pos[2]]])
                    rotated_pos = np.dot(r[0:3, 0:3], pos_slam)
                    x = rotated_pos[0, 0]
                    y = rotated_pos[1, 0]
                    z = rotated_pos[2, 0]
                    
                    
                    R_drone = np.dot( r[0:3, 0:3], R)
                    z_axis = np.array(R[0:3, 2])

                    

                    yaw = np.arctan2(z_axis[0], z_axis[2]) - np.pi/2

                    

                    # 9- Handle the conversion from Camera to NED frame
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position[0] = z
                    trajectory_msg.position[1] = x
                    trajectory_msg.position[2] = y
                    trajectory_msg.yaw = yaw

                    self.publisher_trajectory.publish(trajectory_msg)



                else:
                    # 2B- Stop publishing setpoints
                    self.publish_setpoints_flag = False
                    self.get_logger().info("trajectory Completed")



def main(args=None):
    rclpy.init(args=args)

    drone_trajectory_follower = DroneTrajectoryFollower()

    rclpy.spin(drone_trajectory_follower)

    drone_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
