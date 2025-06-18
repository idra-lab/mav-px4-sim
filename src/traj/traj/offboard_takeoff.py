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

import sys


from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class OffboardTakeoff(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        #Create subscriptions
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_takeoff = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.declare_parameter('altitude', 2.0)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.altitude = self.get_parameter('altitude').value

        self.takeoff_speed = 0.5 # m/s
        self.takeoff_duration = self.altitude /self.takeoff_speed #seconds
        self.takeoff_start_time = self.get_clock().now().nanoseconds

        self.takeoff_completed = False

        self.heading = 0.0

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
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
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()

        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            print("NAV_STATE: OFFBOARD")
            self.set_offboard_mode()

    def vehicle_local_position_callback(self, msg):

        self.heading = msg.heading
        if msg.z <= -self.altitude:
            print("Takeoff completed")
            self.takeoff_completed = True

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        if not self.takeoff_completed:
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                delta_t = (self.get_clock().now().nanoseconds - self.takeoff_start_time) / 1e9

                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position[0] = 0
                trajectory_msg.position[1] = 0
                trajectory_msg.position[2] = -(self.altitude - self.takeoff_speed * (self.takeoff_duration-delta_t))
                trajectory_msg.velocity[0] = 0
                trajectory_msg.velocity[1] = 0
                trajectory_msg.velocity[2] = -self.takeoff_speed
                trajectory_msg.yaw = self.heading
                self.publisher_takeoff.publish(trajectory_msg)
            else: 
                print("Waiting for vehicle to be armed and in offboard mode")
                self.takeoff_completed = False
                self.takeoff_start_time = self.get_clock().now().nanoseconds
        else:
            print("Takeoff completed, stopping node")
            sys.exit()

            


def main(args=None):
    rclpy.init(args=args)

    offboard_takeoff = OffboardTakeoff()

    rclpy.spin(offboard_takeoff)

    offboard_takeoff.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
