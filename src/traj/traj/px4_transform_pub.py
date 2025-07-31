import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

#!/usr/bin/env python


class PX4TransformPublisher(Node):
    def __init__(self):
        super().__init__('px4_transform_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.declare_parameter("hardware", False)
        self.hardware_implementation_flag = self.get_parameter("hardware").get_parameter_value().bool_value

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile,
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
                                                VehicleOdometry, 
                                                '/fmu/out/vehicle_odometry', 
                                                self.vehicle_odometry_callback, 
                                                qos_profile)

        self.br = TransformBroadcaster(self)
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.clock = Clock()

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]
        self.publish_transform()
        self.publish_camera_transform()

    def vehicle_local_position_callback(self, msg):
        if not self.hardware_implementation_flag:
            self.vehicle_local_position[0] = msg.x
            self.vehicle_local_position[1] = -msg.y
            self.vehicle_local_position[2] = -msg.z
            self.publish_transform()
            self.publish_camera_transform()

    def vehicle_odometry_callback(self, msg):

        if self.hardware_implementation_flag:

         
            # TODO: handle NED->ENU transformation
            self.vehicle_local_position[0] = msg.position[0]
            self.vehicle_local_position[1] = -msg.position[1]
            self.vehicle_local_position[2] = -msg.position[2]
     

            self.publish_transform()
            self.publish_camera_transform()
            



    def publish_camera_transform(self):
        t = TransformStamped()
        t.header.stamp = self.clock.now().to_msg()
        t.header.frame_id = 'drone'
        t.child_frame_id = 'x500_realsense/realsense_d435/base_link/realsense_d435'

        
        # Assuming the transform from drone to camera is fixed and known
        t.transform.translation.x = 0.0  # Example values, replace with actual values from sdf model
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = -0.5
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        
        self.br.sendTransform(t)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.clock.now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone'
        t.transform.translation.x = self.vehicle_local_position[0]
        t.transform.translation.y = self.vehicle_local_position[1]
        t.transform.translation.z = self.vehicle_local_position[2]
        t.transform.rotation.w = self.vehicle_attitude[0]
        t.transform.rotation.x = self.vehicle_attitude[1]
        t.transform.rotation.y = self.vehicle_attitude[2]
        t.transform.rotation.z = self.vehicle_attitude[3]
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4TransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()