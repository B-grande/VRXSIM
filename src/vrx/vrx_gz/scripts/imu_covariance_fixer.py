#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuCovarianceFixer(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')
        self.sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',  # updated to match actual IMU topic
            self.imu_callback,
            10
        )
        self.pub = self.create_publisher(Imu, 'imu/data_fixed', 10)  # relative topic name for remapping

    def imu_callback(self, msg):
        # Create a copy of the message to avoid in-place modification issues
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.header.frame_id = 'wamv/wamv/base_link'  # Set to actual IMU sensor frame
        # Normalize quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 1e-6:
            new_msg.orientation.x = qx / norm
            new_msg.orientation.y = qy / norm
            new_msg.orientation.z = qz / norm
            new_msg.orientation.w = qw / norm
        else:
            # fallback to identity quaternion if norm is too small
            new_msg.orientation.x = 0.0
            new_msg.orientation.y = 0.0
            new_msg.orientation.z = 0.0
            new_msg.orientation.w = 1.0
        new_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01]
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
