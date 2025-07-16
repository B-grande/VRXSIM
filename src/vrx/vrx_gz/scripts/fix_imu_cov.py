#!/usr/bin/env python3
"""
Small helper that copies /wamv/sensors/imu/imu/data,
adds tiny non‑zero values to orientation_covariance,
and republishes on /imu/corrected.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

FILL_VALUE = 1e-6          #   1 µrad²

class CovarianceFixer(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')

        # Allow topics to be overridden from a launch file
        self.declare_parameter('imu_in',  '/wamv/sensors/imu/imu/data')
        self.declare_parameter('imu_out', '/imu/corrected')

        in_topic  = self.get_parameter('imu_in').get_parameter_value().string_value
        out_topic = self.get_parameter('imu_out').get_parameter_value().string_value

        qos = 10
        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.sub = self.create_subscription(Imu, in_topic, self.callback, qos)

        self.get_logger().info(f'✓ IMU covariance fixer listening on {in_topic} ➜ {out_topic}')

    def callback(self, msg: Imu):
        cov = list(msg.orientation_covariance)
        if cov[0] == 0.0 and cov[4] == 0.0 and cov[8] == 0.0:
            cov[0] = cov[4] = cov[8] = FILL_VALUE
            msg.orientation_covariance = cov
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CovarianceFixer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
