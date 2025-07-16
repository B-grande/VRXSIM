# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu

# class ImuCovarianceFixer(Node):
#     def __init__(self):
#         super().__init__('imu_covariance_fixer')
#         self.sub = self.create_subscription(
#             Imu,
#             'imu/data_raw',  # relative topic name for remapping
#             self.imu_callback,
#             10
#         )
#         self.pub = self.create_publisher(Imu, 'imu/imu/data_fixed', 10)  # relative topic name for remapping

#     def imu_callback(self, msg):
#         # Set orientation covariance to small nonzero values
#         msg.orientation_covariance = [0.01, 0.0, 0.0,
#                                       0.0, 0.01, 0.0,
#                                       0.0, 0.0, 0.01]
#         self.pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuCovarianceFixer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
