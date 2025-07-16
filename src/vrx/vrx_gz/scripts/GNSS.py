# #!/usr/bin/python3

# import utm
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion

# class GNSSOdometryPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_odometry_publisher')
#         self.subscription = self.create_subscription(
#             NavSatFix,
#             '/wamv/sensors/gps/gps/fix',
#             self.gps_callback,
#             10
#         )
#         self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
#         self.origin_x = None
#         self.origin_y = None
#         self.origin_z = None
#         self.zone_number = None
#         self.zone_letter = None
#         self.get_logger().info('Subscribed to /wamv/sensors/gps/gps/fix and publishing /odometry/gps')

#     def gps_callback(self, msg: NavSatFix):
#         if msg.status.status == -1 or (msg.latitude == 0 and msg.longitude == 0):
#             return
#         x, y, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
#         z = msg.altitude
#         if self.origin_x is None:
#             self.origin_x = x
#             self.origin_y = y
#             self.origin_z = z
#             self.zone_number = zone_number
#             self.zone_letter = zone_letter
#         # Use the same UTM zone for all subsequent fixes
#         if self.zone_number is not None and self.zone_letter is not None:
#             x, y, _, _ = utm.from_latlon(
#                 msg.latitude, msg.longitude,
#                 force_zone_number=self.zone_number,
#                 force_zone_letter=self.zone_letter
#             )
#         local_x = x - self.origin_x
#         local_y = y - self.origin_y
#         local_z = z - self.origin_z
#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'wamv/wamv/base_link'
#         odom.pose.pose.position.x = local_x
#         odom.pose.pose.position.y = local_y
#         odom.pose.pose.position.z = local_z
#         odom.pose.pose.orientation = Quaternion(w=1.0)  # No orientation from GPS
#         # Set minimum covariance
#         min_cov = 1.0
#         cov = [0.0] * 36
#         cov[0] = msg.position_covariance[0] if msg.position_covariance[0] > 0 else min_cov
#         cov[7] = msg.position_covariance[4] if msg.position_covariance[4] > 0 else min_cov
#         cov[14] = msg.position_covariance[8] if msg.position_covariance[8] > 0 else min_cov
#         odom.pose.covariance = cov
#         self.odom_pub.publish(odom)

#        # self.get_logger().info(f'Published Odometry: x={local_x}, y={local_y}, z={local_z}, cov={cov}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GNSSOdometryPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
