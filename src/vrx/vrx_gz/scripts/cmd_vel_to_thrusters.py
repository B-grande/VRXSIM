#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToThrusters(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrusters')
        
        # Subscribe to Nav2's cmd_vel output
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for WAM-V thrusters
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Maritime vehicle parameters
        self.max_thrust = 500.0  # Maximum thruster force (Newton-mode)
        self.max_linear_vel = 2.0  # Max linear velocity from nav2 params
        self.max_angular_vel = 1.0  # Max angular velocity from nav2 params
        
        self.get_logger().info('WAM-V Thruster Controller Started')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist message to differential thrust commands"""
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Turning
        
        # Differential drive kinematics for surface vessel
        # Left and right thrust based on linear + angular components
        left_thrust_raw = linear_x - angular_z
        right_thrust_raw = linear_x + angular_z
        
        # Scale to thruster range (-max_thrust to +max_thrust)
        left_thrust = self.scale_thrust(left_thrust_raw)
        right_thrust = self.scale_thrust(right_thrust_raw)
        
        # Publish thruster commands
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_thrust
        right_msg.data = right_thrust
        
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)
        
        # Log for debugging
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(f'Cmd: lin={linear_x:.2f}, ang={angular_z:.2f} -> Thrust: L={left_thrust:.1f}, R={right_thrust:.1f}')
    
    def scale_thrust(self, vel_component):
        """Scale velocity component to thruster force"""
        # Clamp to maximum velocity range
        max_vel = max(self.max_linear_vel, self.max_angular_vel)
        clamped_vel = max(-max_vel, min(max_vel, vel_component))
        
        # Scale to thrust range
        thrust = (clamped_vel / max_vel) * self.max_thrust
        return float(thrust)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrusters()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()