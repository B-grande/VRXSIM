#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToThrusters(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrusters')
        
        # Subscribe to Nav2's cmd_vel output (try both topics)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Also subscribe to controller's raw output
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10
        )
        
        # Also try velocity smoother output
        self.cmd_vel_smoothed_sub = self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for WAM-V thrusters
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Maritime vehicle parameters
        self.max_thrust = 500.0  # Maximum thruster force (Newton-mode)
        self.max_linear_vel = 0.5  # Match Nav2 MPPI vx_max
        self.max_angular_vel = 1.9  # Match Nav2 MPPI wz_max
        self.thruster_separation = 1.0  # Distance between thrusters (meters)
        
        self.get_logger().info('WAM-V Thruster Controller Started')
        self.get_logger().info(f'Max velocities: linear={self.max_linear_vel}, angular={self.max_angular_vel}')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist message to differential thrust commands"""
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Turning
        
        # Log all cmd_vel messages for debugging
        self.get_logger().info(f'Received cmd_vel: linear.x={linear_x:.3f}, angular.z={angular_z:.3f}')
        
        # Differential drive kinematics for surface vessel
        # Convert angular velocity to linear wheel speeds
        angular_component = angular_z * (self.thruster_separation / 2.0)
        
        # Left and right thrust based on linear + angular components
        left_vel = linear_x - angular_component
        right_vel = linear_x + angular_component
        
        # Scale to thruster range (-max_thrust to +max_thrust)
        left_thrust = self.scale_thrust(left_vel)
        right_thrust = self.scale_thrust(right_vel)
        
        # Publish thruster commands
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_thrust
        right_msg.data = right_thrust
        
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)
        
        # Always log for debugging
        self.get_logger().info(f'Thrust commands: L={left_thrust:.1f}N, R={right_thrust:.1f}N')
    
    def scale_thrust(self, vel_component):
        """Scale velocity component to thruster force"""
        # Clamp to maximum velocity range  
        clamped_vel = max(-self.max_linear_vel, min(self.max_linear_vel, vel_component))
        
        # Scale to thrust range
        thrust = (clamped_vel / self.max_linear_vel) * self.max_thrust
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