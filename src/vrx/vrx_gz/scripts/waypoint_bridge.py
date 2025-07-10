#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class WaypointBridge(Node):
    def __init__(self):
        super().__init__('waypoint_bridge')
        self.declare_parameter('input_topic', '/vizanti/waypoints')
        self.declare_parameter('action_name', '/waypoint_follower/follow_waypoints')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._action_client = ActionClient(self, FollowWaypoints, action_name)
        self._sub = self.create_subscription(Path, input_topic, self.path_callback, 10)
        self.get_logger().info(f"Subscribed to {input_topic}, will send to action {action_name}")

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty path, ignoring.')
            return
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = msg.poses
        self.get_logger().info(f"Sending {len(msg.poses)} waypoints to Nav2 Waypoint Follower...")
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint following finished with result: {result}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
