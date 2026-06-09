import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.action.client import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationNode(Node):
    def __init__(self):
        super().__init__('path_find_node')
        self.get_logger().info('Constructing Path finding node')

        self.current_goal = (0, 0)

        self.goal_pub = self.create_publisher (
            GoalStatus,
            '/goal_pub',
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_location',
            self.goal_sub_callback,
            10
        )

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Initialized action client (Path finding node)')
        self.client.wait_for_server()
        self.get_logger().info('Path finding node operational')

    def goal_sub_callback(self, location: PoseStamped):
        self.current_goal = (location.pose.position.x, location.pose.position.y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = location

        future = self.client.send_goal_async(goal_msg)

        self.get_logger().info('Sending goal to Physical robot')
        future.add_done_callback(self.finished_callback)

    def finished_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            
            status_msg = GoalStatus()
            status_msg.status = GoalStatus.STATUS_CANCELED
            self.goal_pub.publish(status_msg)
            return

        self.get_logger().info(f'Goal accepted, navigating to location [{self.current_goal[0]}] [{self.current_goal[1]}]')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        goal_handle = future.result()
        status = goal_handle.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation complete')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"Could not reach: x: {self.current_goal[0]}, y: {self.current_goal[1]}")

        status_msg = GoalStatus()
        status_msg.status = status
        self.goal_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
