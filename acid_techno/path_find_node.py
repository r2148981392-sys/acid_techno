import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from nav2_msgs.action import NavigateToPose
from . import grid_model

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from std_msgs.msg import Float64MultiArray
from action_msgs.msg import GoalStatus

GRID_SIZE_X = 3.0
GRID_SIZE_Y = 3.0

class PathFindNode(Node):
    grid: grid_model.MapModel
    latest_ph: float

    latest_x: float
    latest_y: float
    latest_w: float

    current_goal: tuple[float, float]
    goal_active: bool
    odom_published: bool
    ph_published: bool
    scan_finished: bool

    def __init__(self):
        super().__init__('path_find_node')
        self.get_logger().info('Constructing Path finding node')

        self.grid = grid_model.MapModel(GRID_SIZE_X, GRID_SIZE_Y)
        self.goal_active = False
        self.latest_ph = 0
        self.ph_published = False
        self.odom_published = False
        self.scan_finished = False

        self.odom_sub = self.create_subscription (
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_pub = self.create_subscription (
            Bool,
            '/goal_pub',
            self.goal_callback,
            10
        )

        self.ph_sub = self.create_subscription(
            Float64,
            '/acidity',
            self.ph_callback,
            10
        )

        self.grid_state_pub = self.create_publisher(
            Float64MultiArray,
            '/grid_state',
            10
        )
        self.grid_publish_timer = self.create_timer(1.0, self.publish_grid_state)

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.get_logger().info('Initialized action client (Path finding node)')

        self.publish_grid_state()

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available yet. Grid state will still be published.')
        else:
            self.get_logger().info('NavigateToPose action server available.')
        self.get_logger().info('Path finding node operational')

    def ph_callback(self, ph: Float64):
        if self.scan_finished:
            return
        self.latest_ph = ph.data
        self.ph_published = True

    def odom_callback(self, msg:Odometry):
        if self.scan_finished:
            return
        self.odom_published = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_x = x
        self.latest_y = y
        self.latest_w = msg.pose.pose.orientation.w

        if not self.grid.location_measured(x, y):
            self.get_logger().info(f"Taken measurement at {x}, {y} of ph: {self.latest_ph}")
            self.grid.set_sample(x, y, self.latest_ph)
            self.publish_grid_state()

        self.send_goal()

    def publish_grid_state(self):
        msg = Float64MultiArray()
        rows = len(self.grid.grid)
        cols = len(self.grid.grid[0]) if rows > 0 else 0
        data = [float(rows), float(cols)]

        for row in self.grid.grid:
            for square in row:
                if not square.accessible:
                    data.append(-2.0)
                elif not square.measured:
                    data.append(-1.0)
                else:
                    data.append(float(square.ph_sample))

        msg.data = data
        self.grid_state_pub.publish(msg)

    def send_goal(self):
        if not self.odom_published or not self.ph_published or self.scan_finished:
            return
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.current_goal = self.grid.get_closest_sample_location(self.latest_x, self.latest_y)

        if self.current_goal == (0, 0):
            self.scan_finished = True
            self.save_map()

        goal_msg.pose.pose.position.x = float(self.current_goal[0])
        goal_msg.pose.pose.position.y = float(self.current_goal[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = float(self.latest_w)

        if not self.goal_active:
            self.goal_active = True
            future = self.client.send_goal_async(goal_msg)
            self.get_logger().info('Sending goal...')
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_active = False
            return
        self.get_logger().info(f'Goal accepted, navigating to location [{self.current_goal[0]}] [{self.current_goal[1]}]')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        goal_handle= future.result()
        status = goal_handle.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation complete')
            if not self.grid.location_measured(self.current_goal[0], self.current_goal[1]):
                self.grid.mark_location_unreachable(self.current_goal[0], self.current_goal[1])
                self.publish_grid_state()

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"Could not reach: x: {self.current_goal[0]}, y: {self.current_goal[1]}")
            self.grid.mark_location_unreachable(self.current_goal[0], self.current_goal[1])
            self.publish_grid_state()

        self.goal_active = False
        self.send_goal()

    def save_map(self):
        #todo
        return


def main(args=None):
    rclpy.init(args=args)
    node = PathFindNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
