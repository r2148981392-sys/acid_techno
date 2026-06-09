from geometry_msgs.msg import Pose2D
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

class CorrectedOdomNode(Node):
    def __init__(self):
        super().__init__('corrected_odom_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.pub = self.create_publisher(
            Pose2D,
            '/corrected_odom',
            10
        )

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            msg = Pose2D()
            msg.x = t.transform.translation.x
            msg.y = t.transform.translation.y
            msg.theta = t.transform.rotation.w  # note: this is not yaw, see below
            self.pub.publish(msg)
        except:
            pass  # TF not ready yet, just wait

def main(args=None):
    rclpy.init(args=args)
    node = CorrectedOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
