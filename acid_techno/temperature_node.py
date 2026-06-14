import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64


INITIAL_TEMPERATURE = 20.0
TARGET_TEMPERATURE = 10.0
OSCILLATION_AMPLITUDE = 2.0
OSCILLATION_PERIOD_SECONDS = 15.0
TEMPERATURE_TRACKING_GAIN = 0.18
TEMPERATURE_NOISE_RANGE = 0.08
MIN_TEMPERATURE = 0.0
MAX_TEMPERATURE = 40.0
TIMER_PERIOD_SECONDS = 1.0


class TemperatureNode(Node):
    def __init__(self):
        super().__init__('temperature_node')

        self.temperature = INITIAL_TEMPERATURE
        self.heater_on = False
        self.elapsed_time = 0.0

        self.temperature_pub = self.create_publisher(Float64, '/temperature', 10)
        self.heater_pub = self.create_publisher(Bool, '/heater_on', 10)
        self.timer = self.create_timer(TIMER_PERIOD_SECONDS, self.timer_callback)

        self.get_logger().info('Temperature node operational')

    def timer_callback(self):
        self.elapsed_time += TIMER_PERIOD_SECONDS

        oscillation = OSCILLATION_AMPLITUDE * math.sin(
            2.0 * math.pi * self.elapsed_time / OSCILLATION_PERIOD_SECONDS
        )
        desired_temperature = TARGET_TEMPERATURE + oscillation

        self.temperature += (desired_temperature - self.temperature) * TEMPERATURE_TRACKING_GAIN
        self.temperature += random.uniform(-TEMPERATURE_NOISE_RANGE, TEMPERATURE_NOISE_RANGE)
        self.temperature = max(MIN_TEMPERATURE, min(MAX_TEMPERATURE, self.temperature))

        self.heater_on = self.temperature < desired_temperature

        temperature_msg = Float64()
        temperature_msg.data = float(self.temperature)
        heater_msg = Bool()
        heater_msg.data = bool(self.heater_on)

        self.temperature_pub.publish(temperature_msg)
        self.heater_pub.publish(heater_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()