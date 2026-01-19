# ws2812b_ros/led_tester.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32MultiArray
import math
import time

# pack R,G,B into 0x00RRGGBB
def rgb(r, g, b):
    return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)

class LEDTester(Node):
    def __init__(self):
        super().__init__('ws2812b_led_tester')
        self.declare_parameter('num_leds', 2)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('hold_secs', 1.0)

        self.n_leds = int(self.get_parameter('num_leds').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.hold_secs = float(self.get_parameter('hold_secs').value)

        self.pub = self.create_publisher(UInt32MultiArray, 'led_colors', 10)

        # Demo pattern: red, green, blue, white, off
        self.palette = [
            rgb(255,   0,   0),
            rgb(  0, 255,   0),
            rgb(  0,   0, 255),
            rgb(255, 255, 255),
            rgb(  0,   0,   0),
        ]
        self.index = 0
        self.t0 = time.time()

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'LED tester running: n_leds={self.n_leds}, rate_hz={self.rate_hz}, hold_secs={self.hold_secs}'
        )

    def tick(self):
        # advance color every hold_secs
        if time.time() - self.t0 >= self.hold_secs:
            self.index = (self.index + 1) % len(self.palette)
            self.t0 = time.time()

        color = self.palette[self.index]
        msg = UInt32MultiArray()
        msg.data = [color] * self.n_leds
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = LEDTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

