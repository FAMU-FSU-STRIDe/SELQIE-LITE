# ws2812b_ros/rainbow_fade.py
import colorsys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32MultiArray

def pack_rgb(r, g, b):
    return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)

class RainbowFade(Node):
    def __init__(self):
        super().__init__('ws2812b_rainbow_fade')

        # Animation controls
        self.declare_parameter('num_leds', 2)
        self.declare_parameter('rate_hz', 60.0)         # FPS / publish rate
        self.declare_parameter('cycle_seconds', 10.0)   # seconds per full hue cycle
        self.declare_parameter('spread_degrees', 120.0) # hue offset per LED
        self.declare_parameter('saturation', 1.0)       # 0..1
        self.declare_parameter('value', 1.0)            # 0..1
        self.declare_parameter('reverse', False)        # bool

        self.n_leds = int(self.get_parameter('num_leds').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cycle_seconds = max(0.001, float(self.get_parameter('cycle_seconds').value))
        self.spread_deg = float(self.get_parameter('spread_degrees').value)
        self.sat = float(self.get_parameter('saturation').value)
        self.val = float(self.get_parameter('value').value)
        self.reverse = bool(self.get_parameter('reverse').value)

        self.pub = self.create_publisher(UInt32MultiArray, 'led_colors', 10)

        self.period = 1.0 / self.rate_hz
        self.start_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f'Rainbow fade: n_leds={self.n_leds}, rate={self.rate_hz} Hz, '
            f'cycle={self.cycle_seconds}s, spread={self.spread_deg}°, '
            f'sat={self.sat}, val={self.val}, reverse={self.reverse}'
        )

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        t_sec = (now_ns - self.start_ns) * 1e-9
        base_h = (t_sec / self.cycle_seconds) % 1.0  # 0..1

        data = []
        dir_sign = -1.0 if self.reverse else 1.0

        for i in range(self.n_leds):
            # Per-LED hue offset
            h = (base_h + dir_sign * (self.spread_deg / 360.0) * i) % 1.0
            r_f, g_f, b_f = colorsys.hsv_to_rgb(
                h,
                max(0.0, min(1.0, self.sat)),
                max(0.0, min(1.0, self.val)),
            )
            r = int(round(r_f * 255))
            g = int(round(g_f * 255))
            b = int(round(b_f * 255))
            data.append(pack_rgb(r, g, b))

        msg = UInt32MultiArray()
        msg.data = data
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RainbowFade()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

