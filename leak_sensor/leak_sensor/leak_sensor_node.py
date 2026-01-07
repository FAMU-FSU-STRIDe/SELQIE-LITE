#!/usr/bin/env python3
"""Leak sensor monitor node.

Polls a digital GPIO pin and publishes a Bool indicating whether a leak is
present. The leak sensor is active high by default, so a logic HIGH is reported
as a leak. Adjust parameters to match your wiring (pin numbering mode, pull
configuration, and active level).
"""

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LeakSensorNode(Node):
    def __init__(self):
        super().__init__("leak_sensor")

        self.declare_parameter("gpio_pin", 33)
        self.declare_parameter("gpio_mode", "BOARD")
        self.declare_parameter("pull", "NONE")
        self.declare_parameter("active_high", True)
        self.declare_parameter("poll_hz", 10.0)

        self.pin = int(self.get_parameter("gpio_pin").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        self.poll_period = 1.0 / poll_hz if poll_hz > 0 else 0.1

        mode_param = str(self.get_parameter("gpio_mode").value).upper()
        if mode_param == "BCM":
            GPIO.setmode(GPIO.BCM)
            pin_label = f"BCM {self.pin}"
        else:
            GPIO.setmode(GPIO.BOARD)
            pin_label = f"BOARD {self.pin}"

        pull_param = str(self.get_parameter("pull").value).upper()
        if pull_param == "UP":
            pull_cfg = GPIO.PUD_UP
        elif pull_param == "NONE":
            pull_cfg = GPIO.PUD_OFF
        else:
            pull_cfg = GPIO.PUD_DOWN

        GPIO.setup(self.pin, GPIO.IN)

        self.publisher_ = self.create_publisher(Bool, "leak_detected", 10)
        self.last_state = None
        self.create_timer(self.poll_period, self._poll_sensor)

        self.get_logger().info(f"Leak sensor on {pin_label} (pin={self.pin})")


    def _poll_sensor(self):
        raw_level = bool(GPIO.input(self.pin))
        leak_state = raw_level if self.active_high else not raw_level

        msg = Bool()
        msg.data = leak_state
        self.publisher_.publish(msg)

        if leak_state != self.last_state:
            leak_text = "LEAK" if leak_state else "clear"
            raw_text = "HIGH" if raw_level else "LOW"
            self.get_logger().info(
                f"Leak sensor state changed: {leak_text} (raw={raw_text}, active_high={self.active_high})"
            )


        self.last_state = leak_state

    def destroy_node(self):
        GPIO.cleanup(self.pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeakSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
