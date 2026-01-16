#!/usr/bin/env python3
"""ROS 2 node to drive the latch GPIO for the Hitec D954SW servo."""
from __future__ import annotations

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        self.declare_parameter("gpio_pin", 15)
        self.declare_parameter("gpio_mode", "BOARD")
        self.declare_parameter("active_high", True)

        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.gpio_mode = str(self.get_parameter("gpio_mode").value)
        self.active_high = bool(self.get_parameter("active_high").value)

        self._set_mode()
        GPIO.setup(self.gpio_pin, GPIO.OUT, initial=self._inactive_level())

        self.create_subscription(Bool, "latch_cmd", self.on_latch_cmd, 10)

        self.get_logger().info(
            "Latch GPIO node ready:\n"
            f"  gpio_pin: {self.gpio_pin}\n"
            f"  gpio_mode: {self.gpio_mode}\n"
            f"  active_high: {self.active_high}"
        )

    def _set_mode(self) -> None:
        mode_upper = self.gpio_mode.upper()
        if mode_upper == "BOARD":
            GPIO.setmode(GPIO.BOARD)
        elif mode_upper == "BCM":
            GPIO.setmode(GPIO.BCM)
        elif mode_upper == "CVM":
            GPIO.setmode(GPIO.CVM)
        elif mode_upper == "TEGRA_SOC":
            GPIO.setmode(GPIO.TEGRA_SOC)
        else:
            raise ValueError(
                f"Unsupported gpio_mode '{self.gpio_mode}'. Use BOARD, BCM, CVM, or TEGRA_SOC."
            )

    def _active_level(self) -> int:
        return GPIO.HIGH if self.active_high else GPIO.LOW

    def _inactive_level(self) -> int:
        return GPIO.LOW if self.active_high else GPIO.HIGH

    def on_latch_cmd(self, msg: Bool) -> None:
        level = self._active_level() if bool(msg.data) else self._inactive_level()
        GPIO.output(self.gpio_pin, level)
        self.get_logger().info(
            "Latch command: %s (pin %s -> %s)",
            "open" if msg.data else "close",
            self.gpio_pin,
            "HIGH" if level == GPIO.HIGH else "LOW",
        )

    def destroy_node(self) -> None:
        try:
            GPIO.output(self.gpio_pin, self._inactive_level())
        finally:
            GPIO.cleanup(self.gpio_pin)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
