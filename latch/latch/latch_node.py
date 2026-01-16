#!/usr/bin/env python3
"""ROS 2 node to command a Teensy latch controller over USB serial."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import serial


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout_s", 0.2)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        self.timeout_s = float(self.get_parameter("timeout_s").get_parameter_value().double_value)

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
        )

        self.create_subscription(Float64, "latch_angle_cmd", self.on_latch_cmd, 10)

        self.get_logger().info(
            "Latch serial node ready:\n"
            f"  port: {self.port}\n"
            f"  baud: {self.baud}\n"
            f"  timeout_s: {self.timeout_s}"
        )

    def on_latch_cmd(self, msg: Float64) -> None:
        angle_deg = float(msg.data)
        payload = f"ANGLE {angle_deg:.1f}\n".encode("utf-8")
        try:
            self.ser.write(payload)
            self.ser.flush()
            self.get_logger().info("Sent latch angle %.1f deg", angle_deg)
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def destroy_node(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
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
