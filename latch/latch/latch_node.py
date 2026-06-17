#!/usr/bin/env python3
"""ROS 2 node for Arduino Nano 33 BLE latch servo + reed switch over USB serial.

Protocol (115200 baud):
  Jetson → Teensy:  "<angle_deg>\\n"          e.g. "90.0\\n"
  Teensy → Jetson:  "REED <0|1>\\n"           e.g. "REED 1\\n"
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

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

        self.ser: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.create_subscription(Float64, "latch_angle_cmd", self.on_latch_cmd, 10)
        self.reed_pub = self.create_publisher(Bool, "reed_switch", 10)

        self._open_serial()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f"Latch node ready — port: {self.port} @ {self.baud} baud\n"
            "  Sub: latch_angle_cmd (Float64 degrees)\n"
            "  Pub: reed_switch (Bool)"
        )

    def _open_serial(self) -> None:
        with self._serial_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=self.timeout_s,
                    write_timeout=self.timeout_s,
                    dsrdtr=False,
                    rtscts=False,
                )
                time.sleep(2.0)  # wait for Arduino to boot after DTR reset
                self.get_logger().info(f"Serial connected: {self.port}")
            except (serial.SerialException, OSError) as exc:
                self.ser = None
                self.get_logger().error(f"Failed to open {self.port}: {exc}")

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._serial_lock:
                s = self.ser
            if s is None or not s.is_open:
                time.sleep(0.5)
                self._open_serial()
                continue
            try:
                line = s.readline()
                if not line:
                    continue
                text = line.decode("utf-8", errors="replace").strip()
                if text.startswith("REED"):
                    parts = text.split()
                    if len(parts) >= 2 and parts[1] in ("0", "1"):
                        msg = Bool()
                        msg.data = (parts[1] == "1")
                        self.reed_pub.publish(msg)
                    else:
                        self.get_logger().warning(f"Malformed REED line: {text}")
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                with self._serial_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.2)

    def on_latch_cmd(self, msg: Float64) -> None:
        angle_deg = float(msg.data)
        payload = f"{angle_deg:.1f}\n".encode("utf-8")
        with self._serial_lock:
            if self.ser is None or not self.ser.is_open:
                self.get_logger().warning("Serial not connected; dropping latch command")
                return
            try:
                self.ser.write(payload)
                self.ser.flush()
                self.get_logger().debug(f"Sent servo angle {angle_deg:.1f} deg")
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Serial write error: {exc}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def destroy_node(self) -> None:
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        with self._serial_lock:
            if self.ser:
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
