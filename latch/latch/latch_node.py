#!/usr/bin/env python3
"""ROS 2 node to command a Teensy latch controller over USB serial + publish reed switch state."""
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
        self._reader_thread: Optional[threading.Thread] = None

        # Publishers/subscribers
        self.create_subscription(Float64, "latch_angle_cmd", self.on_latch_cmd, 10)
        self.reed_pub = self.create_publisher(Bool, "reed_switch", 10)

        # Open serial + start reader
        self._open_serial()
        self._start_reader()

        self.get_logger().info(
            "Latch serial node ready:\n"
            f"  port: {self.port}\n"
            f"  baud: {self.baud}\n"
            f"  timeout_s: {self.timeout_s}\n"
            "Publishing:\n"
            "  reed_switch (std_msgs/Bool)"
        )

    def _open_serial(self) -> None:
        with self._serial_lock:
            # Close if needed
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            # Open
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=self.timeout_s,
                    write_timeout=self.timeout_s,
                )
                # Small settle (Teensy can reset on open)
                time.sleep(0.05)
                self.get_logger().info(f"Connected serial: {self.port} @ {self.baud}")
            except (serial.SerialException, OSError) as exc:
                self.ser = None
                self.get_logger().error(f"Failed to open serial {self.port}: {exc}")

    def _start_reader(self) -> None:
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self) -> None:
        """Continuously read lines and publish reed switch updates."""
        while not self._stop_event.is_set():
            s = None
            with self._serial_lock:
                s = self.ser

            if s is None or not s.is_open:
                # Try to reconnect periodically
                time.sleep(0.5)
                self._open_serial()
                continue

            try:
                line = s.readline()  # bytes until \n or timeout
                if not line:
                    continue
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue

                # Expect: "REED 0" or "REED 1"
                if text.startswith("REED"):
                    parts = text.split()
                    if len(parts) >= 2 and parts[1] in ("0", "1"):
                        msg = Bool()
                        msg.data = (parts[1] == "1")
                        self.reed_pub.publish(msg)
                    else:
                        self.get_logger().warn(f"Malformed REED line: {text}")
                # Optional: you can handle "OK <deg>" / "PONG" / etc here if you want
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                # Force reconnect
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
                self.get_logger().warn("Serial not connected; cannot send latch command")
                return
            try:
                self.ser.write(payload)
                self.ser.flush()
                # Keep this as debug to avoid spamming INFO at control rate
                self.get_logger().debug("Sent latch angle %.1f deg", angle_deg)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Serial write failed: {exc}")
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
                self.ser = None

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

