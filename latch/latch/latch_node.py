#!/usr/bin/env python3
"""ROS 2 node to command a Teensy latch controller over USB serial."""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import serial
import serial.tools.list_ports


def find_serial_port(vid: int | None, pid: int | None, serial_number: str | None) -> Optional[str]:
    """Return a matching /dev/tty* path if found."""
    for p in serial.tools.list_ports.comports():
        if vid is not None and p.vid != vid:
            continue
        if pid is not None and p.pid != pid:
            continue
        if serial_number and (p.serial_number != serial_number):
            continue
        return p.device
    return None


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        # --- Params ---
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout_s", 0.2)

        # Optional auto-detect (useful when ttyACM# changes)
        self.declare_parameter("usb_vid", -1)            # e.g. 0x16C0
        self.declare_parameter("usb_pid", -1)            # e.g. 0x0483
        self.declare_parameter("usb_serial", "")         # exact match, optional
        self.declare_parameter("reconnect_period_s", 1.0)

        # Optional command conditioning
        self.declare_parameter("min_angle_deg", -1e9)
        self.declare_parameter("max_angle_deg",  1e9)
        self.declare_parameter("deadband_deg", 0.0)

        self.port_param = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        usb_vid_raw = int(self.get_parameter("usb_vid").value)
        usb_pid_raw = int(self.get_parameter("usb_pid").value)
        self.usb_vid = None if usb_vid_raw < 0 else usb_vid_raw
        self.usb_pid = None if usb_pid_raw < 0 else usb_pid_raw
        self.usb_serial = str(self.get_parameter("usb_serial").value).strip() or None

        self.reconnect_period_s = float(self.get_parameter("reconnect_period_s").value)

        self.min_angle = float(self.get_parameter("min_angle_deg").value)
        self.max_angle = float(self.get_parameter("max_angle_deg").value)
        self.deadband = float(self.get_parameter("deadband_deg").value)

        # --- State ---
        self.ser: Optional[serial.Serial] = None
        self._last_sent: Optional[float] = None
        self._last_open_attempt = 0.0

        self.create_subscription(Float64, "latch_angle_cmd", self.on_latch_cmd, 10)
        self.create_timer(0.2, self._ensure_serial_open)

        self.get_logger().info(
            "Latch serial node starting:\n"
            f"  port param: {self.port_param}\n"
            f"  baud: {self.baud}\n"
            f"  timeout_s: {self.timeout_s}\n"
            f"  usb_vid: {self.usb_vid}\n"
            f"  usb_pid: {self.usb_pid}\n"
            f"  usb_serial: {self.usb_serial}\n"
            f"  reconnect_period_s: {self.reconnect_period_s}\n"
            f"  clamp: [{self.min_angle}, {self.max_angle}] deg\n"
            f"  deadband_deg: {self.deadband}"
        )

    def _resolve_port(self) -> str:
        # If VID/PID provided, prefer auto-detect.
        if self.usb_vid is not None or self.usb_pid is not None or self.usb_serial is not None:
            found = find_serial_port(self.usb_vid, self.usb_pid, self.usb_serial)
            if found:
                return found
        return str(self.port_param)

    def _ensure_serial_open(self) -> None:
        if self.ser and self.ser.is_open:
            return

        now = time.monotonic()
        if now - self._last_open_attempt < self.reconnect_period_s:
            return
        self._last_open_attempt = now

        port = self._resolve_port()
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baud,
                timeout=self.timeout_s,
                write_timeout=self.timeout_s,
            )
            # Give Teensy a moment after opening (some firmwares reset on open)
            time.sleep(0.05)
            self.get_logger().info(f"Connected to latch on {port} @ {self.baud}")
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().warn(f"Serial not available on {port}: {exc}")

    def on_latch_cmd(self, msg: Float64) -> None:
        angle = float(msg.data)

        # Clamp
        if angle < self.min_angle:
            angle = self.min_angle
        elif angle > self.max_angle:
            angle = self.max_angle

        # Deadband vs last sent
        if self._last_sent is not None and abs(angle - self._last_sent) < self.deadband:
            return

        # Ensure port is open (don’t block too long)
        self._ensure_serial_open()
        if not (self.ser and self.ser.is_open):
            return

        payload = f"{angle:.1f}\n".encode("utf-8")
        try:
            self.ser.write(payload)
            self.ser.flush()
            self._last_sent = angle
            self.get_logger().debug(f"Sent latch angle {angle:.1f} deg")
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self) -> None:
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

