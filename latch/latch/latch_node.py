#!/usr/bin/env python3
"""ROS 2 node to drive a Hitec D954SW servo via Linux PWM sysfs."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


@dataclass
class ServoLimits:
    min_pulse_us: int
    max_pulse_us: int
    min_angle_deg: float
    max_angle_deg: float


class PwmSysfs:
    def __init__(self, chip: int, channel: int, logger):
        self.chip = int(chip)
        self.channel = int(channel)
        self.logger = logger
        self.base = Path(f"/sys/class/pwm/pwmchip{self.chip}")
        self.channel_path = self.base / f"pwm{self.channel}"

    def _write(self, path: Path, value: str) -> None:
        path.write_text(value)

    def ensure_exported(self) -> None:
        if self.channel_path.exists():
            return
        if not self.base.exists():
            raise FileNotFoundError(f"PWM chip path not found: {self.base}")
        self.logger.info(f"Exporting PWM channel {self.channel} on chip {self.chip}")
        (self.base / "export").write_text(str(self.channel))

    def set_period_ns(self, period_ns: int) -> None:
        self._write(self.channel_path / "period", str(int(period_ns)))

    def set_duty_cycle_ns(self, duty_ns: int) -> None:
        self._write(self.channel_path / "duty_cycle", str(int(duty_ns)))

    def set_enabled(self, enabled: bool) -> None:
        self._write(self.channel_path / "enable", "1" if enabled else "0")


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        self.declare_parameter("pwm_chip", 0)
        self.declare_parameter("pwm_channel", 0)
        self.declare_parameter("period_us", 20000)
        self.declare_parameter("min_pulse_us", 1000)
        self.declare_parameter("max_pulse_us", 2000)
        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("neutral_angle_deg", 90.0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("startup_angle_deg", 90.0)

        self.pwm_chip = int(self.get_parameter("pwm_chip").value)
        self.pwm_channel = int(self.get_parameter("pwm_channel").value)
        self.period_us = int(self.get_parameter("period_us").value)
        self.neutral_angle_deg = float(self.get_parameter("neutral_angle_deg").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.startup_angle_deg = float(self.get_parameter("startup_angle_deg").value)

        self.limits = ServoLimits(
            min_pulse_us=int(self.get_parameter("min_pulse_us").value),
            max_pulse_us=int(self.get_parameter("max_pulse_us").value),
            min_angle_deg=float(self.get_parameter("min_angle_deg").value),
            max_angle_deg=float(self.get_parameter("max_angle_deg").value),
        )

        self.pwm = PwmSysfs(self.pwm_chip, self.pwm_channel, self.get_logger())
        self.pwm.ensure_exported()
        self.pwm.set_period_ns(self.period_us * 1000)

        if self.auto_enable:
            self.pwm.set_enabled(True)

        self._current_pulse_us: Optional[int] = None

        self.create_subscription(Float64, "angle_cmd", self.on_angle_cmd, 10)
        self.create_subscription(Float64, "pulse_us_cmd", self.on_pulse_cmd, 10)
        self.create_subscription(Bool, "enable_cmd", self.on_enable_cmd, 10)

        self.get_logger().info(
            "Latch PWM node ready:\n"
            f"  pwm_chip: {self.pwm_chip}\n"
            f"  pwm_channel: {self.pwm_channel}\n"
            f"  period_us: {self.period_us}\n"
            f"  min_pulse_us: {self.limits.min_pulse_us}\n"
            f"  max_pulse_us: {self.limits.max_pulse_us}\n"
            f"  min_angle_deg: {self.limits.min_angle_deg}\n"
            f"  max_angle_deg: {self.limits.max_angle_deg}\n"
            f"  neutral_angle_deg: {self.neutral_angle_deg}\n"
            f"  auto_enable: {self.auto_enable}"
        )

        self.set_angle(self.startup_angle_deg)

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def angle_to_pulse_us(self, angle_deg: float) -> int:
        angle = self.clamp(angle_deg, self.limits.min_angle_deg, self.limits.max_angle_deg)
        span = self.limits.max_angle_deg - self.limits.min_angle_deg
        if span <= 0.0:
            raise ValueError("Invalid angle span; min_angle_deg must be < max_angle_deg")
        ratio = (angle - self.limits.min_angle_deg) / span
        pulse = self.limits.min_pulse_us + ratio * (self.limits.max_pulse_us - self.limits.min_pulse_us)
        return int(round(pulse))

    def set_pulse_us(self, pulse_us: int) -> None:
        pulse_us = int(self.clamp(pulse_us, self.limits.min_pulse_us, self.limits.max_pulse_us))
        duty_ns = pulse_us * 1000
        self.pwm.set_duty_cycle_ns(duty_ns)
        self._current_pulse_us = pulse_us
        self.get_logger().debug(f"Set pulse to {pulse_us} us")

    def set_angle(self, angle_deg: float) -> None:
        pulse_us = self.angle_to_pulse_us(angle_deg)
        self.set_pulse_us(pulse_us)
        self.get_logger().info(f"Set angle to {angle_deg:.2f} deg (pulse {pulse_us} us)")

    def on_angle_cmd(self, msg: Float64) -> None:
        try:
            self.set_angle(float(msg.data))
        except Exception as exc:
            self.get_logger().error(f"Failed to set angle: {exc}")

    def on_pulse_cmd(self, msg: Float64) -> None:
        try:
            self.set_pulse_us(int(round(float(msg.data))))
        except Exception as exc:
            self.get_logger().error(f"Failed to set pulse: {exc}")

    def on_enable_cmd(self, msg: Bool) -> None:
        try:
            self.pwm.set_enabled(bool(msg.data))
            self.get_logger().info(f"PWM enable set to {msg.data}")
        except Exception as exc:
            self.get_logger().error(f"Failed to set enable: {exc}")

    def destroy_node(self) -> None:
        try:
            if self._current_pulse_us is not None:
                self.set_angle(self.neutral_angle_deg)
            if self.auto_enable:
                self.pwm.set_enabled(False)
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
