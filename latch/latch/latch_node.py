#!/usr/bin/env python3
"""ROS 2 node to drive a Hitec D954SW servo via Jetson.GPIO PWM."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


@dataclass
class ServoLimits:
    min_pulse_us: int
    max_pulse_us: int
    min_angle_deg: float
    max_angle_deg: float


class PwmGpio:
    def __init__(self, pin: int, mode: str, period_us: int, logger):
        self.pin = int(pin)
        self.mode = mode
        self.period_us = int(period_us)
        self.logger = logger
        self._pwm = None
        self._running = False

    def _set_mode(self) -> None:
        mode_upper = self.mode.upper()
        if mode_upper == "BOARD":
            GPIO.setmode(GPIO.BOARD)
        elif mode_upper == "BCM":
            GPIO.setmode(GPIO.BCM)
        elif mode_upper == "CVM":
            GPIO.setmode(GPIO.CVM)
        elif mode_upper == "TEGRA_SOC":
            GPIO.setmode(GPIO.TEGRA_SOC)
        else:
            raise ValueError(f"Unsupported gpio_mode '{self.mode}'. Use BOARD, BCM, CVM, or TEGRA_SOC.")

    def setup(self) -> None:
        self._set_mode()
        GPIO.setup(self.pin, GPIO.OUT)
        frequency_hz = 1_000_000.0 / float(self.period_us)
        self._pwm = GPIO.PWM(self.pin, frequency_hz)
        self.logger.info(f"Initialized PWM on GPIO pin {self.pin} at {frequency_hz:.2f} Hz")

    def start(self, duty_cycle_percent: float) -> None:
        if self._pwm is None:
            self.setup()
        if not self._running:
            self._pwm.start(duty_cycle_percent)
            self._running = True
        else:
            self._pwm.ChangeDutyCycle(duty_cycle_percent)

    def change_duty_cycle(self, duty_cycle_percent: float) -> None:
        if self._pwm is None:
            self.setup()
        if self._running:
            self._pwm.ChangeDutyCycle(duty_cycle_percent)
        else:
            self._pwm.start(duty_cycle_percent)
            self._running = True

    def stop(self) -> None:
        if self._pwm is not None and self._running:
            self._pwm.stop()
            self._running = False

    def cleanup(self) -> None:
        try:
            self.stop()
        finally:
            GPIO.cleanup(self.pin)


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        self.declare_parameter("gpio_pin", 33)
        self.declare_parameter("gpio_mode", "BOARD")
        self.declare_parameter("period_us", 20000)
        self.declare_parameter("min_pulse_us", 1000)
        self.declare_parameter("max_pulse_us", 2000)
        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("neutral_angle_deg", 90.0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("startup_angle_deg", 90.0)

        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.gpio_mode = str(self.get_parameter("gpio_mode").value)
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

        self.pwm = PwmGpio(self.gpio_pin, self.gpio_mode, self.period_us, self.get_logger())
        self.pwm.setup()

        self._current_pulse_us: Optional[int] = None

        self.create_subscription(Float64, "angle_cmd", self.on_angle_cmd, 10)
        self.create_subscription(Float64, "pulse_us_cmd", self.on_pulse_cmd, 10)
        self.create_subscription(Bool, "enable_cmd", self.on_enable_cmd, 10)

        self.get_logger().info(
            "Latch PWM node ready:\n"
            f"  gpio_pin: {self.gpio_pin}\n"
            f"  gpio_mode: {self.gpio_mode}\n"
            f"  period_us: {self.period_us}\n"
            f"  min_pulse_us: {self.limits.min_pulse_us}\n"
            f"  max_pulse_us: {self.limits.max_pulse_us}\n"
            f"  min_angle_deg: {self.limits.min_angle_deg}\n"
            f"  max_angle_deg: {self.limits.max_angle_deg}\n"
            f"  neutral_angle_deg: {self.neutral_angle_deg}\n"
            f"  auto_enable: {self.auto_enable}"
        )

        self.set_angle(self.startup_angle_deg)

        if self.auto_enable and self._current_pulse_us is not None:
            self.pwm.start(self.pulse_to_duty_cycle(self._current_pulse_us))

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

    def pulse_to_duty_cycle(self, pulse_us: int) -> float:
        return (float(pulse_us) / float(self.period_us)) * 100.0

    def set_pulse_us(self, pulse_us: int) -> None:
        pulse_us = int(self.clamp(pulse_us, self.limits.min_pulse_us, self.limits.max_pulse_us))
        self._current_pulse_us = pulse_us
        duty_cycle = self.pulse_to_duty_cycle(pulse_us)
        if self.auto_enable:
            self.pwm.change_duty_cycle(duty_cycle)
        self.get_logger().debug(f"Set pulse to {pulse_us} us ({duty_cycle:.2f}% duty)")

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
            enable = bool(msg.data)
            if enable:
                if self._current_pulse_us is None:
                    self.set_angle(self.neutral_angle_deg)
                duty_cycle = self.pulse_to_duty_cycle(self._current_pulse_us or self.limits.min_pulse_us)
                self.pwm.start(duty_cycle)
            else:
                self.pwm.stop()
            self.get_logger().info(f"PWM enable set to {enable}")
        except Exception as exc:
            self.get_logger().error(f"Failed to set enable: {exc}")

    def destroy_node(self) -> None:
        try:
            if self._current_pulse_us is not None:
                self.set_angle(self.neutral_angle_deg)
            self.pwm.stop()
            self.pwm.cleanup()
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
