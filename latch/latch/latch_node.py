#!/usr/bin/env python3
"""ROS 2 node to command a servo latch via Jetson GPIO PWM on pin 32."""
from __future__ import annotations

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


# Standard servo PWM: 50 Hz, 1 ms–2 ms pulse
_PWM_HZ = 50
_MIN_DUTY = 2.5   # ~0 deg  (1 ms / 20 ms)
_MAX_DUTY = 12.5  # ~180 deg (2.5 ms / 20 ms)


def _angle_to_duty(angle_deg: float) -> float:
    angle_deg = max(0.0, min(180.0, angle_deg))
    return _MIN_DUTY + (angle_deg / 180.0) * (_MAX_DUTY - _MIN_DUTY)


class LatchNode(Node):
    def __init__(self) -> None:
        super().__init__("latch_node")

        self.declare_parameter("servo_pin", 32)
        self.declare_parameter("gpio_mode", "BOARD")

        self.pin = int(self.get_parameter("servo_pin").value)
        mode = str(self.get_parameter("gpio_mode").value).upper()

        GPIO.setmode(GPIO.BCM if mode == "BCM" else GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        self._pwm = GPIO.PWM(self.pin, _PWM_HZ)
        self._pwm.start(_angle_to_duty(90.0))  # start at neutral

        self.create_subscription(Float64, "latch_angle_cmd", self.on_latch_cmd, 10)

        self.get_logger().info(
            f"Latch servo node ready: BOARD pin {self.pin}, {_PWM_HZ} Hz PWM"
        )

    def on_latch_cmd(self, msg: Float64) -> None:
        angle_deg = float(msg.data)
        duty = _angle_to_duty(angle_deg)
        self._pwm.ChangeDutyCycle(duty)
        self.get_logger().debug(f"Servo angle {angle_deg:.1f} deg -> duty {duty:.2f}%")

    def destroy_node(self) -> None:
        try:
            self._pwm.stop()
        except Exception:
            pass
        GPIO.cleanup(self.pin)
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
