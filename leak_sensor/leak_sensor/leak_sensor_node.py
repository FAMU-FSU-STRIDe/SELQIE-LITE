#!/usr/bin/env python3
"""Reed switch monitor node.

Polls a digital GPIO pin on the Jetson and publishes a Bool indicating whether
the reed switch has triggered. Detection is active-low: a logic LOW on the input
pin means triggered. A servo on a second GPIO pin is driven to a configurable
active or idle duty cycle in response to the switch state.
"""

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LeakSensorNode(Node):
    def __init__(self):
        super().__init__("leak_sensor")

        self.declare_parameter("gpio_pin", 16)
        self.declare_parameter("gpio_mode", "BOARD")
        self.declare_parameter("pull", "UP")
        self.declare_parameter("active_high", False)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("servo_pin", 32)
        self.declare_parameter("servo_hz", 50.0)
        self.declare_parameter("servo_idle_duty", 5.0)    # duty cycle (%) when not triggered
        self.declare_parameter("servo_active_duty", 10.0) # duty cycle (%) when triggered

        self.pin = int(self.get_parameter("gpio_pin").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        self.poll_period = 1.0 / poll_hz if poll_hz > 0 else 0.1

        self.servo_pin = int(self.get_parameter("servo_pin").value)
        servo_hz = float(self.get_parameter("servo_hz").value)
        self.servo_idle_duty = float(self.get_parameter("servo_idle_duty").value)
        self.servo_active_duty = float(self.get_parameter("servo_active_duty").value)

        mode_param = str(self.get_parameter("gpio_mode").value).upper()
        if mode_param == "BCM":
            GPIO.setmode(GPIO.BCM)
            pin_label = f"BCM {self.pin}"
            servo_label = f"BCM {self.servo_pin}"
        else:
            GPIO.setmode(GPIO.BOARD)
            pin_label = f"BOARD {self.pin}"
            servo_label = f"BOARD {self.servo_pin}"

        pull_param = str(self.get_parameter("pull").value).upper()
        if pull_param == "UP":
            pull_cfg = GPIO.PUD_UP
        elif pull_param == "NONE":
            pull_cfg = GPIO.PUD_OFF
        else:
            pull_cfg = GPIO.PUD_DOWN

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull_cfg)

        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, servo_hz)
        self.servo_pwm.start(self.servo_idle_duty)

        self.publisher_ = self.create_publisher(Bool, "leak_detected", 10)
        self.last_state = None
        self.create_timer(self.poll_period, self._poll_sensor)

        self.get_logger().info(
            f"Reed switch monitoring {pin_label} (active_high={self.active_high}, "
            f"poll_hz={poll_hz:.2f}, pull={pull_param}); "
            f"servo on {servo_label} at {servo_hz:.0f} Hz "
            f"(idle={self.servo_idle_duty}%, active={self.servo_active_duty}%)"
        )

    def _poll_sensor(self):
        raw_level = bool(GPIO.input(self.pin))
        triggered = raw_level if self.active_high else not raw_level

        msg = Bool()
        msg.data = triggered
        self.publisher_.publish(msg)

        if triggered != self.last_state:
            state_text = "TRIGGERED" if triggered else "clear"
            raw_text = "HIGH" if raw_level else "LOW"
            self.get_logger().info(
                f"Reed switch state changed: {state_text} (raw={raw_text})"
            )
            self.servo_pwm.ChangeDutyCycle(
                self.servo_active_duty if triggered else self.servo_idle_duty
            )

        self.last_state = triggered

    def destroy_node(self):
        self.servo_pwm.stop()
        GPIO.cleanup(self.pin)
        GPIO.cleanup(self.servo_pin)
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
