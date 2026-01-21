#!/usr/bin/env python3
"""Glider control script for SELQIE."""

from __future__ import annotations

import threading
import time

import rclpy
from std_msgs.msg import Bool, Float32

from selqie_ui.selqie import BeuhlerClock, MotorConsole, StandHold, SwimGait, pack_rgb


COLOR_WHITE = pack_rgb(255, 255, 255)
COLOR_GREEN = pack_rgb(0, 255, 0)
COLOR_BLUE = pack_rgb(0, 0, 255)
COLOR_ORANGE = pack_rgb(255, 165, 0)
COLOR_CYAN = pack_rgb(0, 255, 255)
COLOR_PURPLE = pack_rgb(160, 32, 240)
COLOR_PINK = pack_rgb(255, 105, 180)


class GliderController:
    def __init__(self) -> None:
        self._console = MotorConsole()
        self._swim = SwimGait(self._console)
        self._beuhler = BeuhlerClock(self._console)
        self._stand = StandHold(self._console)
        self._reed_state = False
        self._depth_m: float | None = None
        self._lock = threading.Lock()

        self._console.create_subscription(Bool, "/reed_switch", self._on_reed, 10)
        self._console.create_subscription(Float32, "/bar30/depth_salt", self._on_depth, 10)

    def _on_reed(self, msg: Bool) -> None:
        with self._lock:
            self._reed_state = bool(msg.data)

    def _reed_active(self) -> bool:
        with self._lock:
            return self._reed_state

    def _on_depth(self, msg: Float32) -> None:
        with self._lock:
            self._depth_m = float(msg.data)

    def _depth(self) -> float | None:
        with self._lock:
            return self._depth_m

    def _set_lights(self, color: int) -> None:
        self._console.send_led_colors([color, color])

    def _start_motors(self) -> None:
        self._console.send_special("start", MotorConsole.MOTOR_IDS)

    def _zero_motors(self) -> None:
        targets = MotorConsole.MOTOR_IDS
        self._console.send_idle(targets)
        time.sleep(0.02)
        self._console.send_special("zero", targets)

    def _stop_all_gaits(self) -> None:
        self._swim.stop()
        self._beuhler.stop()

    def _wait_for_reed_hold(self, hold_seconds: float) -> None:
        self._set_lights(COLOR_BLUE)
        flash_colors = [COLOR_WHITE, COLOR_GREEN]

        while rclpy.ok():
            if not self._reed_active():
                time.sleep(0.1)
                continue

            start_time = time.monotonic()
            flash_index = 0
            while rclpy.ok():
                if not self._reed_active():
                    self._set_lights(COLOR_BLUE)
                    break

                elapsed = time.monotonic() - start_time
                if elapsed >= hold_seconds:
                    self._set_lights(COLOR_GREEN)
                    return

                self._set_lights(flash_colors[flash_index])
                flash_index = 1 - flash_index
                time.sleep(0.5)

    def _wait_for_depth_rise(self, rise_m: float) -> None:
        start_depth = None
        while rclpy.ok() and start_depth is None:
            start_depth = self._depth()
            if start_depth is None:
                time.sleep(0.1)

        if start_depth is None:
            return

        target_depth = start_depth - rise_m
        while rclpy.ok():
            current_depth = self._depth()
            if current_depth is None:
                time.sleep(0.1)
                continue
            if current_depth <= target_depth:
                return
            time.sleep(0.1)

    def run(self) -> None:
        self._zero_motors()
        self._console.send_latch_angle(180.0)
        self._wait_for_reed_hold(10.0)

        self._wait_for_depth_rise(0.2)
        self._set_lights(COLOR_BLUE)
        time.sleep(5.0)
        self._set_lights(COLOR_ORANGE)
        self._start_motors()
        self._swim.start(frequency_hz=self._swim.frequency_hz)
        time.sleep(10.0)
        self._swim.stop()
        self._stand.start(position_deg=0.0)
        time.sleep(0.02)
        self._set_lights(COLOR_CYAN)
        self._console.send_latch_angle(0.0)
        time.sleep(2.0)
        self._console.send_latch_angle(180.0)
        time.sleep(18.0)

        self._set_lights(COLOR_PURPLE)
        self._stand.stop()
        time.sleep(1.0)
        self._beuhler.start()
        time.sleep(10.0)
        self._beuhler.stop()
        time.sleep(2)
        self._zero_motors()
        time.sleep(0.02)
        self._stand.start(position_deg=0.0)
        self._set_lights(COLOR_PINK)

        while rclpy.ok():
            time.sleep(1.0)

    def shutdown(self) -> None:
        self._stop_all_gaits()
        self._console.shutdown()


def main() -> None:
    rclpy.init()
    controller = GliderController()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
