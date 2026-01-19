"""Core SELQIE console logic shared by interactive frontends."""

import math
import threading
import time
from typing import Iterable, List, Optional

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Float64MultiArray, String, UInt32MultiArray

from motor_interfaces.msg import MotorState


class MotorConsole(Node):
    """ROS 2 helper that publishes commands and captures motor state."""

    MOTOR_IDS = (1, 2, 3, 4)

    def __init__(self) -> None:
        super().__init__('selqie_motor_console')

        # Publishers
        self._cmd_pubs = {
            motor_id: self.create_publisher(
                Float64MultiArray, f'/motor{motor_id}/servo_cmd', 10
            )
            for motor_id in self.MOTOR_IDS
        }
        self._special_pubs = {
            motor_id: self.create_publisher(
                String, f'/motor{motor_id}/special_cmd', 10
            )
            for motor_id in self.MOTOR_IDS
        }
        self._latch_pub = self.create_publisher(Float64, '/latch_angle_cmd', 10)
        self._led_pub = self.create_publisher(UInt32MultiArray, '/led_colors', 10)

        # Subscriptions (state + error) with local caches for printing
        self._state_cache: dict[int, MotorState] = {}
        self._error_cache: dict[int, str] = {}
        self._battery_voltage: Optional[float] = None
        self._battery_voltage_stamp: Optional[float] = None
        self._lock = threading.Lock()

        for motor_id in self.MOTOR_IDS:
            self.create_subscription(
                MotorState,
                f'/motor{motor_id}/motor_state',
                lambda msg, mid=motor_id: self._on_state(mid, msg),
                10,
            )
            # Prefer `current_state` if available because it carries a relative
            # encoder angle without wrap drift; we cache it the same way so the
            # rest of the console can transparently use it.
            self.create_subscription(
                MotorState,
                f'/motor{motor_id}/current_state',
                lambda msg, mid=motor_id: self._on_state(mid, msg),
                10,
            )
            self.create_subscription(
                String,
                f'/motor{motor_id}/error_code',
                lambda msg, mid=motor_id: self._on_error(mid, msg),
                10,
            )

        self.create_subscription(
            Float32,
            '/tinybms/pack_voltage',
            self._on_battery_voltage,
            10,
        )

        # Background executor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._shutdown = False

    # ------------------------------------------------------------------
    # ROS callbacks / caches
    # ------------------------------------------------------------------
    def _on_state(self, motor_id: int, msg: MotorState) -> None:
        with self._lock:
            self._state_cache[motor_id] = msg

    def _on_error(self, motor_id: int, msg: String) -> None:
        with self._lock:
            self._error_cache[motor_id] = msg.data

    def _on_battery_voltage(self, msg: Float32) -> None:
        with self._lock:
            self._battery_voltage = msg.data
            self._battery_voltage_stamp = time.time()

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def send_special(self, command: str, targets: Iterable[int]) -> None:
        msg = String()
        msg.data = command
        for motor_id in targets:
            pub = self._special_pubs.get(motor_id)
            if pub:
                pub.publish(msg)

    def send_servo_cmd(self, target: int, mode: int, v0: float = 0.0, v1: float = 0.0, v2: float = 0.0) -> None:
        msg = Float64MultiArray()
        msg.data = [float(mode), float(v0), float(v1), float(v2)]
        pub = self._cmd_pubs.get(target)
        if pub:
            pub.publish(msg)

    def send_position(self, targets: Iterable[int], position_deg: float) -> None:
        for motor_id in targets:
            # Mode 4 = position loop control on the motor
            self.send_servo_cmd(motor_id, 4, position_deg, 0.0, 0.0)

    def send_idle(self, targets: Iterable[int]) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 7, 0.0, 0.0, 0.0)

    def send_velocity(self, targets: Iterable[int], velocity: float) -> None:
        erpm = velocity * 60.0 / (2.0 * math.pi)
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 3, erpm)

    def send_brake_current(self, targets: Iterable[int], current_a: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 2, current_a)

    def snapshot_states(self) -> dict[int, MotorState]:
        with self._lock:
            return dict(self._state_cache)

    def snapshot_errors(self) -> dict[int, str]:
        with self._lock:
            return dict(self._error_cache)

    def snapshot_battery_voltage(self) -> tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self._battery_voltage, self._battery_voltage_stamp

    def send_latch_angle(self, angle_deg: float) -> None:
        msg = Float64()
        msg.data = float(angle_deg)
        self._latch_pub.publish(msg)

    def send_led_colors(self, colors: Iterable[int]) -> None:
        msg = UInt32MultiArray()
        msg.data = [int(color) for color in colors]
        self._led_pub.publish(msg)

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._executor.shutdown()
        self.destroy_node()
        self._spin_thread.join(timeout=1.0)


#######################################################
########### BEUHLER CLOCK IMPLEMENTATION ##############
#######################################################

## Helper Functions
def _wrap_to_180(x_deg: float) -> float:
    return (x_deg + 180.0) % 360.0 - 180.0


def _sgn(x: float) -> float:
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


def pack_rgb(r: int, g: int, b: int) -> int:
    return ((int(r) & 0xFF) << 16) | ((int(g) & 0xFF) << 8) | (int(b) & 0xFF)


## Beuhler Class
class BeuhlerClock:
    """Threaded implementation of the Beuhler clock position pattern."""

    def __init__(self, console: MotorConsole):
        self._console = console

        # Tunables (defaults mirror the standalone beuhler_clock node)
        self.gait_frequency_hz = 0.5 # 1 / total gait period
        self.alpha = 6.0 # how much faster fast portion is
        self.slow_band_deg = 30.0 # d_theta in stance

        # Constants
        self.group_offset_deg = 180.0
        self.control_hz = 50.0
        self.max_vel_abs_deg_s = math.degrees(20.0)
        self.position_limit_deg = 36000.0

        # Internal state
        self._theta_a_deg = 0.0
        self._theta_b_deg = 0.0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cfg_lock = threading.Lock()

    # ---- core math ---------------------------------------------------
    def _calc_omega_slow_deg_s(self, f_abs: float, alpha: float, slow_band_deg: float) -> float:
        t_slow = alpha / (f_abs * (alpha + 1.0))
        return slow_band_deg / t_slow

    def _calc_omega_fast_deg_s(self, f_abs: float, alpha: float, slow_band_deg: float) -> float:
        t_fast = 1.0 / (f_abs * (alpha + 1.0))
        return (360.0 - slow_band_deg) / t_fast

    def _region_speed_deg_s(self, theta_deg: float, f_hz: float, slow_band_deg: float, alpha: float) -> float:
        if f_hz == 0.0:
            return 0.0

        slow_band_min = -slow_band_deg / 2.0
        slow_band_max = slow_band_deg / 2.0

        phase_deg = _wrap_to_180(theta_deg)
        if slow_band_min <= phase_deg <= slow_band_max:
            omega_mag = self._calc_omega_slow_deg_s(abs(f_hz), alpha, slow_band_deg)
        else:
            omega_mag = self._calc_omega_fast_deg_s(abs(f_hz), alpha, slow_band_deg)

        omega_mag = min(omega_mag, self.max_vel_abs_deg_s)
        return _sgn(f_hz) * omega_mag

    def _nearest_relative_zero_deg(self, pos_deg: float) -> float:
        return 360.0 * round(pos_deg / 360.0)

    # ---- lifecycle ---------------------------------------------------
    def _apply_config(
        self,
        *,
        gait_frequency_hz: float | None = None,
        alpha: float | None = None,
        slow_band_deg: float | None = None,
    ) -> None:
        with self._cfg_lock:
            if gait_frequency_hz is not None:
                self.gait_frequency_hz = gait_frequency_hz
            if alpha is not None:
                self.alpha = alpha
            if slow_band_deg is not None:
                self.slow_band_deg = slow_band_deg

    def start(
        self,
        gait_frequency_hz: float | None = None,
        alpha: float | None = None,
        slow_band_deg: float | None = None,
    ) -> None:
        self._apply_config(
            gait_frequency_hz=gait_frequency_hz,
            alpha = alpha,
            slow_band_deg=slow_band_deg
        )

        if self._thread and self._thread.is_alive():
            # Update in-place without phase reset for real-time tuning.
            return

        self._stop_event.clear()
        self._theta_a_deg = 0.0
        self._theta_b_deg = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        # Stop the gait thread
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

        # Snap each leg to its nearest relative zero (nearest multiple of 360 deg)
        theta_a = self._theta_a_deg
        theta_b = self._theta_b_deg

        motor_order = np.array([1, 4, 2, 3])

        for i, motor in enumerate(motor_order):
            pos = theta_a if i < 2 else theta_b
            target_zero = self._nearest_relative_zero_deg(pos)
            self._console.send_position((int(motor),), target_zero)


    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- runner ------------------------------------------------------
    def _run(self) -> None:
        control_dt = 1.0 / max(self.control_hz, 1e-6)
        self._theta_a_deg = 0.0
        self._theta_b_deg = self.group_offset_deg

        while not self._stop_event.is_set():
            cycle_start = time.monotonic()
            with self._cfg_lock:
                gait_frequency_hz = self.gait_frequency_hz
                alpha = self.alpha
                slow_band_deg = self.slow_band_deg

            f = gait_frequency_hz
            theta_a = self._theta_a_deg
            theta_b = self._theta_b_deg

            vA = self._region_speed_deg_s(theta_a, f, slow_band_deg, alpha)
            vB = self._region_speed_deg_s(theta_b, f, slow_band_deg, alpha)

            self._theta_a_deg = theta_a + vA / self.control_hz
            self._theta_b_deg = theta_b + vB / self.control_hz

            if (
                abs(self._theta_a_deg) >= self.position_limit_deg
                or abs(self._theta_b_deg) >= self.position_limit_deg
            ):
                self._stop_event.set()
                break

            motorOrder = np.array([1, 4, 2, 3])

            for i, motor in enumerate(motorOrder):
                target_pos = self._theta_a_deg if i < 2 else self._theta_b_deg
                self._console.send_position((int(motor),), target_pos)


            # Rate-limit the loop to the configured control frequency
            elapsed = time.monotonic() - cycle_start
            sleep_time = control_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

class SwimGait:
    """Synchronized oscillatory position control gait for all legs."""

    def __init__(self, console: MotorConsole):
        self._console = console

        # Tunables
        self.frequency_hz = 0.5
        self.center_angle = 0.0
        self.delta_angle = 0.5

        # Control settings
        self.control_hz = 50.0
        self.kp = 5.0
        self.kd = 1.0

        # Internal state
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cfg_lock = threading.Lock()

    def _apply_config(
        self,
        *,
        frequency_hz: float,
        center_angle: float | None = None,
        delta_angle: float | None = None,
    ) -> None:
        with self._cfg_lock:
            self.frequency_hz = frequency_hz
            if center_angle is not None:
                self.center_angle = center_angle
            if delta_angle is not None:
                self.delta_angle = abs(delta_angle)

    def start(
        self,
        *,
        frequency_hz: float,
        center_angle: float | None = None,
        delta_angle: float | None = None,
    ) -> None:
        self._apply_config(
            frequency_hz=frequency_hz,
            center_angle=center_angle,
            delta_angle=delta_angle,
        )

        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        start_time = time.time()
        while not self._stop_event.is_set():
            with self._cfg_lock:
                freq = self.frequency_hz
                center = self.center_angle
                delta = self.delta_angle

            elapsed = time.time() - start_time
            omega = 2.0 * math.pi * freq
            position = center if freq == 0.0 else center + delta * math.sin(omega * elapsed)

            for motor_id in MotorConsole.MOTOR_IDS:
                self._console.send_position((motor_id,), position)

            time.sleep(1.0 / self.control_hz)


class StandHold:
    """Continuously command all legs to a fixed zero pose."""

    def __init__(self, console: MotorConsole):
        self._console = console
        self.control_hz = 50.0
        self.position_deg = 0.0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cfg_lock = threading.Lock()

    def _apply_config(self, *, position_deg: float | None = None) -> None:
        with self._cfg_lock:
            if position_deg is not None:
                self.position_deg = position_deg

    def start(self, *, position_deg: float | None = None) -> None:
        self._apply_config(position_deg=position_deg)
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            with self._cfg_lock:
                position = self.position_deg

            for motor_id in MotorConsole.MOTOR_IDS:
                self._console.send_position((motor_id,), position)

            time.sleep(1.0 / self.control_hz)


class SELQIE:
    """Shared SELQIE motor control behaviors."""

    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self) -> None:
        self._console = MotorConsole()
        self._beuhler = BeuhlerClock(self._console)
        self._swim = SwimGait(self._console)
        self._stand = StandHold(self._console)

    # ---- helpers ------------------------------------------------------
    def _parse_targets(self, text: str, default_all: bool = False) -> List[int]:
        tokens = text.split()
        if not tokens:
            return list(MotorConsole.MOTOR_IDS) if default_all else []

        token = tokens[0].lower()
        if token in ('*', 'all'):
            return list(MotorConsole.MOTOR_IDS)

        try:
            motor_id = int(token)
        except ValueError:
            print("Motor id must be 1-4, 'all', or '*'")
            return []

        if motor_id not in MotorConsole.MOTOR_IDS:
            print('Motor id must be between 1 and 4')
            return []
        return [motor_id]

    def set_led_colors(self, color0: int, color1: int) -> None:
        """Publish packed RGB colors for the two WS2812B LEDs."""
        self._console.send_led_colors([color0, color1])

    # ---- lifecycle ----------------------------------------------------
    def handle_exit(self) -> bool:
        print('Exiting...')
        self._beuhler.stop()
        self._swim.stop()
        self._stand.stop()
        self._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        return True

    # ---- special commands --------------------------------------------
    def _stop_stand(self) -> None:
        if self._stand.is_running():
            self._stand.stop()

    def start_motors(self, line: str) -> None:
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('start', targets)

    def stop_motors(self, line: str) -> None:
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        #if targets:
        #    self._console.send_special('exit', targets)

    def zero_motors(self, line: str) -> None:
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('zero', targets)

    def clear(self, line: str) -> None:
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('clear', targets)

    def beuhler(self, line: str) -> None:
        self._stop_stand()
        parts = line.split()
        if not parts:
            print('Usage: beuhler stop | <frequency_hz> [slow_band_deg] [alpha]')
            return

        if parts[0].lower() == 'stop':
            if self._beuhler.is_running():
                self._beuhler.stop()
                self.stop_motors("All")
                print('Beuhler clock and motors stopped.')
            else:
                print('Beuhler clock is not running.')
            return

        try:
            freq = float(parts[0])
            slow_band_deg = float(parts[1]) if len(parts) > 1 else None
            alpha = float(parts[2]) if len(parts) > 2 else None
        except ValueError:
            print('Frequency, slow_band_deg, and alpha must be numeric')
            return

        self.start_motors('All')
        self._swim.stop()
        self._beuhler.start(gait_frequency_hz=freq, slow_band_deg=slow_band_deg, alpha=alpha)
        print('Started Beuhler clock with:', freq, slow_band_deg, alpha)

    def swim(self, line: str) -> None:
        self._stop_stand()
        parts = line.split()
        if not parts:
            print('Usage: swim stop | <frequency_hz> [center_angle] [delta_angle]')
            return

        if parts[0].lower() == 'stop':
            if self._swim.is_running():
                self._swim.stop()
                self.start_motors('All')
                self._stand.start(position_deg=0.0)
                print('Swim gait stopped. Standing.')
            else:
                print('Swim gait is not running.')
            return

        try:
            freq = float(parts[0])
            center = float(parts[1]) if len(parts) > 1 else None
            delta = float(parts[2]) if len(parts) > 2 else None
        except ValueError:
            print('Frequency, center_angle, and delta_angle must be numeric')
            return

        if freq < 0:
            print('Frequency must be non-negative')
            return

        self.start_motors('All')
        self._beuhler.stop()
        self._swim.start(frequency_hz=freq, center_angle=center, delta_angle=delta)
        print('Started swim gait with:', freq, center, delta)

    def stand(self, line: str) -> None:
        if line.strip():
            print('Usage: stand')
            return

        self._beuhler.stop()
        self._swim.stop()
        self.start_motors('All')
        self._stand.start(position_deg=0.0)
        print('Standing: commanding all legs to zero.')

    def idle(self, line: str) -> None:
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_idle(targets)

    def brake_current(self, line: str) -> None:
        self._stop_stand()
        parts = line.split()
        if not parts:
            print('Usage: brake <current_a> [motor_id|all]')
            return

        try:
            current_a = float(parts[0])
        except ValueError:
            print('Brake current must be numeric (amps)')
            return

        target_text = parts[1] if len(parts) > 1 else ''
        targets = self._parse_targets(target_text, default_all=True)
        if targets:
            self._console.send_brake_current(targets, current_a)

    def snap_zero_multiple(self, line: str) -> None:
        """Snap leg position to the nearest multiple of the zero point."""
        self._stop_stand()
        targets = self._parse_targets(line, default_all=True)
        if not targets:
            return

        states = self._console.snapshot_states()
        missing = [motor_id for motor_id in targets if motor_id not in states]
        for motor_id in missing:
            print(f'No state received yet for motor{motor_id}')

        for motor_id in targets:
            state = states.get(motor_id)
            if not state:
                continue
            current_deg = math.degrees(state.position)
            k_floor = math.floor(current_deg / 360.0)
            k_ceil  = k_floor + 1

            cand0 = 360.0 * k_floor
            cand1 = 360.0 * k_ceil

            target_deg = cand0 if abs(current_deg - cand0) <= abs(current_deg - cand1) else cand1
            self._console.send_position((motor_id,), target_deg)
            self._console.send_special('zero', targets)
            
            self._stand.start(position_deg=0.0)
            print(
                f'motor{motor_id}: current={current_deg:.2f} deg -> target={target_deg:.2f} deg'
            )

    def latch(self, line: str) -> None:
        """Command the latch angle. Usage: latch open|close"""
        command = line.strip().lower()
        if command == 'open':
            self._console.send_latch_angle(0.0)
            print('Latch commanded open (0 degrees).')
            return
        if command == 'close':
            self._console.send_latch_angle(180.0)
            print('Latch commanded close (180 degrees).')
            return

        print('Usage: latch open|close')

    # ---- inspection ---------------------------------------------------
    def status(self, line: str) -> None:
        states = self._console.snapshot_states()
        if not states:
            print('No motor state messages received yet.')
            return

        for motor_id in sorted(states):
            state = states[motor_id]
            print(
                f"motor{motor_id}: pos={state.position:.3f} rad, "
                f"abs={state.abs_position:.3f} deg, vel={state.velocity:.3f} rad/s, "
                f"torque={state.torque:.3f} Nm, current={state.current:.3f} A, "
                f"temp={state.temperature} C"
            )

    def errors(self, line: str) -> None:
        errors = self._console.snapshot_errors()
        if not errors:
            print('No error messages received yet.')
            return

        for motor_id in sorted(errors):
            print(f'motor{motor_id}: {errors[motor_id]}')

    def battery_voltage(self, line: str) -> None:
        if line.strip():
            print('Usage: battery')
            return

        voltage, stamp = self._console.snapshot_battery_voltage()
        if voltage is None or stamp is None:
            print('No battery voltage messages received yet.')
            return

        age_s = time.time() - stamp
        print(f'Battery voltage: {voltage:.2f} V (age {age_s:.1f}s)')

    def shutdown(self) -> None:
        self._console.shutdown()
