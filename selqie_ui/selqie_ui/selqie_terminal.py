#!/usr/bin/env python3
"""Interactive command console tailored for the SELQIE quadruped."""

import math
import threading
import time
from cmd import Cmd
from typing import Iterable, List
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

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

        # Subscriptions (state + error) with local caches for printing
        self._state_cache: dict[int, MotorState] = {}
        self._error_cache: dict[int, str] = {}
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

    def send_duty(self, targets: Iterable[int], duty: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 0, duty)

    def send_current(self, targets: Iterable[int], current: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 1, current)

    def send_brake_current(self, targets: Iterable[int], current: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 2, current)

    def send_rpm(self, targets: Iterable[int], erpm: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 3, erpm)

    def send_position_deg(self, targets: Iterable[int], position_deg: float) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 4, position_deg)

    def send_position_rad(self, targets: Iterable[int], position_rad: float) -> None:
        self.send_position_deg(targets, math.degrees(position_rad))

    def send_origin(self, targets: Iterable[int], mode: int = 1) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 5, mode)

    def send_position_speed(
        self, targets: Iterable[int], position_rad: float, velocity_rad_s: float, accel_rad_s2: float = 0.0
    ) -> None:
        position_deg = math.degrees(position_rad)
        erpm = velocity_rad_s * 60.0 / (2.0 * math.pi)
        accel_erpm_s = accel_rad_s2 * 60.0 / (2.0 * math.pi)
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 6, position_deg, erpm, accel_erpm_s)

    def send_idle(self, targets: Iterable[int]) -> None:
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 7, 0.0, 0.0, 0.0)

    def send_velocity(self, targets: Iterable[int], velocity: float) -> None:
        erpm = velocity * 60.0 / (2.0 * math.pi)
        for motor_id in targets:
            self.send_servo_cmd(motor_id, 3, erpm)

    def snapshot_states(self) -> dict[int, MotorState]:
        with self._lock:
            return dict(self._state_cache)

    def snapshot_errors(self) -> dict[int, str]:
        with self._lock:
            return dict(self._error_cache)

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
def _wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _sgn(x: float) -> float:
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


## Beuhler Class
class BeuhlerClock:
    """Threaded implementation of the Beuhler clock velocity pattern."""

    def __init__(self, console: MotorConsole):
        self._console = console

        # Tunables (defaults mirror the standalone beuhler_clock node)
        self.gait_frequency_hz = 0.5 # 1 / total gait period
        self.alpha = 6.0 # how much faster fast portion is
        self.slow_band_deg = 30.0 # d_theta in stance

        # Constants
        self.group_offset_deg = 180.0
        self.control_hz = 100.0
        self.kp = 0.0
        self.kd = 1.0
        self.max_vel_abs = 20.0
        
        # Internal state
        self._theta_base = 0.0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cfg_lock = threading.Lock()

    # ---- core math ---------------------------------------------------
    def _calcOmegaSlow(self, f_abs: float, alpha: float) -> float:
        t = alpha/(f_abs * (alpha + 1))
        omegaSlow = (2* math.pi - math.radians(self.slow_band_deg)) / t
        return (omegaSlow)
    
    def _calcOmegaFast(self, f_abs: float, alpha: float) -> float:
        t = 1/(f_abs * (alpha + 1))
        omegaFast = math.radians(self.slow_band_deg) / t
        return (omegaFast)

    def _feedback_theta(self, motor: int) -> float:
        """Return the latest measured leg angle (rad) if available."""

        # Users report `/motor2/current_state` publishes the relative encoder
        # angle; we read motor 2 by default but keep the method general in case
        # we want to make the source configurable later.
        state = self._console.snapshot_states().get(motor)
        if state is None:
            return 0.0
            # return None
        return _wrap_to_pi(float(state.position))

    def _region_speed(self, theta: float, f_hz: float, slow_band_deg: float, alpha: float) -> float:
        if f_hz == 0.0:
            return 0.0
        
        slow_band_min = - slow_band_deg / 2
        slow_band_max =  slow_band_deg / 2
        
        if ( _wrap_to_pi(theta) <= math.radians(slow_band_max)) and ( _wrap_to_pi(theta) >= math.radians(slow_band_min)):
            omega_mag = self._calcOmegaSlow(abs(f_hz), alpha)
        else:
            omega_mag = self._calcOmegaFast(abs(f_hz), alpha)
            
        if omega_mag <= self.max_vel_abs:
            return _sgn(f_hz) * omega_mag
        else:
            return _sgn(f_hz) * self.max_vel_abs

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
        self._theta_base = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- runner ------------------------------------------------------
    def _run(self) -> None:
        control_dt = 1.0 / max(self.control_hz, 1e-6)
        last_feedback_theta = self._feedback_theta(2)
        cur_feedback_theta = last_feedback_theta

        while not self._stop_event.is_set():
            cycle_start = time.monotonic()
            with self._cfg_lock:
                gait_frequency_hz = self.gait_frequency_hz
                alpha = self.alpha
                slow_band_deg = self.slow_band_deg

            f = gait_frequency_hz
            
            # For now, going to trust that none of the motors slip, so we can rely on one reading alone
            trustworthy_motor = 2
            cur_feedback_theta = self._feedback_theta(trustworthy_motor)
            d_theta = _wrap_to_pi(cur_feedback_theta - last_feedback_theta)

            # Add the change in angle to the base
            self._theta_base += d_theta

            last_feedback_theta = cur_feedback_theta

            theta_a = self._theta_base
            theta_b = self._theta_base + math.radians(self.group_offset_deg)
            
            vA = self._region_speed(theta_a, f, slow_band_deg, alpha)
            vB = self._region_speed(theta_b, f, slow_band_deg, alpha)
            
            motorOrder = np.array([1, 4, 2, 3])
            
            for i, motor in enumerate(motorOrder):
                if i <= 2:
                    curV = vA
                else:
                    curV = vB
                self._console.send_velocity((motor,), curV, kp=self.kp, kd=self.kd)

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
        self.control_hz = 100.0
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
            velocity = 0.0 if freq == 0.0 else omega * delta * math.cos(omega * elapsed)

            for motor_id in MotorConsole.MOTOR_IDS:
                self._console.send_position_speed((motor_id,), position, velocity)

            time.sleep(1.0 / self.control_hz)

#######################################################
###################### TERMINAL #######################
#######################################################

class SELQIETerminal(Cmd):
    """Cmd-based shell that speaks directly to the servo motor topics."""

    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self) -> None:
        super().__init__()
        self._console = MotorConsole()
        self._beuhler = BeuhlerClock(self._console)
        self._swim = SwimGait(self._console)

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

    # ---- lifecycle ----------------------------------------------------
    def do_exit(self, line: str) -> bool:
        """Exit the terminal."""
        print('Exiting...')
        self._beuhler.stop()
        self._swim.stop()
        self._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        return True

    # ---- special commands --------------------------------------------
    def do_start_motors(self, line: str) -> None:
        """Send the 'start' special command. Usage: start_motors [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('start', targets)

    def do_stop_motors(self, line: str) -> None:
        """Send the 'exit' special command to stop servo mode. Usage: stop_motors [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('exit', targets)

    def do_zero(self, line: str) -> None:
        """Zero encoders. Usage: zero [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('zero', targets)

    def do_origin(self, line: str) -> None:
        """Command motors to return to the origin (0 rad).

        Usage: origin [motor_id|all]
        """

        targets = self._parse_targets(line, default_all=True)
        if not targets:
            return

        self._console.send_position_rad(targets, 0.0)

    def do_clear(self, line: str) -> None:
        """Clear commands and hold zeros. Usage: clear [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('clear', targets)

    def do_beuhler(self, line: str) -> None:
        """Start or stop the Beuhler clock pattern.

        Usage:
          beuhler stop
          beuhler <frequency_hz> [group_offset_deg] [alpha]
        """

        parts = line.split()
        if not parts:
            print('Usage: beuhler stop | <frequency_hz> [slow_band_deg] [alpha]')
            return

        if parts[0].lower() == 'stop':
            if self._beuhler.is_running():
                self._beuhler.stop()
                self.do_stop_motors("All")
                print('Beuhler clock and motors stopped.')
            else:
                print('Beuhler clock was not running.')
            return

        try:
            values = [float(p) for p in parts]
        except ValueError:
            print('All parameters must be numeric. Usage: beuhler <frequency_hz> [slow_band_deg] [alpha]')
            return

        params = {
            'gait_frequency_hz': values[0],
            'slow_band_deg': values[1] if len(values) > 1 else None,
            'alpha': values[2] if len(values) > 2 else None,
        }

        was_running = self._beuhler.is_running()
        self._beuhler.start(**params)
        print(
            ('Beuhler clock updated: ' if was_running else 'Beuhler clock running: '),
            f"f={self._beuhler.gait_frequency_hz:+.3f} Hz, "
            f"band=±{self._beuhler.slow_band_deg:.1f}°, alpha={self._beuhler.alpha}"
        )

    def do_swim(self, line: str) -> None:
        """Start or stop the synchronized swim gait.

        Usage:
          swim stop
          swim <frequency_hz> [center_angle_deg] [delta_angle_deg]
        """

        parts = line.split()
        if not parts:
            print('Usage: swim stop | <frequency_hz> [center_angle_deg] [delta_angle_deg]')
            return

        if parts[0].lower() == 'stop':
            if self._swim.is_running():
                self._swim.stop()
                self.do_stop_motors('All')
                print('Swim gait and motors stopped.')
            else:
                print('Swim gait was not running.')
            return

        try:
            frequency_hz = float(parts[0])
            center_angle_deg = float(parts[1]) if len(parts) > 1 else None
            delta_angle_deg = float(parts[2]) if len(parts) > 2 else None
        except ValueError:
            print('All parameters must be numeric. Usage: swim <frequency_hz> [center_angle_deg] [delta_angle_deg]')
            return

        self._swim.start(
            frequency_hz=frequency_hz,
            center_angle=math.radians(center_angle_deg) if center_angle_deg is not None else None,
            delta_angle=math.radians(delta_angle_deg) if delta_angle_deg is not None else None,
        )
        print(
            'Swim gait running: '
            f"f={self._swim.frequency_hz:+.3f} Hz, center={math.degrees(self._swim.center_angle):+.1f}°, "
            f"delta={math.degrees(self._swim.delta_angle):.1f}°"
        )

    # ---- Servo command helpers ---------------------------------------
    def do_set_cmd(self, line: str) -> None:
        """Send a servo position+speed command (mode 6).

        Usage: set_cmd <motor_id|all> <position_rad> <velocity_rad_s> [kp] [kd] [torque]

        The MIT gains/torque parameters are accepted for compatibility but are
        ignored; position/velocity are converted to degrees/ERPM for servo
        mode.
        """

        parts = line.split()
        if len(parts) < 3:
            print('Usage: set_cmd <motor_id|all> <position_rad> <velocity_rad_s> [kp] [kd] [torque]')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return
        try:
            p_val = float(parts[1])
            v_val = float(parts[2])
        except ValueError:
            print('Position and velocity must be numeric')
            return

        self._console.send_position_speed(targets, p_val, v_val)

    def do_set_vel(self, line: str) -> None:
        """Set target motor speed via servo RPM command (mode 3).

        Usage: set_vel <motor_id|all> <velocity_rad_s>
        """
        parts = line.split()
        if len(parts) < 2:
            print('Usage: set_vel <motor_id|all> <velocity_rad_s>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            vel = float(parts[1])
        except ValueError:
            print('Velocity must be numeric')
            return

        self._console.send_velocity(targets, vel)

    def do_set_duty(self, line: str) -> None:
        """Send raw duty cycle (mode 0). Usage: set_duty <motor_id|all> <duty[-1..1]>"""
        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_duty <motor_id|all> <duty[-1..1]>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            duty = float(parts[1])
        except ValueError:
            print('Duty must be numeric')
            return

        self._console.send_duty(targets, duty)

    def do_set_current(self, line: str) -> None:
        """Send torque current (mode 1). Usage: set_current <motor_id|all> <amps>"""
        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_current <motor_id|all> <amps>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            current = float(parts[1])
        except ValueError:
            print('Current must be numeric')
            return

        self._console.send_current(targets, current)

    def do_set_brake(self, line: str) -> None:
        """Send brake current (mode 2). Usage: set_brake <motor_id|all> <amps>"""
        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_brake <motor_id|all> <amps>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            current = float(parts[1])
        except ValueError:
            print('Current must be numeric')
            return

        self._console.send_brake_current(targets, current)

    def do_set_rpm(self, line: str) -> None:
        """Send ERPM directly (mode 3). Usage: set_rpm <motor_id|all> <erpm>"""
        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_rpm <motor_id|all> <erpm>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            erpm = float(parts[1])
        except ValueError:
            print('ERPM must be numeric')
            return

        self._console.send_rpm(targets, erpm)

    def do_set_pos(self, line: str) -> None:
        """Send absolute position (degrees) via servo position command (mode 4).

        Usage: set_pos <motor_id|all> <position_deg>
        """

        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_pos <motor_id|all> <position_deg>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            pos_deg = float(parts[1])
        except ValueError:
            print('Position must be numeric')
            return

        self._console.send_position_deg(targets, pos_deg)

    def do_set_pos_rad(self, line: str) -> None:
        """Send absolute position in radians (converted to degrees). Usage: set_pos_rad <motor_id|all> <position_rad>"""
        parts = line.split()
        if len(parts) != 2:
            print('Usage: set_pos_rad <motor_id|all> <position_rad>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            pos_rad = float(parts[1])
        except ValueError:
            print('Position must be numeric')
            return

        self._console.send_position_rad(targets, pos_rad)

    def do_set_pos_spd(self, line: str) -> None:
        """Send position + speed + accel (mode 6).

        Usage: set_pos_spd <motor_id|all> <position_deg> <erpm> [accel_erpm_s]
        """

        parts = line.split()
        if len(parts) < 3:
            print('Usage: set_pos_spd <motor_id|all> <position_deg> <erpm> [accel_erpm_s]')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            pos_deg = float(parts[1])
            erpm = float(parts[2])
            accel = float(parts[3]) if len(parts) > 3 else 0.0
        except ValueError:
            print('Position, ERPM, and accel must be numeric')
            return

        for motor_id in targets:
            self._console.send_servo_cmd(motor_id, 6, pos_deg, erpm, accel)

    def do_idle(self, line: str) -> None:
        """Send idle (mode 7). Usage: idle [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_idle(targets)

    # ---- inspection ---------------------------------------------------
    def do_status(self, line: str) -> None:
        """Print the latest motor states received."""
        states = self._console.snapshot_states()
        if not states:
            print('No motor state messages received yet.')
            return

        for motor_id in sorted(states):
            state = states[motor_id]
            print(
                f"motor{motor_id}: pos={state.position:.3f} rad, "
                f"abs={state.abs_position:.3f} rad, vel={state.velocity:.3f} rad/s, "
                f"torque={state.torque:.3f} Nm, current={state.current:.3f} A, "
                f"temp={state.temperature} C"
            )

    def do_errors(self, line: str) -> None:
        """Print the latest error strings from each motor."""
        errors = self._console.snapshot_errors()
        if not errors:
            print('No error messages received yet.')
            return

        for motor_id in sorted(errors):
            print(f'motor{motor_id}: {errors[motor_id]}')


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    terminal = SELQIETerminal()
    try:
        terminal.cmdloop()
    finally:
        terminal._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    raise SystemExit(main())
