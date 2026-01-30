#!/usr/bin/env python3
"""Interactive command console tailored for the SELQIE quadruped."""

from cmd import Cmd
from datetime import datetime
from pathlib import Path
import signal
import subprocess
from typing import List

import rclpy

from .selqie import MotorConsole, SELQIE


class SELQIETerminal(SELQIE, Cmd):
    """Cmd-based shell that speaks directly to the servo motor topics."""

    def __init__(self) -> None:
        SELQIE.__init__(self)
        Cmd.__init__(self)
        self._bag_process: subprocess.Popen[str] | None = None
        self._bag_output_dir: Path | None = None

    def _recording_topics(self) -> list[str]:
        topics: list[str] = []
        for motor_id in MotorConsole.MOTOR_IDS:
            topics.extend(
                [
                    f'/motor{motor_id}/servo_cmd',
                    f'/motor{motor_id}/special_cmd',
                    f'/motor{motor_id}/motor_state',
                    f'/motor{motor_id}/current_state',
                    f'/motor{motor_id}/error_code',
                ]
            )
        topics.extend(
            [
                '/latch_angle_cmd',
                '/led_colors',
                '/tinybms/pack_voltage',
            ]
        )
        return topics

    def start_recording(self, line: str) -> None:
        """Start recording all SELQIE custom topics into a ros2 bag.

        Usage: start_recording <output_dir>
        """
        if self._bag_process and self._bag_process.poll() is None:
            print('ros2 bag recording already in progress.')
            return

        output_root = line.strip()
        if not output_root:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_root = str(Path('rosbags') / f'selqie_{timestamp}')

        output_dir = Path(output_root).expanduser()
        output_dir.parent.mkdir(parents=True, exist_ok=True)

        cmd = ['ros2', 'bag', 'record', '-o', str(output_dir), *self._recording_topics()]
        try:
            self._bag_process = subprocess.Popen(cmd)
        except FileNotFoundError:
            print('Unable to start ros2 bag recording: ros2 CLI not found.')
            self._bag_process = None
            return

        self._bag_output_dir = output_dir
        print(f'Started ros2 bag recording to {output_dir}')

    def stop_recording(self, line: str) -> None:
        """Stop an active ros2 bag recording."""
        if not self._bag_process or self._bag_process.poll() is not None:
            print('No ros2 bag recording is currently active.')
            self._bag_process = None
            self._bag_output_dir = None
            return

        process = self._bag_process
        output_dir = self._bag_output_dir
        self._bag_process = None
        self._bag_output_dir = None

        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)

        if output_dir:
            print(f'Stopped ros2 bag recording. Output saved to {output_dir}')

    # ---- lifecycle ----------------------------------------------------
    def do_exit(self, line: str) -> bool:  # noqa: D401 - inherited docstring
        self.stop_recording('')
        return self.handle_exit()

    # ---- special commands --------------------------------------------
    def do_zero(self, line: str) -> None:
        """Zero encoders. Usage: zero [motor_id|all]"""
        self.zero_motors(line)

    def do_clear(self, line: str) -> None:
        """Clear commands and hold zeros. Usage: clear [motor_id|all]"""
        self.clear(line)

    def do_beuhler(self, line: str) -> None:
        """Start or stop the position-controlled Beuhler clock pattern.

        Usage:
          beuhler stop
          beuhler <frequency_hz> [slow_band_deg] [alpha]
        """

        self.beuhler(line)

    def do_swim(self, line: str) -> None:
        """Start or stop the swim gait.

        Usage:
          swim stop
          swim <frequency_hz> [center_angle] [delta_angle]
        """
        self.swim(line)

    # ---- servo commands ----------------------------------------------
    def do_stand(self, line: str) -> None:
        """Continuously command all legs to zero. Usage: stand"""
        self.stand(line)

    def do_idle(self, line: str) -> None:
        """Send idle (mode 7). Usage: idle [motor_id|all]"""
        self.idle(line)

    def do_brake(self, line: str) -> None:
        """Apply braking current (mode 2). Usage: brake <current_a> [motor_id|all]"""
        self.brake_current(line)

    def do_snap(self, line: str) -> None:
        """Snap to nearest zero multiple. Usage: snap [motor_id|all]"""
        self.snap_zero_multiple(line)

    def do_latch(self, line: str) -> None:
        """Command latch servo. Usage: latch open|close"""
        self.latch(line)

    # ---- inspection ---------------------------------------------------
    def do_status(self, line: str) -> None:
        """Print the latest motor states received."""
        self.status(line)
        
    def do_errors(self, line: str) -> None:
        """Print the latest error strings from each motor."""
        self.errors(line)

    def do_battery(self, line: str) -> None:
        """Print the latest battery voltage reading. Usage: battery"""
        self.battery_voltage(line)

    # ---- recording ---------------------------------------------------
    def do_start_recording(self, line: str) -> None:
        """Start recording custom topics. Usage: start_recording <output_dir>"""
        self.start_recording(line)

    def do_stop_recording(self, line: str) -> None:
        """Stop the active recording. Usage: stop_recording"""
        self.stop_recording(line)


def main(argv: List[str] | None = None) -> int:
    rclpy.init(args=argv)
    terminal = SELQIETerminal()
    try:
        terminal.cmdloop()
    finally:
        terminal.stop_recording('')
        terminal._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    raise SystemExit(main())
