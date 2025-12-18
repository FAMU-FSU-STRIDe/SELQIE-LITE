#!/usr/bin/env python3
"""Interactive command console tailored for the SELQIE quadruped."""

from cmd import Cmd
from typing import List

import rclpy

from .selqie import SELQIE


class SELQIETerminal(SELQIE, Cmd):
    """Cmd-based shell that speaks directly to the servo motor topics."""

    def __init__(self) -> None:
        SELQIE.__init__(self)
        Cmd.__init__(self)

    # ---- lifecycle ----------------------------------------------------
    def do_exit(self, line: str) -> bool:  # noqa: D401 - inherited docstring
        return self.handle_exit()

    # ---- special commands --------------------------------------------
    def do_start_motors(self, line: str) -> None:
        """Send the 'start' special command. Usage: start_motors [motor_id|all]"""
        self.start_motors(line)

    def do_stop_motors(self, line: str) -> None:
        """Send the 'exit' special command to stop servo mode. Usage: stop_motors [motor_id|all]"""
        self.stop_motors(line)

    def do_zero(self, line: str) -> None:
        """Zero encoders. Usage: zero [motor_id|all]"""
        self.zero_motors(line)

    def do_origin(self, line: str) -> None:
        """Command motors to return to the origin (0 rad).

        Usage: origin [motor_id|all]
        """

        self.origin(line)

    def do_clear(self, line: str) -> None:
        """Clear commands and hold zeros. Usage: clear [motor_id|all]"""
        self.clear(line)

    def do_beuhler(self, line: str) -> None:
        """Start or stop the Beuhler clock pattern.

        Usage:
          beuhler stop
          beuhler <frequency_hz> [group_offset_deg] [alpha]
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
    def do_set_duty(self, line: str) -> None:
        """Send duty (mode 0). Usage: set_duty <motor_id|all> <duty>"""
        self.set_duty(line)

    def do_set_current(self, line: str) -> None:
        """Send current (mode 1). Usage: set_current <motor_id|all> <amps>"""
        self.set_current(line)

    def do_set_brake(self, line: str) -> None:
        """Send brake current (mode 2). Usage: set_brake <motor_id|all> <amps>"""
        self.set_brake(line)

    def do_set_rpm(self, line: str) -> None:
        """Send ERPM directly (mode 3). Usage: set_rpm <motor_id|all> <erpm>"""
        self.set_rpm(line)

    def do_set_pos(self, line: str) -> None:
        """Send absolute position (degrees) via servo position command (mode 4).

        Usage: set_pos <motor_id|all> <position_deg>
        """

        self.set_pos(line)

    def do_set_pos_rad(self, line: str) -> None:
        """Send absolute position in radians (converted to degrees). Usage: set_pos_rad <motor_id|all> <position_rad>"""
        self.set_pos_rad(line)

    def do_set_pos_spd(self, line: str) -> None:
        """Send position + speed + accel (mode 6).

        Usage: set_pos_spd <motor_id|all> <position_deg> <erpm> [accel_erpm_s]
        """

        self.set_pos_spd(line)

    def do_idle(self, line: str) -> None:
        """Send idle (mode 7). Usage: idle [motor_id|all]"""
        self.idle(line)

    # ---- inspection ---------------------------------------------------
    def do_status(self, line: str) -> None:
        """Print the latest motor states received."""
        self.status(line)

    def do_errors(self, line: str) -> None:
        """Print the latest error strings from each motor."""
        self.errors(line)


def main(argv: List[str] | None = None) -> int:
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
