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
