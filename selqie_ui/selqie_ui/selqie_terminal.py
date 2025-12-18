#!/usr/bin/env python3
"""Interactive console for controlling SELQIE servo motors.

This tool publishes servo-mode commands and special commands to the four
CubeMars actuators defined in ``servo/launch/servo_motor.launch.py``.
It mirrors the ServoMotorNode topic contract:

* ``/<motor>/servo_cmd`` (``std_msgs/msg/Float64MultiArray``) expects
  ``[mode, value0, value1, value2]`` using the modes documented in
  ``servo_motor_node.py`` (0=duty, 1=current, 2=brake current,
  3=RPM, 4=position, 5=set_origin, 6=position+speed, 7=idle).
* ``/<motor>/special_cmd`` (``std_msgs/msg/String``) forwards lifecycle
  commands such as ``start``, ``exit``, ``zero``, and ``clear``.

Run with ``ros2 run selqie_ui selqie_terminal``. Type ``help`` inside the
prompt for available commands.
"""

import sys
import threading
from typing import Dict, Iterable, List

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from motor_interfaces.msg import MotorState
from std_msgs.msg import Float64MultiArray, String


MOTOR_NAMES: List[str] = ["motor1", "motor2", "motor3", "motor4"]


class SelqieTerminal(Node):
    def __init__(self) -> None:
        super().__init__("selqie_terminal")

        self._servo_pubs = {
            name: self.create_publisher(Float64MultiArray, f"/{name}/servo_cmd", 10)
            for name in MOTOR_NAMES
        }
        self._special_pubs = {
            name: self.create_publisher(String, f"/{name}/special_cmd", 10)
            for name in MOTOR_NAMES
        }
        self._states: Dict[str, MotorState] = {}
        self._lock = threading.Lock()

        for name in MOTOR_NAMES:
            self.create_subscription(
                MotorState,
                f"/{name}/motor_state",
                lambda msg, motor=name: self._on_state(motor, msg),
                10,
            )

        self.get_logger().info("SELQIE terminal ready. Type 'help' for commands.")

    def _on_state(self, motor: str, msg: MotorState) -> None:
        with self._lock:
            self._states[motor] = msg

    def _publish_servo(self, target: Iterable[str], mode: float, *values: float) -> None:
        data = [mode, *values]
        for name in target:
            self._servo_pubs[name].publish(Float64MultiArray(data=data))
            self.get_logger().info(f"[{name}] servo_cmd -> {data}")

    def _publish_special(self, target: Iterable[str], command: str) -> None:
        for name in target:
            self._special_pubs[name].publish(String(data=command))
            self.get_logger().info(f"[{name}] special_cmd -> {command}")

    def _resolve_targets(self, raw: str) -> Iterable[str]:
        if raw.lower() == "all":
            return MOTOR_NAMES
        if raw not in MOTOR_NAMES:
            raise ValueError(f"Unknown motor '{raw}'. Choose one of {MOTOR_NAMES} or 'all'.")
        return [raw]

    def print_status(self) -> None:
        with self._lock:
            if not self._states:
                print("No motor_state messages received yet.")
                return
            for name in MOTOR_NAMES:
                state = self._states.get(name)
                if not state:
                    print(f"{name}: (no data)")
                    continue
                print(
                    f"{name}: pos={state.position_deg:.2f} deg, "
                    f"vel={state.velocity_rpm:.1f} rpm, torque={state.torque_nm:.2f} Nm, "
                    f"i={state.current_a:.2f} A, temp={state.temperature_c:.1f} C"
                )

    def handle_command(self, line: str) -> bool:
        tokens = line.strip().split()
        if not tokens:
            return True

        cmd, *args = tokens
        cmd = cmd.lower()

        try:
            if cmd in {"exit", "quit"}:
                self._publish_special(MOTOR_NAMES, "exit")
                return False
            if cmd == "help":
                self._print_help()
                return True
            if cmd == "start":
                self._publish_special(self._resolve_targets(args[0] if args else "all"), "start")
                return True
            if cmd in {"stop", "stop_motors"}:
                self._publish_special(self._resolve_targets(args[0] if args else "all"), "exit")
                return True
            if cmd == "zero":
                self._publish_special(self._resolve_targets(args[0] if args else "all"), "zero")
                return True
            if cmd == "clear":
                self._publish_special(self._resolve_targets(args[0] if args else "all"), "clear")
                return True
            if cmd == "status":
                self.print_status()
                return True
            if cmd == "errors":
                self._publish_special(self._resolve_targets(args[0] if args else "all"), "errors")
                return True
            if cmd == "duty":
                target, duty = args[0], float(args[1])
                self._publish_servo(self._resolve_targets(target), 0.0, duty)
                return True
            if cmd == "current":
                target, amps = args[0], float(args[1])
                self._publish_servo(self._resolve_targets(target), 1.0, amps)
                return True
            if cmd in {"brake", "current_brake"}:
                target, amps = args[0], float(args[1])
                self._publish_servo(self._resolve_targets(target), 2.0, amps)
                return True
            if cmd == "rpm":
                target, rpm = args[0], float(args[1])
                self._publish_servo(self._resolve_targets(target), 3.0, rpm)
                return True
            if cmd == "pos":
                target, degrees = args[0], float(args[1])
                self._publish_servo(self._resolve_targets(target), 4.0, degrees)
                return True
            if cmd == "origin":
                target, mode = args[0], float(args[1]) if len(args) > 1 else 0.0
                self._publish_servo(self._resolve_targets(target), 5.0, mode)
                return True
            if cmd in {"pos_spd", "position_speed"}:
                target = args[0]
                position = float(args[1])
                rpm = float(args[2])
                accel = float(args[3]) if len(args) > 3 else 0.0
                self._publish_servo(self._resolve_targets(target), 6.0, position, rpm, accel)
                return True
            if cmd == "idle":
                target = args[0] if args else "all"
                self._publish_servo(self._resolve_targets(target), 7.0)
                return True
        except (IndexError, ValueError) as exc:
            print(f"Error: {exc}")
            return True

        print(f"Unknown command '{cmd}'. Type 'help' for options.")
        return True

    def _print_help(self) -> None:
        print(
            "Commands:\n"
            "  start [motor|all]           Start the selected motors\n"
            "  stop|stop_motors [target]   Stop motors (exit)\n"
            "  zero [target]               Zero encoders\n"
            "  clear [target]              Clear fault state\n"
            "  status                      Print latest motor_state\n"
            "  errors [target]             Request error string\n"
            "  duty <target> <duty>        Set duty cycle (-1..1)\n"
            "  current <target> <amps>     Set phase current (A)\n"
            "  brake <target> <amps>       Apply braking current (A)\n"
            "  rpm <target> <erpm>         Set speed (eRPM)\n"
            "  pos <target> <deg>          Move to position (deg)\n"
            "  origin <target> [mode]      Set origin (0/1/2)\n"
            "  pos_spd <target> <deg> <erpm> [accel]  Position + speed\n"
            "  idle [target]               Send idle packet (mode 7)\n"
            "  exit|quit                   Exit the console"
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SelqieTerminal()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            try:
                line = input("selqie> ")
            except EOFError:
                break
            if not node.handle_command(line):
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        sys.exit(0)


if __name__ == "__main__":
    main()
