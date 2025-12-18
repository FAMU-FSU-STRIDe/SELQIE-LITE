#!/usr/bin/env python3
"""
ROS2 Node for CubeMars Servo Mode (VESC-style) control via CAN (socketcan/can0)

This node is derived structurally from motor_node.py (MIT mode) but uses the
Servo-mode CAN packet layout (duty/current/rpm/pos/origin/pos+spd) like servo_can.py.

Topics (default namespace: /<joint_name>/...):
  Publishers:
    - motor_state   (motor_interfaces/msg/MotorState)
    - error_code    (std_msgs/msg/String)

  Subscribers:
    - servo_cmd     (std_msgs/msg/Float64MultiArray)
        Generic command: [mode, value0, value1, value2]
        mode:
          0 duty           value0=duty (-1..1)
          1 current        value0=current_A
          2 current_brake  value0=brake_current_A
          3 rpm            value0=erpm
          4 position       value0=position_deg
          5 set_origin     value0=origin_mode (0/1/2)
          6 pos_spd        value0=position_deg, value1=erpm, value2=accel_erpm_per_s
          7 idle           (no values) -> sends duty=0
    - special_cmd   (std_msgs/msg/String)
        start | exit | zero | clear

Notes:
- Servo-mode packets use *extended CAN IDs* in the form:
    arbitration_id = (motor_id & 0xFF) | ((packet_id & 0xFF) << 8)
  and `is_extended_id=True`.
- Replies are typically sent at arbitration_id = motor_id (extended id may vary by firmware);
  we accept any frame whose low byte matches motor_id and length==8, then parse with the
  servo mode unpacking scheme.

Parameters:
  - can_interface   (string) default: 'can0'
  - can_id          (int)    default: 1
  - motor_type      (string) default: 'AK80-9' (used only for torque scaling if provided)
  - control_hz      (double) default: 50.0
  - joint_name      (string) default: 'joint'
  - auto_start      (bool)   default: False
  - reverse_polarity(bool)   default: False
  - torque_constant (double) default: 1.0  (Nm/A, optional)
  - gear_ratio      (double) default: 1.0

"""
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import can
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState


class ServoPacketID(IntEnum):
    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_SET_POS = 4
    CAN_PACKET_SET_ORIGIN_HERE = 5
    CAN_PACKET_SET_POS_SPD = 6


ERROR_CODES = {
    0: "No Error",
    1: "Over temperature fault",
    2: "Over current fault",
    3: "Over voltage fault",
    4: "Under voltage fault",
    5: "Encoder fault",
    6: "Phase current unbalance fault (hardware may be damaged)",
}


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _i32_to_be_bytes(x: int) -> bytes:
    # signed int32, big-endian
    x &= 0xFFFFFFFF
    return bytes([(x >> 24) & 0xFF, (x >> 16) & 0xFF, (x >> 8) & 0xFF, x & 0xFF])


def _i16_to_be_bytes(x: int) -> bytes:
    x &= 0xFFFF
    return bytes([(x >> 8) & 0xFF, x & 0xFF])


def make_ext_arb(motor_id: int, packet_id: int) -> int:
    # matches servo_can.py convention: controller_id | (packet_id << 8)
    return (motor_id & 0xFF) | ((packet_id & 0xFF) << 8)


def make_power_frame(code: int) -> bytes:
    # same byte pattern used by servo_can.py and MIT mode:
    # 0xFF*7 + code (0xFC on, 0xFD off, 0xFE zero in MIT; in servo-mode we use origin packet instead)
    return b"\xFF" * 7 + bytes([code & 0xFF])



@dataclass
class ServoState:
    position_deg: float = 0.0
    speed_erpm: float = 0.0
    current_a: float = 0.0
    temperature_c: float = 0.0
    error: int = 0


def parse_servo_reply(data: bytes) -> Optional[ServoState]:
    """
    Parse a servo-mode status reply (8 bytes) using the layout in servo_can.py:
      pos_int: int16 => position_deg = pos_int * 0.1
      spd_int: int16 => speed_erpm    = spd_int * 10.0
      cur_int: int16 => current_a     = cur_int * 0.01
      temp:    uint8
      err:     uint8
    """
    if len(data) != 8:
        return None
    pos_int = int.from_bytes(data[0:2], byteorder="big", signed=True)
    spd_int = int.from_bytes(data[2:4], byteorder="big", signed=True)
    cur_int = int.from_bytes(data[4:6], byteorder="big", signed=True)
    temp = data[6]
    err = data[7]
    return ServoState(
        position_deg=pos_int * 0.1,
        speed_erpm=spd_int * 10.0,
        current_a=cur_int * 0.01,
        temperature_c=float(temp),
        error=int(err),
    )

TORQUE_CONSTANTS = {
    "AK10-9": 0.198,  # Nm/A
    "AK70-10": 0.123, # Nm/A
    "AK80-64": 0.136, # Nm/A
    "AK40-10": 0.056,
}

class ServoMotorNode(Node):
    def __init__(self):
        super().__init__("servo_motor_node")

        # ---- ROS Parameters (mirrors motor_node.py) ----
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("can_id", 1)
        self.declare_parameter("motor_type", "AK40-10")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("joint_name", "joint")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("reverse_polarity", False)


        self.iface = str(self.get_parameter("can_interface").value)
        self.can_id = int(self.get_parameter("can_id").value) & 0xFF
        self.motor_type = str(self.get_parameter("motor_type").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.control_dt = 1.0 / max(1e-3, self.control_hz)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.reverse_polarity = bool(self.get_parameter("reverse_polarity").value)

        self.get_logger().info(
            f"\nServo Motor Node\n"
            f"  joint: {self.joint_name}\n"
            f"  can_interface: {self.iface}\n"
            f"  can_id: {self.can_id}\n"
            f"  motor_type: {self.motor_type}\n"
            f"  control_hz: {self.control_hz}\n"
            f"  auto_start: {self.auto_start}\n"
            f"  reverse_polarity: {self.reverse_polarity}\n"
        )


        # ---- CAN Bus ----
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)
        except Exception as e:
            self.get_logger().error(f"Failed to init CAN bus {self.iface}: {e}")
            raise

        # ---- ROS Publishers/Subscribers (mirrors motor_node.py style) ----
        self.pub_err = self.create_publisher(String, f"/{self.joint_name}/error_code", 10)
        self.pub_state = self.create_publisher(MotorState, f"/{self.joint_name}/motor_state", 10)

        # Generic servo command
        self.create_subscription(Float64MultiArray, f"/{self.joint_name}/servo_cmd", self.on_servo_cmd, 10)
        self.create_subscription(String, f"/{self.joint_name}/special_cmd", self.on_special, 10)

        # ---- Internal state ----
        self._lock = threading.Lock()
        # cmd format: [mode, v0, v1, v2]
        self.cmd = [float(7), 0.0, 0.0, 0.0]  # default idle
        self._neutral_hold = False
        self._started = False
        self._stop = False

        # ---- Timers/Threads ----
        self.create_timer(self.control_dt, self._tick_control)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

        if self.auto_start:
            self._power_on()
            self._started = True

    # -------------------- RX loop --------------------
    def _rx_loop(self):
        while not self._stop:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if len(msg.data) != 8:
                continue
            # accept frames for this motor if low byte matches
            if (msg.arbitration_id & 0xFF) != self.can_id:
                continue

            st = parse_servo_reply(bytes(msg.data))
            if st is None:
                continue

            # Apply polarity if configured
            pos_deg = -st.position_deg if self.reverse_polarity else st.position_deg
            spd_erpm = -st.speed_erpm if self.reverse_polarity else st.speed_erpm
            cur_a = -st.current_a if self.reverse_polarity else st.current_a

            # Convert to SI-ish for MotorState message
            # position: deg -> rad (motor-side)
            pos_rad = (pos_deg * 3.141592653589793 / 180.0)
            # speed: ERPM -> rad/s (electrical rpm -> mechanical depends on pole pairs; we expose ERPM as-is scaled to rad/s via rpm)
            # Treat ERPM as rpm here; user can set pole-pair scaling externally if needed.
            vel_rads = (spd_erpm * 2.0 * 3.141592653589793 / 60.0)

            err_msg = String()
            err_msg.data = f"Error Code {st.error}: {ERROR_CODES.get(st.error, 'Unknown error')}"
            self.pub_err.publish(err_msg)

            ms = MotorState()
            ms.name = self.joint_name
            ms.position = pos_rad
            ms.abs_position = pos_rad  # servo mode reply doesn't wrap the same way; keep equal
            ms.velocity = vel_rads
            ms.current = cur_a
            ms.torque = cur_a / TORQUE_CONSTANTS[self.motor_type]
            ms.temperature = int(st.temperature_c)
            self.pub_state.publish(ms)

            # If motor reports an error, log it loudly
            if st.error != 0:
                self.get_logger().error(f"{self.joint_name}: {err_msg.data}")

    # -------------------- Command callbacks --------------------
    def on_servo_cmd(self, msg: Float64MultiArray):
        # Expect at least [mode, v0] and up to [mode, v0, v1, v2]
        if len(msg.data) < 1:
            self.get_logger().warn("servo_cmd expects [mode, v0, v1, v2] (mode required)")
            return

        mode = int(msg.data[0])
        v0 = float(msg.data[1]) if len(msg.data) > 1 else 0.0
        v1 = float(msg.data[2]) if len(msg.data) > 2 else 0.0
        v2 = float(msg.data[3]) if len(msg.data) > 3 else 0.0

        with self._lock:
            self.cmd = [float(mode), v0, v1, v2]
            self._neutral_hold = False

        if not self._started:
            self._power_on()
            self._started = True
            self.get_logger().info(f"Auto-starting {self.joint_name} on first servo_cmd")

    def on_special(self, msg: String):
        m = msg.data.strip().lower()
        if m == "start":
            if not self._started:
                self._power_on()
                self._started = True
                self.get_logger().info(f"Starting {self.joint_name} (servo power_on)")
        elif m == "exit":
            if self._started:
                self._power_off()
                self._started = False
                self.get_logger().info(f"Stopping {self.joint_name} (servo power_off)")
        elif m == "zero":
            # In servo mode, use ORIGIN packet with mode=1 (permanent) by default
            self._send_set_origin(1)
            self.get_logger().info(f"Zeroing {self.joint_name} via set_origin=1")
        elif m == "clear":
            with self._lock:
                self.cmd = [float(7), 0.0, 0.0, 0.0]
                self._neutral_hold = True
            self._send_duty(0.0)
            self.get_logger().info(f"Clearing commands for {self.joint_name} (idle hold)")
        else:
            self.get_logger().warn("Unknown special_cmd. Use: start | exit | zero | clear")

    # -------------------- Control tick --------------------
    def _tick_control(self):
        # If not started, do nothing (but you can still receive replies)
        if not self._started:
            return

        with self._lock:
            mode, v0, v1, v2 = self.cmd
            if self._neutral_hold:
                mode, v0, v1, v2 = float(7), 0.0, 0.0, 0.0

        mode_i = int(mode)

        try:
            if mode_i == 0:
                self._send_duty(v0)
            elif mode_i == 1:
                self._send_current(v0)
            elif mode_i == 2:
                self._send_current_brake(v0)
            elif mode_i == 3:
                self._send_rpm(v0)
            elif mode_i == 4:
                self._send_position_deg(v0)
            elif mode_i == 5:
                self._send_set_origin(int(v0))
            elif mode_i == 6:
                self._send_pos_spd(v0, v1, v2)
            elif mode_i == 7:
                self._send_duty(0.0)
            else:
                self.get_logger().warn(f"Unknown servo mode {mode_i}; using idle")
                self._send_duty(0.0)
        except can.CanError:
            self.get_logger().error(f"CAN send failed for {self.joint_name}")

    # -------------------- Low-level send helpers --------------------
    def _send_ext(self, packet_id: int, payload: bytes):
        arb = make_ext_arb(self.can_id, packet_id)
        # python-can uses len(payload) as DLC; servo_can.py sometimes passes 0; we keep payload length correct.
        msg = can.Message(arbitration_id=arb, data=payload, is_extended_id=True)
        self.bus.send(msg)

    def _power_on(self):
        # In servo_can.py this is sent with arbitration_id=motor_id (extended) and last byte 0xFC
        msg = can.Message(arbitration_id=self.can_id, data=make_power_frame(0xFC), is_extended_id=True)
        self.bus.send(msg)

    def _power_off(self):
        msg = can.Message(arbitration_id=self.can_id, data=make_power_frame(0xFD), is_extended_id=True)
        self.bus.send(msg)

    def _send_duty(self, duty: float):
        duty = _clamp(float(duty), -1.0, 1.0)
        if self.reverse_polarity:
            duty = -duty
        # int32 scaled by 100000 (servo_can.py)
        payload = _i32_to_be_bytes(int(duty * 100000.0))
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_DUTY), payload)

    def _send_current(self, current_a: float):
        cur = float(current_a)
        if self.reverse_polarity:
            cur = -cur
        payload = _i32_to_be_bytes(int(cur * 1000.0))
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_CURRENT), payload)

    def _send_current_brake(self, current_a: float):
        cur = float(current_a)
        # brake current is usually positive; still obey reverse_polarity in case wiring is flipped
        if self.reverse_polarity:
            cur = -cur
        payload = _i32_to_be_bytes(int(cur * 1000.0))
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_CURRENT_BRAKE), payload)

    def _send_rpm(self, erpm: float):
        rpm = float(erpm)
        if self.reverse_polarity:
            rpm = -rpm
        payload = _i32_to_be_bytes(int(rpm))
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_RPM), payload)

    def _send_position_deg(self, position_deg: float):
        pos = float(position_deg)
        if self.reverse_polarity:
            pos = -pos
        payload = _i32_to_be_bytes(int(pos * 1000000.0))
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_POS), payload)

    def _send_set_origin(self, mode: int):
        # mode: 0 temp, 1 permanent, 2 restore default
        m = int(mode) & 0xFF
        payload = bytes([m])
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_ORIGIN_HERE), payload)

    def _send_pos_spd(self, position_deg: float, erpm: float, accel_erpm_per_s: float):
        pos = float(position_deg)
        spd = int(float(erpm))
        acc = int(float(accel_erpm_per_s))
        if self.reverse_polarity:
            pos = -pos
            spd = -spd
            acc = -acc
        # servo_can.py uses int32 pos*10000 + int16 spd + int16 acc
        payload = _i32_to_be_bytes(int(pos * 10000.0)) + _i16_to_be_bytes(spd) + _i16_to_be_bytes(acc)
        self._send_ext(int(ServoPacketID.CAN_PACKET_SET_POS_SPD), payload)

    # -------------------- Shutdown --------------------
    def destroy_node(self):
        # Best-effort stop
        try:
            self._send_duty(0.0)
            self._power_off()
        except Exception:
            pass
        self._stop = True
        try:
            self._rx.join(timeout=0.3)
        except Exception:
            pass
        try:
            if hasattr(self.bus, "shutdown"):
                self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
