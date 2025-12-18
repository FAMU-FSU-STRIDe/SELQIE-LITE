#!/usr/bin/env python3
"""
ROS2 Node for Cubemars AK (Servo Mode) Control via CAN (SocketCAN)

Servo mode uses EXTENDED CAN frames:
  ExtID bits [28:8] = control mode, [7:0] = source node ID (motor_id) :contentReference[oaicite:4]{index=4}

Common scalings:
- Position mode command: int32(pos_deg * 10000) :contentReference[oaicite:5]{index=5}
- Pos-Speed mode payload: pos(int32)*10000, spd(int16), accel(int16) where 1 LSB = 10 ERPM (they divide by 10 before packing) :contentReference[oaicite:6]{index=6}
- Upload feedback (8B): pos(int16)*0.1deg, spd(int16)*10 ERPM, cur(int16)*0.01A, temp(int8), err(uint8) :contentReference[oaicite:7]{index=7}
"""

import threading
import time
import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState


# Control modes (manual enum) :contentReference[oaicite:8]{index=8}
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4
CAN_PACKET_SET_ORIGIN_HERE = 5
CAN_PACKET_SET_POS_SPD = 6


def _i16_from_be(b0: int, b1: int) -> int:
    v = (b0 << 8) | b1
    return v - 0x10000 if v & 0x8000 else v


def _to_be_i32(v: int) -> bytes:
    v &= 0xFFFFFFFF
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def _to_be_i16(v: int) -> bytes:
    v &= 0xFFFF
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


class ServoMotorNode(Node):
    def __init__(self):
        super().__init__("servo_motor_node")

        # ---- Params ----
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("motor_id", 1)              # node id in [7:0]
        self.declare_parameter("joint_name", "ak_joint")
        self.declare_parameter("control_hz", 200.0)
        self.declare_parameter("mode", "pos_spd")          # pos|pos_spd|rpm|current|duty|brake_current
        self.declare_parameter("reverse_polarity", False)
        self.declare_parameter("pole_pairs", 0)            # optional for velocity conversion
        self.declare_parameter("command_timeout_s", 0.25)

        self.iface = self.get_parameter("can_interface").value
        self.motor_id = int(self.get_parameter("motor_id").value) & 0xFF
        self.joint_name = self.get_parameter("joint_name").value
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.reverse = bool(self.get_parameter("reverse_polarity").value)
        self.pole_pairs = int(self.get_parameter("pole_pairs").value)
        self.cmd_timeout = float(self.get_parameter("command_timeout_s").value)

        self.get_logger().info(
            f"ServoMotorNode on {self.iface}, motor_id={self.motor_id}, mode={self.mode}, hz={self.control_hz}"
        )

        # ---- CAN ----
        self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)

        # Try to filter to frames whose low 8 bits match this motor_id (extended IDs).
        # Not all backends support extended filters; if it fails, we just software-filter.
        try:
            self.bus.set_filters([{"can_id": self.motor_id, "can_mask": 0xFF, "extended": True}])
        except Exception:
            pass

        # ---- ROS pubs/subs ----
        self.pub_joint = self.create_publisher(JointState, f"/{self.joint_name}/joint_state", 10)
        self.pub_err = self.create_publisher(String, f"/{self.joint_name}/error", 10)

        # Commands:
        #  - pos mode:    [pos_deg]
        #  - rpm mode:    [speed_erpm]
        #  - current:     [current_a]
        #  - duty:        [duty_0to1]
        #  - pos_spd:     [pos_deg, speed_erpm, accel_erpm_s2]
        self.create_subscription(Float64MultiArray, f"/{self.joint_name}/servo_cmd", self.on_cmd, 10)

        # Special commands: "origin_temp", "origin_perm", "stop"
        self.create_subscription(String, f"/{self.joint_name}/special_cmd", self.on_special, 10)

        # ---- State ----
        self._lock = threading.Lock()
        self._last_cmd_time = 0.0
        self._cmd = [0.0, 0.0, 0.0]  # pos_deg, speed_erpm, accel_erpm_s2

        # Feedback cache
        self._fb_pos_deg = 0.0
        self._fb_speed_erpm = 0.0
        self._fb_current_a = 0.0
        self._fb_temp_c = 0
        self._fb_err = 0
        self._fb_stamp = 0.0

        # Unwrap position (servo upload pos is int16 deg*0.1, limited span)
        self._last_pos_deg = None
        self._pos_abs_deg = 0.0
        self._unwrap_span_deg = 6400.0  # upload spec is roughly ±3200° :contentReference[oaicite:9]{index=9}

        # Timers & RX thread
        self.create_timer(1.0 / max(self.control_hz, 1.0), self._tick)
        self._stop = False
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

    # ---- ID & send helpers ----
    def _ext_id(self, control_mode: int) -> int:
        # ExtID = (control_mode << 8) | node_id :contentReference[oaicite:10]{index=10}
        return ((int(control_mode) & 0x1FFFFF) << 8) | self.motor_id

    def _send_ext(self, control_mode: int, data: bytes):
        msg = can.Message(
            arbitration_id=self._ext_id(control_mode),
            is_extended_id=True,
            data=data[:8],
        )
        try:
            self.bus.send(msg, timeout=0.01)
        except can.CanError as e:
            self.get_logger().error(f"CAN send failed: {e}")

    # ---- ROS callbacks ----
    def on_cmd(self, msg: Float64MultiArray):
        with self._lock:
            # accept variable lengths; interpret by mode
            if self.mode == "pos":
                if len(msg.data) < 1:
                    return
                self._cmd = [float(msg.data[0]), 0.0, 0.0]
            elif self.mode == "rpm":
                if len(msg.data) < 1:
                    return
                self._cmd = [0.0, float(msg.data[0]), 0.0]
            elif self.mode == "current" or self.mode == "brake_current":
                if len(msg.data) < 1:
                    return
                self._cmd = [0.0, 0.0, float(msg.data[0])]  # store current in slot2 for these modes
            elif self.mode == "duty":
                if len(msg.data) < 1:
                    return
                self._cmd = [float(msg.data[0]), 0.0, 0.0]  # store duty in slot0
            else:  # pos_spd
                if len(msg.data) < 3:
                    return
                self._cmd = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

            self._last_cmd_time = time.time()

    def on_special(self, msg: String):
        m = msg.data.strip().lower()
        if m == "origin_temp":
            # set_origin_mode: 0 = temporary origin :contentReference[oaicite:11]{index=11}
            self._send_ext(CAN_PACKET_SET_ORIGIN_HERE, bytes([0]))
            self.get_logger().info("Set TEMP origin (clears on power-off).")
        elif m == "origin_perm":
            # set_origin_mode: 1 = permanent origin (dual encoder models only) :contentReference[oaicite:12]{index=12}
            self._send_ext(CAN_PACKET_SET_ORIGIN_HERE, bytes([1]))
            self.get_logger().info("Set PERMANENT origin (dual encoder models only).")
        elif m == "stop":
            # Soft stop: command zero current
            self._send_ext(CAN_PACKET_SET_CURRENT, _to_be_i32(0))
            self.get_logger().info("Soft stop (0 A).")
        else:
            self.get_logger().warn("Unknown special_cmd. Use origin_temp|origin_perm|stop")

    # ---- Control tick ----
    def _tick(self):
        now = time.time()
        with self._lock:
            age = now - self._last_cmd_time
            cmd = list(self._cmd)

        # timeout -> command zero current
        if age > self.cmd_timeout:
            self._send_ext(CAN_PACKET_SET_CURRENT, _to_be_i32(0))
            return

        # reverse polarity (for position/speed/current sign)
        if self.reverse:
            if self.mode in ("pos", "pos_spd"):
                cmd[0] = -cmd[0]
                cmd[1] = -cmd[1]
            elif self.mode == "rpm":
                cmd[1] = -cmd[1]
            elif self.mode in ("current", "brake_current"):
                cmd[2] = -cmd[2]
            # duty usually stays same sign; if you want sign, flip here.

        if self.mode == "pos":
            # int32(pos_deg * 10000) :contentReference[oaicite:13]{index=13}
            pos_i32 = int(round(cmd[0] * 10000.0))
            self._send_ext(CAN_PACKET_SET_POS, _to_be_i32(pos_i32))

        elif self.mode == "pos_spd":
            # payload: pos(int32)*10000, spd(int16), accel(int16)
            # manual shows packing spd/10 and accel/10 into int16 (1 LSB=10 ERPM or 10 ERPM/s^2) :contentReference[oaicite:14]{index=14}
            pos_i32 = int(round(cmd[0] * 10000.0))
            spd_i16 = int(round(cmd[1] / 10.0))
            acc_i16 = int(round(cmd[2] / 10.0))
            payload = _to_be_i32(pos_i32) + _to_be_i16(spd_i16) + _to_be_i16(acc_i16)
            self._send_ext(CAN_PACKET_SET_POS_SPD, payload)

        elif self.mode == "rpm":
            # speed int32 = ERPM :contentReference[oaicite:15]{index=15}
            rpm_i32 = int(round(cmd[1]))
            self._send_ext(CAN_PACKET_SET_RPM, _to_be_i32(rpm_i32))

        elif self.mode == "current":
            # int32(current_A * 1000) :contentReference[oaicite:16]{index=16}
            cur_i32 = int(round(cmd[2] * 1000.0))
            self._send_ext(CAN_PACKET_SET_CURRENT, _to_be_i32(cur_i32))

        elif self.mode == "brake_current":
            # int32(brake_current_A * 1000) :contentReference[oaicite:17]{index=17}
            cur_i32 = int(round(cmd[2] * 1000.0))
            self._send_ext(CAN_PACKET_SET_CURRENT_BRAKE, _to_be_i32(cur_i32))

        elif self.mode == "duty":
            # int32(duty * 100000) (shown in manual snippet) :contentReference[oaicite:18]{index=18}
            duty_i32 = int(round(cmd[0] * 100000.0))
            self._send_ext(CAN_PACKET_SET_DUTY, _to_be_i32(duty_i32))

    # ---- RX loop ----
    def _rx_loop(self):
        while not self._stop:
            rx = self.bus.recv(timeout=0.1)
            if not rx:
                continue
            if not rx.is_extended_id:
                continue
            if len(rx.data) != 8:
                continue
            # Software filter: node id is low 8 bits of ExtID (same placement as TX) :contentReference[oaicite:19]{index=19}
            if (rx.arbitration_id & 0xFF) != self.motor_id:
                continue

            d = rx.data
            # Upload protocol decode :contentReference[oaicite:20]{index=20}
            pos_i = _i16_from_be(d[0], d[1])
            spd_i = _i16_from_be(d[2], d[3])
            cur_i = _i16_from_be(d[4], d[5])
            temp = int(d[6]) if d[6] < 128 else int(d[6]) - 256
            err = int(d[7])

            pos_deg = float(pos_i) * 0.1
            spd_erpm = float(spd_i) * 10.0
            cur_a = float(cur_i) * 0.01

            # reverse polarity on feedback, to match command convention
            if self.reverse:
                pos_deg = -pos_deg
                spd_erpm = -spd_erpm
                cur_a = -cur_a

            # unwrap deg (optional)
            if self._last_pos_deg is None:
                self._pos_abs_deg = pos_deg
            else:
                dp = pos_deg - self._last_pos_deg
                if dp > 0.5 * self._unwrap_span_deg:
                    dp -= self._unwrap_span_deg
                elif dp < -0.5 * self._unwrap_span_deg:
                    dp += self._unwrap_span_deg
                self._pos_abs_deg += dp
            self._last_pos_deg = pos_deg

            # publish JointState + error string
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [self.joint_name]
            js.position = [self._pos_abs_deg * 3.141592653589793 / 180.0]

            # velocity: if pole_pairs provided, ERPM -> mechanical rad/s
            if self.pole_pairs and self.pole_pairs > 0:
                mech_rpm = spd_erpm / float(self.pole_pairs)
                js.velocity = [mech_rpm * (2.0 * 3.141592653589793 / 60.0)]
            else:
                js.velocity = [float("nan")]

            js.effort = [cur_a]
            self.pub_joint.publish(js)

            if err != 0:
                self.pub_err.publish(String(data=f"err={err} tempC={temp}"))

    def destroy_node(self):
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

