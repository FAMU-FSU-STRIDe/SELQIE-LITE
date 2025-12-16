import math
import struct
from typing import Dict, Optional

import can
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8MultiArray


def parse_power_dist_payload(data: bytes) -> Dict[str, Optional[float]]:
    """Parse a power distribution telemetry or register-reply frame.

    The function first checks for the register-reply format used by the upstream
    ``mjbots/power_dist`` example (as seen in the Pi3Hat demo script). When the
    headers match, it unpacks the extended values (including energy and switch
    state). Otherwise it falls back to the 16-byte periodic telemetry layout.

    Frames shorter than the expected layout return any available fields and
    leave the remainder as ``None``. Values are scaled to volts/amps/degrees
    Celsius.
    """
    fields: Dict[str, Optional[float]] = {
        'bus_voltage_v': None,
        'bus_current_a': None,
        'board_temp_c': None,
        'rail5_voltage_v': None,
        'rail5_current_a': None,
        'rail12_voltage_v': None,
        'rail12_current_a': None,
        'status_flags': None,
        'energy_j': None,
        'switch_closed': None,
    }

    register_fmt = struct.Struct('<bbhhhbbibbb')
    if len(data) >= register_fmt.size:
        try:
            unpacked = register_fmt.unpack(data[0 : register_fmt.size])
        except struct.error:
            unpacked = None
        if unpacked and unpacked[0] == 0x27 and unpacked[1] == 0x10:
            if unpacked[5] == 0x29 and unpacked[6] == 0x13 and unpacked[8] == 0x21 and unpacked[9] == 0x02:
                fields['bus_voltage_v'] = unpacked[2] * 0.1
                fields['bus_current_a'] = unpacked[3] * 0.1
                fields['board_temp_c'] = unpacked[4] * 0.1
                fields['energy_j'] = unpacked[7] * 0.000001
                fields['switch_closed'] = unpacked[10] != 0
                return fields

    if len(data) >= 2:
        fields['bus_voltage_v'] = int.from_bytes(data[0:2], byteorder='little', signed=False) / 1000.0
    if len(data) >= 4:
        fields['bus_current_a'] = int.from_bytes(data[2:4], byteorder='little', signed=True) / 1000.0
    if len(data) >= 6:
        fields['board_temp_c'] = int.from_bytes(data[4:6], byteorder='little', signed=True) / 100.0
    if len(data) >= 8:
        fields['rail5_voltage_v'] = int.from_bytes(data[6:8], byteorder='little', signed=False) / 1000.0
    if len(data) >= 10:
        fields['rail5_current_a'] = int.from_bytes(data[8:10], byteorder='little', signed=True) / 1000.0
    if len(data) >= 12:
        fields['rail12_voltage_v'] = int.from_bytes(data[10:12], byteorder='little', signed=False) / 1000.0
    if len(data) >= 14:
        fields['rail12_current_a'] = int.from_bytes(data[12:14], byteorder='little', signed=True) / 1000.0
    if len(data) >= 16:
        fields['status_flags'] = int.from_bytes(data[14:16], byteorder='little', signed=False)

    return fields


class PowerDistNode(Node):
    def __init__(self) -> None:
        super().__init__('mjbots_power_dist')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('status_id', 0x2000)
        self.declare_parameter('status_id_mask', 0x1FFFFFFF)
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('frame_length_warning', 16)
        self.declare_parameter('register_query_id', 0x8020)
        self.declare_parameter('register_reply_id', 0x2000)
        self.declare_parameter('register_poll_hz', 10.0)

        self._can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self._status_id = int(self.get_parameter('status_id').value)
        self._status_id_mask = int(self.get_parameter('status_id_mask').value)
        poll_hz = float(self.get_parameter('poll_hz').value)
        self._frame_length_warning = int(self.get_parameter('frame_length_warning').value)
        self._register_query_id = int(self.get_parameter('register_query_id').value)
        self._register_reply_id = int(self.get_parameter('register_reply_id').value)
        register_poll_hz = float(self.get_parameter('register_poll_hz').value)

        self.get_logger().info(
            f'Opening SocketCAN interface "{self._can_interface}" for power dist telemetry'
        )
        if self._status_id:
            self.get_logger().info(
                f'Expecting status frames matching ID 0x{self._status_id:X} (mask 0x{self._status_id_mask:X})'
            )

        self._bus = can.interface.Bus(channel=self._can_interface, bustype='socketcan')
        self._reader = can.BufferedReader()
        self._notifier = can.Notifier(self._bus, [self._reader], timeout=1.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._battery_pub = self.create_publisher(BatteryState, 'power_dist/battery', qos)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, 'power_dist/diagnostics', qos)
        self._raw_pub = self.create_publisher(UInt8MultiArray, 'power_dist/raw_frame', qos)

        poll_period = 1.0 / poll_hz if poll_hz > 0.0 else 0.1
        self._timer = self.create_timer(poll_period, self._poll_bus)
        if self._register_query_id and register_poll_hz > 0.0:
            self._register_timer = self.create_timer(
                1.0 / register_poll_hz, self._send_register_query
            )

    def destroy_node(self) -> bool:
        if hasattr(self, '_notifier'):
            self._notifier.stop()
        if hasattr(self, '_bus'):
            self._bus.shutdown()
        return super().destroy_node()

    def _poll_bus(self) -> None:
        while True:
            frame = self._reader.get_message(0.0)
            if frame is None:
                break
            if self._status_id:
                if (frame.arbitration_id & self._status_id_mask) != (
                    self._status_id & self._status_id_mask
                ):
                    continue
            elif self._register_reply_id:
                if (frame.arbitration_id & self._status_id_mask) != (
                    self._register_reply_id & self._status_id_mask
                ):
                    continue
            elif len(frame.data) < 8:
                # Skip frames that cannot match the upstream telemetry format when auto-detecting
                continue
            self._handle_frame(frame)

    def _send_register_query(self) -> None:
        if not hasattr(self, '_bus'):
            return
        try:
            msg = can.Message(
                arbitration_id=self._register_query_id,
                is_extended_id=self._register_query_id > 0x7FF,
                is_fd=True,
                data=bytes(
                    [
                        0x17,
                        0x10,
                        0x19,
                        0x13,
                        0x11,
                        0x02,
                    ]
                ),
            )
            self._bus.send(msg, timeout=0.01)
        except can.CanError as exc:
            self.get_logger().warn(f'Failed to send register query: {exc}')

    def _handle_frame(self, frame: can.Message) -> None:
        data = bytes(frame.data)
        if self._status_id == 0:
            # Auto-detect the first plausible telemetry frame so the node works with
            # the default MJBots ID (0x500) as well as custom firmware IDs from the
            # upstream repository.
            self._status_id = frame.arbitration_id
            self.get_logger().info(
                f'Auto-detected power dist status ID: 0x{self._status_id:X} (extended={frame.is_extended_id})'
            )

        if len(data) < self._frame_length_warning:
            self.get_logger().warn(
                f'Received short power dist frame (len={len(data)}); expected at least {self._frame_length_warning} bytes'
            )

        fields = parse_power_dist_payload(data)
        now = self.get_clock().now().to_msg()

        battery = BatteryState()
        battery.header.stamp = now
        battery.voltage = fields['bus_voltage_v'] if fields['bus_voltage_v'] is not None else math.nan
        battery.current = fields['bus_current_a'] if fields['bus_current_a'] is not None else math.nan
        battery.temperature = fields['board_temp_c'] if fields['board_temp_c'] is not None else math.nan
        battery.present = True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self._battery_pub.publish(battery)

        diag_status = DiagnosticStatus()
        diag_status.name = 'mjbots_power_dist'
        diag_status.level = DiagnosticStatus.OK
        diag_status.message = 'OK'
        if fields['status_flags']:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = 'Flags set'
        elif fields['switch_closed'] is False:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = 'Switch open'
        diag_status.hardware_id = self._can_interface

        def add_kv(key: str, value: Optional[float], suffix: str = '') -> None:
            msg = 'nan' if value is None or math.isnan(value) else f'{value:.3f}{suffix}'
            diag_status.values.append(KeyValue(key=key, value=msg))

        add_kv('bus_voltage_v', fields['bus_voltage_v'])
        add_kv('bus_current_a', fields['bus_current_a'])
        add_kv('board_temp_c', fields['board_temp_c'])
        add_kv('rail5_voltage_v', fields['rail5_voltage_v'])
        add_kv('rail5_current_a', fields['rail5_current_a'])
        add_kv('rail12_voltage_v', fields['rail12_voltage_v'])
        add_kv('rail12_current_a', fields['rail12_current_a'])
        if fields['status_flags'] is not None:
            diag_status.values.append(
                KeyValue(key='status_flags', value=f"0x{int(fields['status_flags']):04X}")
            )
        if fields['energy_j'] is not None:
            diag_status.values.append(KeyValue(key='energy_j', value=f'{fields["energy_j"]:.6f}'))
        if fields['switch_closed'] is not None:
            diag_status.values.append(
                KeyValue(key='switch_closed', value=str(bool(fields['switch_closed'])))
            )

        diag_array = DiagnosticArray()
        diag_array.header.stamp = now
        diag_array.status.append(diag_status)
        self._diagnostic_pub.publish(diag_array)

        raw_msg = UInt8MultiArray()
        raw_msg.data = list(data)
        self._raw_pub.publish(raw_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerDistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
