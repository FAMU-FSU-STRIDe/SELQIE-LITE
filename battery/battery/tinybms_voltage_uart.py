#!/usr/bin/env python3
import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial


def crc16_modbus(data: bytes) -> int:
    """
    MODBUS CRC16 (init 0xFFFF). Matches TinyBMS spec (poly 0x8005). :contentReference[oaicite:5]{index=5}
    Returns 16-bit int; transmit as LSB then MSB.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class TinyBmsUart(Node):
    """
    Publishes TinyBMS pack voltage read via UART command 0x14. :contentReference[oaicite:6]{index=6}
    Topic: /tinybms/pack_voltage (std_msgs/Float32)
    """

    def __init__(self):
        super().__init__("tinybms_voltage_uart")

        # Parameters
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 115200)  # fixed by spec :contentReference[oaicite:7]{index=7}
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("timeout_s", 0.25)
        self.declare_parameter("wakeup_send_twice", True)  # sleep behavior note :contentReference[oaicite:8]{index=8}

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        timeout_s = float(self.get_parameter("timeout_s").get_parameter_value().double_value)

        # Open serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )

        self.pub = self.create_publisher(Float32, "/tinybms/pack_voltage", 10)

        rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)
        period = 1.0 / max(rate_hz, 0.1)
        self.timer = self.create_timer(period, self.poll_once)

        self.get_logger().info(f"TinyBMS UART voltage node on {port} @ {baud} baud")

    def build_voltage_request(self) -> bytes:
        # Request: 0xAA 0x14 CRC_LSB CRC_MSB :contentReference[oaicite:9]{index=9}
        payload = bytes([0xAA, 0x14])
        crc = crc16_modbus(payload)
        return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def read_voltage_response(self) -> Optional[float]:
        """
        Expected OK response for 0x14:
          0xAA 0x14 <4 bytes float LSB..MSB> CRC_LSB CRC_MSB  (total 8 bytes) :contentReference[oaicite:10]{index=10}
        Expected ERROR response:
          0xAA 0x00 0x14 ERROR CRC_LSB CRC_MSB (6 bytes) :contentReference[oaicite:11]{index=11}
        """
        # Read first 2 bytes to classify
        hdr = self.ser.read(2)
        if len(hdr) < 2:
            return None

        if hdr[0] != 0xAA:
            # resync: flush whatever is there
            self.ser.reset_input_buffer()
            return None

        if hdr[1] == 0x14:
            rest = self.ser.read(6)  # 4 data + 2 crc
            if len(rest) != 6:
                return None
            frame = hdr + rest
            data4 = frame[2:6]
            rx_crc = frame[6] | (frame[7] << 8)
            calc_crc = crc16_modbus(frame[:6])
            if rx_crc != calc_crc:
                self.get_logger().warn(f"CRC mismatch: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
                return None

            # DATA is LSB..MSB => little-endian float :contentReference[oaicite:12]{index=12}
            volts = struct.unpack("<f", data4)[0]
            return volts

        if hdr[1] == 0x00:
            # error packet: need CMD (expect 0x14), ERROR, CRC16
            rest = self.ser.read(4)
            if len(rest) != 4:
                return None
            frame = hdr + rest
            cmd = frame[2]
            err = frame[3]
            rx_crc = frame[4] | (frame[5] << 8)
            calc_crc = crc16_modbus(frame[:4])
            if rx_crc != calc_crc:
                self.get_logger().warn(f"CRC mismatch on ERROR frame: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
                return None
            if cmd == 0x14:
                self.get_logger().warn(f"TinyBMS returned ERROR for voltage cmd 0x14: code=0x{err:02X}")
            return None

        # Unknown response type
        return None

    def poll_once(self):
        try:
            req = self.build_voltage_request()

            # Sleep-mode note: first command might just wake it; send twice if enabled :contentReference[oaicite:13]{index=13}
            send_twice = self.get_parameter("wakeup_send_twice").get_parameter_value().bool_value

            self.ser.reset_input_buffer()
            self.ser.write(req)
            self.ser.flush()

            v = self.read_voltage_response()
            if v is None and send_twice:
                # send again and try once more
                self.ser.reset_input_buffer()
                self.ser.write(req)
                self.ser.flush()
                v = self.read_voltage_response()

            if v is None:
                return

            msg = Float32()
            msg.data = float(v)
            self.pub.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unhandled error: {e}")


def main():
    rclpy.init()
    node = TinyBmsUart()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

