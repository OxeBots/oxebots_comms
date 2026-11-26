#!/usr/bin/env python3
"""ROS 2 Python helper node bridging /robot_commands to the ESP32 Base Station."""

import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
import serial.tools.list_ports

from oxebots_interfaces.msg import RobotCmd

BS_MSG_PING = 0x80
BS_MSG_RF_TX = 0x84
BS_MSG_INFO = 0x88
MSG_TYPE_COMMAND = 0
DEFAULT_BAUD = 921600


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b != 0:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = 0xFF
                code_idx = len(out)
                out.append(0)
                code = 1
        else:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
    out[code_idx] = code
    return bytes(out)


def encode_frame(msg_type: int, payload: bytes) -> bytes:
    staged = bytes([msg_type]) + payload
    staged += bytes([crc8(staged)])
    return cobs_encode(staged) + b"\x00"


def encode_robot_command(robot_id: int, vx_mms: int, vy_mms: int, w_crad_s: int, kick: int) -> bytes:
    ts = int(time.time() * 1000) & 0xFFFFFFFF
    hdr = bytes([(robot_id << 4) | MSG_TYPE_COMMAND]) + struct.pack("<I", ts)
    pose = struct.pack("<6h", 0, 0, 0, vx_mms, vy_mms, w_crad_s)
    return hdr + pose + bytes(12) + bytes([kick & 0xFF])


def open_serial_no_reset(port: str, baud: int = DEFAULT_BAUD):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.05,
        write_timeout=0.1,
        rtscts=False,
        dsrdtr=False,
        exclusive=True,
    )
    try:
        import fcntl
        import termios

        fd = ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        bits = struct.unpack("I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
        bits &= ~(termios.TIOCM_DTR | termios.TIOCM_RTS)
        fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", bits))
    except Exception:
        ser.rts = False
        ser.dtr = False
    return ser


def find_base_station(baud: int = DEFAULT_BAUD):
    for port in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
        if not any(tok in port.device for tok in ("ttyUSB", "ttyACM")):
            continue
        ser = None
        try:
            ser = open_serial_no_reset(port.device, baud=baud)
            ser.reset_input_buffer()
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                ser.write(encode_frame(BS_MSG_PING, struct.pack("<I", 1)))
                ser.flush()
                data = ser.read(64)
                if b"\x00" in data and len(data) >= 28:
                    return ser
            ser.close()
        except (serial.SerialException, OSError):
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
    return None


class BaseStationAdapterNode(Node):
    def __init__(self):
        super().__init__("esp32_base_station_python_adapter")
        self._seq = 1
        self._ser = find_base_station()
        if self._ser is None:
            raise RuntimeError("ESP32 Base Station não encontrada nas portas USB.")
        self.get_logger().info(f"Conectado à ESP32 Base Station em {self._ser.port}")
        self._sub = self.create_subscription(
            RobotCmd, "/robot_commands", self._on_cmd, qos_profile_sensor_data
        )

    def _on_cmd(self, msg: RobotCmd):
        for r in msg.robots:
            payload = encode_robot_command(
                int(r.id),
                int(round(r.x_velocity * 1000.0)),
                int(round(r.y_velocity * 1000.0)),
                -int(round(r.angular_velocity * 100.0)),
                max(0, min(255, int(round((r.kick_speed / 3.0) * 255.0)))),
            )
            rf_tx = struct.pack("<HBBB31s", self._seq & 0xFFFF, int(r.id), 0, len(payload), payload.ljust(31, b"\x00"))
            self._seq = (self._seq + 1) & 0xFFFF or 1
            self._ser.write(encode_frame(BS_MSG_RF_TX, rf_tx))

    def destroy_node(self):
        if self._ser is not None:
            self._ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseStationAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
