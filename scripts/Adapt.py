#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import subprocess
import sys
import time
import serial
import serial.tools.list_ports
from oxebots_interfaces.msg import RobotCmd
import os

try:
    import ssl_robot_protocol_bp
except ImportError:
    print("Generating Python bitproto module...")
    from ament_index_python.packages import get_package_share_directory
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        interfaces_share = get_package_share_directory("oxebots_interfaces")
        proto_path = os.path.join(interfaces_share, "proto", "ssl_robot_protocol.bitproto")

        subprocess.run(["bitproto", "py", proto_path, script_dir], check=True)
        import ssl_robot_protocol_bp
    except Exception as e:
        print(f"Failed to generate bitproto python module. Error: {e}")
        sys.exit(1)

MSG_TYPE_COMMAND = ssl_robot_protocol_bp.MSG_TYPE_COMMAND
MSG_TYPE_CONFIG = ssl_robot_protocol_bp.MSG_TYPE_CONFIG
MSG_TYPE_TELEMETRY = ssl_robot_protocol_bp.MSG_TYPE_TELEMETRY
ROBOT_ID_BROADCAST = ssl_robot_protocol_bp.ROBOT_ID_BROADCAST
ROBOT_ID_MAX = ssl_robot_protocol_bp.ROBOT_ID_MAX

CONFIG_FLAG_SET_ID = ssl_robot_protocol_bp.CONFIG_FLAG_SET_ID
CONFIG_FLAG_RUN_CALIBRATION = ssl_robot_protocol_bp.CONFIG_FLAG_RUN_CALIBRATION
CONFIG_FLAG_CALIBRATE_MAG = ssl_robot_protocol_bp.CONFIG_FLAG_CALIBRATE_MAG
CONFIG_FLAG_FACTORY_RESET = ssl_robot_protocol_bp.CONFIG_FLAG_FACTORY_RESET

class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"

def _make_header(msg_type, robot_id, timestamp=None):
    if timestamp is None:
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF  # uint32
    hdr = ssl_robot_protocol_bp.Header()
    hdr.msg_type = msg_type
    hdr.robot_id = robot_id
    hdr.timestamp = timestamp
    return hdr

class nRF24L01_Controller:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        if self.ser.is_open:
            print(f"{Colors.GREEN}Connected to {port} at {baudrate} baud{Colors.RESET}")

    @staticmethod
    def auto_connect(baudrate=115200, timeout=2):
        TARGET_VID, TARGET_PID = 0x1A86, 0x7523
        print(f"{Colors.BLUE}Searching for nRF24L01 device (VID:{TARGET_VID:x}, PID:{TARGET_PID:x})...{Colors.RESET}")
        for port in serial.tools.list_ports.comports():
            if port.vid == TARGET_VID and port.pid == TARGET_PID:
                print(f"{Colors.GREEN}Found device at: {port.device}{Colors.RESET}")
                try:
                    return nRF24L01_Controller(port=port.device, baudrate=baudrate, timeout=timeout)
                except serial.SerialException as e:
                    print(f"{Colors.RED}Error connecting to {port.device}: {e}{Colors.RESET}")
                    return None
        print(f"\n{Colors.RED}Error: Could not find nRF24L01 USB adapter.{Colors.RESET}")
        return None

    @staticmethod
    def decode_gb2312(hex_string):
        try:
            byte_data = bytearray(int(x, 16) for x in hex_string.strip().split())
            return byte_data.decode("gb2312")
        except Exception as e:
            return f"Decoding error: {e}"

    @staticmethod
    def translate_chinese(text):
        translations = {
            "系统信息": "System Information",
            "波特率": "Baud Rate",
            "目标地址": "Target Address",
            "本地接收地址": "Local Receive Address",
            "通讯频率": "Communication Frequency",
            "校验模式": "Check Mode",
            "发射功率": "Transmit Power",
            "空中传输速率": "Air Data Rate",
            "低噪声放大增益": "Low Noise Amplifier Gain",
            "开启": "Enabled",
            "关闭": "Disabled",
            "传输速率设置成功": "Data rate setting successful",
            "通讯波特率设置成功": "Baud rate setting successful",
            "地址设置成功": "Address setting successful",
            "设置成功": "Setting successful",
            "成功": "successful",
            "CRC校验": "CRC Check",
            "位": " bits ",
            "传输速率": "Data Rate",
        }
        for cn, en in translations.items():
            text = text.replace(cn, en)
        return text

    def send_at_command(self, command, wait_time=0.1):
        print(f"{Colors.BLUE}>>> {command}{Colors.RESET}")
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()
        time.sleep(wait_time)

        response = b""
        start_time = time.time()
        while time.time() - start_time < 5:
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
                time.sleep(0.1)
            elif response and time.time() - start_time > 0.5:
                break

        if response:
            hex_str = " ".join(f"{b:02x}" for b in response)
            decoded = self.decode_gb2312(hex_str)
            if decoded.startswith("Decoding error"):
                print(f"{Colors.RED}<<< {decoded}{Colors.RESET}")
                return decoded
            translated = self.translate_chinese(decoded)
            print(f"{Colors.GREEN}<<< {translated}{Colors.RESET}")
            return translated
        print(f"{Colors.YELLOW}<<< No response{Colors.RESET}")
        return ""

    def get_system_info(self):
        print("\n" + "=" * 50)
        print(f"{Colors.BOLD}SYSTEM INFORMATION{Colors.RESET}")
        print("=" * 50)
        response = self.send_at_command("AT?")
        if not response:
            return
        key_map = {
            "Baud Rate": "Baud Rate",
            "Target Address": "Target Address",
            "Local Receive Address": "Local Receive Address",
            "Communication Frequency": "Communication Frequency",
            "Check Mode": "Check Mode",
            "Transmit Power": "Transmit Power",
            "Air Data Rate": "Air Data Rate",
            "Low Noise Amplifier Gain": "Low Noise Amplifier Gain",
        }
        for line in response.split("\r\n"):
            line = line.strip()
            if not line or line in ["OK", "System Information"]:
                continue
            if ":" in line:
                key, value = line.split(":", 1)
                display_key = next((lbl for sub, lbl in key_map.items() if sub in key), key.strip())
                print(f"{display_key:25}: {value.strip()}")

    def set_receive_address(self, address_bytes):
        if len(address_bytes) != 5:
            return False
        addr_str = ",".join(f"0x{b:02X}" for b in address_bytes)
        return "successful" in self.send_at_command(f"AT+RXA={addr_str}", wait_time=3).lower()

    def set_transmit_address(self, address_bytes):
        if len(address_bytes) != 5:
            return False
        addr_str = ",".join(f"0x{b:02X}" for b in address_bytes)
        return "successful" in self.send_at_command(f"AT+TXA={addr_str}", wait_time=3).lower()

    def set_addresses(self, rx_address, tx_address):
        print(f"\n{Colors.BLUE}Setting RX: {[hex(x) for x in rx_address]}  TX: {[hex(x) for x in tx_address]}{Colors.RESET}")
        return self.set_receive_address(rx_address) and self.set_transmit_address(tx_address)

    def set_frequency(self, frequency_ghz):
        if not 2.400 <= frequency_ghz <= 2.525:
            return False
        return "successful" in self.send_at_command(f"AT+FREQ={frequency_ghz:.3f}").lower()

    def set_data_rate(self, rate):
        if rate not in [1, 2, 3]:
            return False
        rates = {1: "250Kbps", 2: "1Mbps", 3: "2Mbps"}
        print(f"\n{Colors.BLUE}Setting data rate to {rates[rate]}{Colors.RESET}")
        return "successful" in self.send_at_command(f"AT+RATE={rate}").lower()

    def send_data(self, raw_bytes):
        self.ser.reset_input_buffer()
        self.ser.write(raw_bytes)
        self.ser.flush()

    def send_command(self, robot_id, x=0, y=0, angle=0, vx=0, vy=0, w=0, kick=0, timestamp=None):
        if timestamp is None:
            timestamp = int(time.time() * 1000) & 0xFFFFFFFF

        cmd = ssl_robot_protocol_bp.RobotCommand()
        cmd.header = _make_header(MSG_TYPE_COMMAND, robot_id, timestamp)
        cmd.target_pose.x = x
        cmd.target_pose.y = y
        cmd.target_pose.angle = angle
        cmd.target_pose.x_v = vx
        cmd.target_pose.y_v = vy
        cmd.target_pose.angular_vel = w
        cmd.kick_velocity = kick

        encoded = cmd.encode()
        self.send_data(encoded)
        return timestamp

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            print(f"{Colors.BLUE}Serial connection closed{Colors.RESET}")

# gambiarra do ros
class Nrf24HardwareBridge(Node):
    def __init__(self, device):
        super().__init__('nrf24_hardware_bridge')
        self.device = device
        self.subscription = self.create_subscription(
            RobotCmd,
            'robot_commands',
            self.robot_cmd_callback,
            10
        )
        self.get_logger().info(f"{Colors.BOLD}ROS 2 -> NRF24L01 Bridge Started! Listening to /robot_commands{Colors.RESET}")

    def robot_cmd_callback(self, msg):
        for robot in msg.robots:
            vx_mm = int(robot.x_velocity * 1000)
            vy_mm = int(robot.y_velocity * 1000)
            w_scaled = int(robot.angular_velocity * 100)
            kick = int(robot.kick_speed)

            self.device.send_command(
                robot_id=0,
                vx=vx_mm,
                vy=vy_mm,
                w=w_scaled,
                kick=kick
            )
            self.get_logger().debug(f"Robot {robot.id}: vx={vx_mm} vy={vy_mm} w={w_scaled} kick={kick}")

def main():
    parser = argparse.ArgumentParser(
        description="nRF24L01 Wireless Module Controller Bridge",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", type=str, default="auto", help="Serial port device.")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("-f", "--freq", type=float, default=2.401, help="Communication frequency in GHz.")
    parser.add_argument("-r", "--rate", type=int, default=3, choices=[1, 2, 3], help="Air data rate: 1=250Kbps, 2=1Mbps, 3=2Mbps")
    parser.add_argument("-a", "--address", type=str, default="ESP32", help="5-byte address string.")
    parser.add_argument("--skip-config", action="store_true", help="Skip sending AT config commands to the USB adapter.")

    args, unknown = parser.parse_known_args()

    print(f"{Colors.BOLD} Conectando ao Hardware {Colors.RESET}")
    device = (
        nRF24L01_Controller.auto_connect(baudrate=args.baud)
        if args.port.lower() == "auto"
        else nRF24L01_Controller(port=args.port, baudrate=args.baud)
    )

    if not device:
        return

    if not args.skip_config:
        print(f"\n{Colors.BOLD}CONFIGURING MODULE:{Colors.RESET}")
        if len(args.address) != 5:
            print(f"{Colors.RED}Error: Address must be exactly 5 characters.{Colors.RESET}")
            return
        rx_address_bytes = [ord(c) for c in "ADMIN"]
        tx_address_bytes = list(args.address.encode("ascii"))
        device.set_addresses(rx_address_bytes, tx_address_bytes)
        device.set_frequency(2.400 + 76 * 0.001)
        device.set_data_rate(args.rate)
        device.get_system_info()
    else:
        print(f"\n{Colors.YELLOW}Skipping USB Adapter AT-Configuration...{Colors.RESET}")

    rclpy.init()
    bridge_node = Nrf24HardwareBridge(device)

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Stopped by user{Colors.RESET}")
    except Exception as e:
        print(f"{Colors.RED}An unexpected error occurred: {e}{Colors.RESET}")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()
        if device.ser.is_open:
            device.close()

if __name__ == '__main__':
    main()