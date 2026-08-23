import argparse
import subprocess
import sys
import time

try:
    import ssl_robot_protocol_bp
except ImportError:
    print("Generating Python bitproto module")
    try:
        subprocess.run(["bitproto", "py", "proto/ssl_robot_protocol.bitproto", "."], check=True)
        import ssl_robot_protocol_bp
    except Exception as e:
        print(
            f"Failed to generate bitproto python module. Ensure bitproto is installed: pip install bitproto. Error: {e}"
        )
        sys.exit(1)

import serial
import serial.tools.list_ports

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
    """Construct a populated Header sub-message."""
    if timestamp is None:
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF  # uint32
    hdr = ssl_robot_protocol_bp.Header()
    hdr.msg_type = msg_type
    hdr.robot_id = robot_id
    hdr.timestamp = timestamp
    return hdr


class nRF24L01_Controller:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1):
        """
        Initializes the serial connection.
        Note: This __init__ expects a specific port.
        Use the auto_connect() static method to find and connect automatically.
        """
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
        """
        Find the nRF24L01 USB adapter by its VID/PID and return an
        initialized controller instance.
        """
        # These IDs are from dmesg output:
        # idVendor=1a86, idProduct=7523
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
        """Decode GB2312 Chinese text from hex string"""
        try:
            byte_data = bytearray(int(x, 16) for x in hex_string.strip().split())
            return byte_data.decode("gb2312")
        except Exception as e:
            return f"Decoding error: {e}"

    @staticmethod
    def translate_chinese(text):
        """Translate common Chinese terms to English"""
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
        """Send AT command and read response"""
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
        """Get and display system information"""
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
        """Set receive pipe address using AT+RXA command"""
        if len(address_bytes) != 5:
            return False
        addr_str = ",".join(f"0x{b:02X}" for b in address_bytes)
        return "successful" in self.send_at_command(f"AT+RXA={addr_str}", wait_time=3).lower()

    def set_transmit_address(self, address_bytes):
        """Set transmit pipe address using AT+TXA command"""
        if len(address_bytes) != 5:
            return False
        addr_str = ",".join(f"0x{b:02X}" for b in address_bytes)
        return "successful" in self.send_at_command(f"AT+TXA={addr_str}", wait_time=3).lower()

    def set_addresses(self, rx_address, tx_address):
        """Set both receive and transmit addresses"""
        print(
            f"\n{Colors.BLUE}Setting RX: {[hex(x) for x in rx_address]}  TX: {[hex(x) for x in tx_address]}{Colors.RESET}"
        )
        return self.set_receive_address(rx_address) and self.set_transmit_address(tx_address)

    def set_frequency(self, frequency_ghz):
        """Set operating frequency in GHz (e.g., 2.404 for 2.404GHz)"""
        if not 2.400 <= frequency_ghz <= 2.525:
            return False
        return "successful" in self.send_at_command(f"AT+FREQ={frequency_ghz:.3f}").lower()

    def set_data_rate(self, rate):
        """Set data rate (1=250Kbps, 2=1Mbps, 3=2Mbps)"""
        if rate not in [1, 2, 3]:
            return False
        rates = {1: "250Kbps", 2: "1Mbps", 3: "2Mbps"}
        print(f"\n{Colors.BLUE}Setting data rate to {rates[rate]}{Colors.RESET}")
        return "successful" in self.send_at_command(f"AT+RATE={rate}").lower()

    def set_baudrate(self, new_baud):
        """Change baud rate (1-7 for different rates)"""
        baud_rates = {
            1: 4800,
            2: 9600,
            3: 14400,
            4: 19200,
            5: 38400,
            6: 57600,
            7: 115200,
        }
        if new_baud not in baud_rates:
            print(f"{Colors.RED}Error: Invalid baud rate code: {new_baud}{Colors.RESET}")
            return False
        print(f"\n{Colors.BLUE}Setting baud rate to {baud_rates[new_baud]}{Colors.RESET}")
        if "successful" in self.send_at_command(f"AT+BAUD={new_baud}").lower():
            print(f"{Colors.GREEN}✓ Baud rate changed to {baud_rates[new_baud]}{Colors.RESET}")
            return True
        print(f"{Colors.RED}✗ Baud rate change failed{Colors.RESET}")
        return False

    # Protocol-specific commands for robot control

    def send_data(self, raw_bytes):
        """
        Send raw bytes to the USB dongle.
        The dongle counts UART bytes, prepends the length in the RF packet,
        and fires it over the air — do NOT zero-pad or add a length prefix here.
        """
        self.ser.reset_input_buffer()
        self.ser.write(raw_bytes)
        self.ser.flush()

    def send_command(self, robot_id, x=0, y=0, angle=0, vx=0, vy=0, w=0, kick=0, timestamp=None):
        """
        Build and send a RobotCommand.

        :param robot_id:  Target robot ID or ROBOT_ID_BROADCAST.
        :param x:         Target pose X (mm).
        :param y:         Target pose Y (mm).
        :param angle:     Target orientation (0.01° units).
        :param vx:        Target X velocity (mm/s).
        :param vy:        Target Y velocity (mm/s).
        :param w:         Target angular velocity (0.01 rad/s).
        :param kick:      Kick strength (0-255).
        :param timestamp: uint32 ms timestamp; auto-generated if None.
        :returns:         The timestamp used (for RTT matching).
        """
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

    def send_config(self, robot_id, config_flags, param=0):
        """
        Send a RobotConfig message.

        :param robot_id:      Target robot ID or ROBOT_ID_BROADCAST.
        :param config_flags:  Bitmask of CONFIG_FLAG_* constants.
        :param param:         Auxiliary value (e.g. new ID for CONFIG_FLAG_SET_ID).

        Examples:
            device.send_config(0, CONFIG_FLAG_SET_ID, param=3)   # assign ID 3 to robot 0
            device.send_config(ROBOT_ID_BROADCAST, CONFIG_FLAG_RUN_CALIBRATION)
        """
        cfg = ssl_robot_protocol_bp.RobotConfig()
        cfg.header = _make_header(MSG_TYPE_CONFIG, robot_id)
        cfg.config_flags = config_flags
        cfg.param = param

        print(f"{Colors.CYAN}[CONFIG] robot_id={robot_id} flags=0x{config_flags:02X} param={param}{Colors.RESET}")
        self.send_data(cfg.encode())

    def receive_data(self, timeout=0.5):
        """
        Receive a RobotTelemetry packet.

        Strips the dongle's 6-byte TX-complete status code (02 00 00 00 00 00),
        peeks header.msg_type to discard non-telemetry frames, then decodes.

        Returns a decoded RobotTelemetry on success, or None on timeout/error.
        """
        start_time = time.time()
        buffer = bytearray()
        expected_len = ssl_robot_protocol_bp.RobotTelemetry.BYTES_LENGTH

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                buffer.extend(self.ser.read(self.ser.in_waiting))

                # Strip dongle TX-complete status codes
                while len(buffer) >= 6 and buffer.startswith(b"\x02\x00\x00\x00\x00\x00"):
                    buffer = buffer[6:]

                if len(buffer) >= expected_len:
                    packet = bytearray(buffer[:expected_len])

                    raw_msg_type = packet[0] & 0x0F
                    if raw_msg_type != MSG_TYPE_TELEMETRY:
                        buffer = buffer[expected_len:]
                        continue

                    try:
                        telemetry = ssl_robot_protocol_bp.RobotTelemetry()
                        telemetry.decode(packet)
                        print(
                            f"\r{Colors.GREEN}✓ [RX] Robot {telemetry.header.robot_id} | "
                            f"ts={telemetry.header.timestamp} | "
                            f"Batt={telemetry.battery_percentage}% | "
                            f"Kicker={telemetry.kicker_voltage / 100:.2f}V{Colors.RESET}" + " " * 10
                        )
                        return telemetry
                    except Exception as e:
                        print(f"\n{Colors.RED}[RX] Decode error: {e}{Colors.RESET}")

                    buffer = buffer[expected_len:]
            else:
                time.sleep(0.001)

        return None

    def close(self):
        """Close serial connection"""
        if self.ser.is_open:
            self.ser.close()
            print(f"{Colors.BLUE}Serial connection closed{Colors.RESET}")


class CLIController:
    """A blocking, synchronous CLI command loop for testing the Bitproto protocol"""

    def __init__(self, device):
        self.device = device
        self.running = True

    def _print_help(self):
        print(f"""
{Colors.BOLD}Commands:{Colors.RESET}
  {{x:<mm>, y:<mm>, vx:<mm/s>, vy:<mm/s>, w:<0.01rad/s>, kick:<0-255>, id:<robot_id>}} — send RobotCommand
  set_id <target_id> <new_id>                            — assign new robot ID
  calibrate [<robot_id>|broadcast]                       — wheel + IMU calibration + PID tuning
  calibrate_mag [<robot_id>|broadcast]                   — mag calibration
  factory_reset [<robot_id>|broadcast]                   — full NVS erase + reboot
  help                                                   — show this help
  q / quit                                               — exit
""")

    def _parse_robot_id(self, token):
        return ROBOT_ID_BROADCAST if token.lower() == "broadcast" else int(token)

    def start(self):
        print(f"\n{Colors.BOLD}=== ROBOT CLI CONTROL ==={Colors.RESET}")
        self._print_help()

        while self.running:
            try:
                raw = input(f"\n{Colors.BLUE}Cmd > {Colors.RESET}").strip()
                if not raw:
                    continue
                if raw.lower() in ["q", "quit", "exit"]:
                    break
                if raw.lower() == "help":
                    self._print_help()
                    continue

                # Config shortcuts
                if raw.startswith("set_id"):
                    parts = raw.split()
                    if len(parts) < 3:
                        print("Usage: set_id <target_id> <new_id>")
                        continue
                    self.device.send_config(
                        self._parse_robot_id(parts[1]),
                        CONFIG_FLAG_SET_ID,
                        param=int(parts[2]),
                    )
                    continue

                if raw.startswith("calibrate_mag"):
                    parts = raw.split()
                    rid = self._parse_robot_id(parts[1]) if len(parts) > 1 else 0
                    self.device.send_config(rid, CONFIG_FLAG_CALIBRATE_MAG)
                    continue

                if raw.startswith("calibrate"):
                    parts = raw.split()
                    rid = self._parse_robot_id(parts[1]) if len(parts) > 1 else 0
                    self.device.send_config(rid, CONFIG_FLAG_RUN_CALIBRATION)
                    continue

                if raw.startswith("factory_reset"):
                    parts = raw.split()
                    rid = self._parse_robot_id(parts[1]) if len(parts) > 1 else 0
                    self.device.send_config(rid, CONFIG_FLAG_FACTORY_RESET)
                    continue

                # Motion command
                x, y, vx, vy, w, kick, robot_id = 0, 0, 0, 0, 0, 0, 0
                for p in raw.strip("{} ").split(","):
                    if ":" in p:
                        k, v = p.split(":", 1)
                        k, v = k.strip(" \"'"), v.strip()
                        if k == "x":
                            x = int(v)
                        elif k == "y":
                            y = int(v)
                        elif k == "vx":
                            vx = int(v)
                        elif k == "vy":
                            vy = int(v)
                        elif k == "w":
                            w = int(v)
                        elif k == "kick":
                            kick = int(v)
                        elif k == "id":
                            robot_id = self._parse_robot_id(v)

                timestamp = self.device.send_command(robot_id, x=x, y=y, vx=vx, vy=vy, w=w, kick=kick)
                id_label = "BROADCAST" if robot_id == ROBOT_ID_BROADCAST else str(robot_id)
                print(
                    f"{Colors.CYAN}→ [CMD] robot={id_label} ts={timestamp} x={x} y={y} vx={vx} vy={vy} w={w} kick={kick}{Colors.RESET}",
                    end="",
                    flush=True,
                )

                if robot_id != ROBOT_ID_BROADCAST:
                    # Await a response from the robot firmware via NRF24 USB dongle, on average it takes 20ms. We will wait up to 200ms to be safe.

                    self.device.receive_data(timeout=0.200)
                else:
                    print(f"  {Colors.YELLOW}(broadcast — no reply expected){Colors.RESET}")

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"{Colors.RED}Error: {e}{Colors.RESET}")


def main():
    parser = argparse.ArgumentParser(
        description="nRF24L01 Wireless Module Controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", type=str, default="auto", help="Serial port device.")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "-f",
        "--freq",
        type=float,
        default=2.401,
        help="Communication frequency in GHz.",
    )
    parser.add_argument(
        "-r",
        "--rate",
        type=int,
        default=3,
        choices=[1, 2, 3],
        help="Air data rate: 1=250Kbps, 2=1Mbps, 3=2Mbps",
    )
    parser.add_argument("-a", "--address", type=str, default="ESP32", help="5-byte address string.")
    parser.add_argument(
        "--skip-config",
        action="store_true",
        help="Skip sending AT config commands to the USB adapter.",
    )
    args = parser.parse_args()

    device = None
    try:
        print(f"{Colors.BOLD}=== nRF24L01 Wireless Module Controller ==={Colors.RESET}\n")

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

        CLIController(device).start()

    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Stopped by user{Colors.RESET}")
    except serial.SerialException as e:
        print(f"\n{Colors.RED}Serial Error: {e}{Colors.RESET}")
        print(f"{Colors.YELLOW}Hint: Try 'sudo usermod -a -G dialout $USER'{Colors.RESET}")
    except Exception as e:
        print(f"{Colors.RED}An unexpected error occurred: {e}{Colors.RESET}")
    finally:
        if device and device.ser.is_open:
            device.close()


if __name__ == "__main__":
    main()
