import socket
import json
import time
import serial
import struct
import threading

# --- Mock Server and Ports ---

class MockMCPServer(threading.Thread):
    """A mock TCP server that emulates the ESP32's MCP server."""
    def __init__(self, host='localhost', port=8888):
        super().__init__()
        self.host = host
        self.port = port
        self.server_socket = None
        self._stop_event = threading.Event()
        self.client_threads = []
        self.last_action_vector = None
        self.tool_list = [
            {"name": "set_torque"}, {"name": "set_acceleration"},
            {"name": "get_status"}, {"name": "get_current"},
            {"name": "get_power"}, {"name": "get_torque"},
            {"name": "export_data"}, {"name": "export_nn"},
            {"name": "import_nn"}, {"name": "import_nn_json"},
            {"name": "calibrate_servo"}, {"name": "set_pos"}, {"name": "get_pos"},
            {"name": "move_towards_goal_embedding"}, {"name": "set_goal_embedding"},
            {"name": "execute_behavior"}
        ]

    def handle_client(self, conn, addr):
        print(f"  [MOCK_MCP] Client connected: {addr}")
        conn.settimeout(1.0)
        buffer = ""
        try:
            while not self._stop_event.is_set():
                try:
                    data = conn.recv(4096)
                    if not data:
                        break

                    buffer += data.decode('utf-8')

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if not line.strip():
                            continue

                        request = json.loads(line.strip())
                        command = request.get("command")

                        if command == "close":
                            break # Exit loop to close connection

                        response = {}
                        if command == "list_tools":
                            response = {"tools": self.tool_list}
                        elif command == "call_tool":
                            tool_name = request.get("tool_name")
                            response = {"result": "OK", "tool_name": tool_name}
                            if tool_name == "get_pos":
                                response["result"] = 2048
                            elif tool_name == "get_status":
                                response["result"] = {"pos": 2048, "moving": False}
                            elif tool_name == "export_nn":
                                response["result"] = {"dummy_nn": True}
                            elif tool_name == "calibrate_servo":
                                response = {"prompt": "Move servo to min and press enter."}
                            elif tool_name == "move_towards_goal_embedding":
                                self.last_action_vector = request.get("arguments", {}).get("goal_embedding")
                                response["result"] = "OK"
                            elif tool_name == "set_goal_embedding":
                                response = {"status": "OK"}
                            elif tool_name == "execute_behavior":
                                response = {"status": "OK"}

                        conn.sendall((json.dumps(response) + '\n').encode('utf-8'))
                except (socket.timeout, ConnectionResetError, json.JSONDecodeError):
                    break
        finally:
            conn.close()
            print(f"  [MOCK_MCP] Client disconnected: {addr}")

    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.5)
        print(f"  [MOCK_MCP] Server listening on {self.host}:{self.port}")

        while not self._stop_event.is_set():
            try:
                conn, addr = self.server_socket.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
                self.client_threads.append(client_thread)
            except socket.timeout:
                continue

        print("  [MOCK_MCP] Server loop finished.")
        self.server_socket.close()

    def stop(self):
        print("  [MOCK_MCP] Stopping server...")
        self._stop_event.set()
        time.sleep(0.1) # Give threads time to see the event
        # Wait for all client threads to finish
        for thread in self.client_threads:
            thread.join(timeout=1.0)
        print("  [MOCK_MCP] All client threads finished.")

class MockSerial:
    """A mock serial port that emulates pyserial for offline testing."""
    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port = port
        self.is_open = True
        self._in_buffer = b''
        self._out_buffer = b''
        self.name = port
        self.servo_positions = {}

        # --- NEW: Flags for controlling error simulation ---
        self.simulate_no_response = False
        self.simulate_corrupt_response = False
        self.simulate_feetech_error_bit = False
        self.send_bad_checksum = False

        # Pre-populate with a prompt for the console client
        self.write(b"robot>")

    # --- NEW: Helper method to control error flags from tests ---
    def set_error_simulation(self, no_response=False, corrupt_response=False, feetech_error_bit=False):
        """Sets the error simulation flags for the next interaction."""
        self.simulate_no_response = no_response
        self.simulate_corrupt_response = corrupt_response
        self.simulate_feetech_error_bit = feetech_error_bit
        # Always clear the input buffer when changing modes
        self.flushInput()

    def close(self):
        self.is_open = False

    def flushInput(self):
        self._in_buffer = b''

    def write(self, data):
        self._out_buffer += data
        # --- Emulate Console Responses ---
        if b"get_pos 1 --arm 1" in data:
            pos = self.servo_positions.get(1, 1234)
            self._in_buffer += f"Servo 1 on arm 1 current position: {pos}\nrobot>".encode('utf-8')
        elif b"get_pos 1" in data:
            pos = self.servo_positions.get(1, 1234) # Get servo 1 pos or default
            self._in_buffer += f"Servo 1 current position: {pos}\nrobot>".encode('utf-8')
        elif b"set-learning motors on" in data:
            self._in_buffer += b"Motor learning loop set to on\nrobot>"
        elif b"set-learning states off" in data:
            self._in_buffer += b"State learning loop set to off\nrobot>"
        elif b"set-mode 2" in data:
            self._in_buffer += b"Operating mode set to: 2\nrobot>"
        elif b"set_pos 1 2048" in data:
            self._in_buffer += b"Set servo 1 on arm 0 to position 2048\nrobot>"
        elif b"get_current 1" in data:
            self._in_buffer += b"present current: 123 (raw) -> 799.50 mA (0.800 A)\nrobot>"
        elif b"set_sa 1 100" in data:
            self._in_buffer += b"Acceleration for servo 1 on arm 0 set to 100\nrobot>"
        elif b"get_sa 1" in data:
            self._in_buffer += b"current acceleration: 100\nrobot>"
        elif b"set_tl 1 500" in data:
            self._in_buffer += b"torque limit read back: 500\nrobot>"
        elif b"set_max_torque 500" in data:
            self._in_buffer += b"Babble max torque limit set to: 500\nrobot>"
        elif b"set_ema_alpha 0.2" in data:
            self._in_buffer += b"EMA alpha set to: 0.200000\nrobot>"
        elif b"set_traj_step 5" in data:
            self._in_buffer += b"Trajectory step size set to: 5\nrobot>"
        elif b"set_max_accel 100" in data:
            self._in_buffer += b"Babble min acceleration value set to: 100\nrobot>"
        elif b"plan-move 1 2 3 4 5 6" in data:
            self._in_buffer += b"Goal set.\nrobot>"
        elif b"save" in data:
            self._in_buffer += b"Saving network to NVS...\nrobot>"
        elif b"export" in data:
            self._in_buffer += b"{\"hidden_layer\":{...}}\nrobot>"
        elif b"reset_nn" in data:
            self._in_buffer += b"Forcing network re-initialization\nrobot>"
        elif b"start-data-acq" in data:
            self._in_buffer += b"Starting data acquisition\nrobot>"
        elif b"get_accel_raw" in data:
            self._in_buffer += b"Raw Accelerometer: X=0.0, Y=0.0, Z=1.0 (G)\nrobot>"
        elif b"start_map_cal 1" in data:
            self._in_buffer += b"--- Starting Calibration for Servo 1 on Arm 0 ---\nrobot>"
        elif b"get_stats" in data:
            self._in_buffer += b"Task Name\tStatus\tPrio\tHWM\tTask#\nrobot>"
        elif b"get_wifi_config" in data:
            self._in_buffer += b"Attempting to connect to SSID: YourSSID\nrobot>"
        elif b"scan_wifi" in data:
            self._in_buffer += b"Found 1 access points:\nrobot>"
        elif b"rw-set-params 50 100" in data:
            self._in_buffer += b"Random walk parameters updated\nrobot>"
        elif b"export-states" in data:
            self._in_buffer += b"--- BEGIN STATE EXPORT ---\n--- END STATE EXPORT ---\nrobot>"
        elif b"import-states '{\"centroids\":[]}'" in data:
            self._in_buffer += b"Successfully imported 0 state tokens.\nrobot>"
        elif b"set-mode 2" in data:
            self._in_buffer += b"Operating mode set to: 2\nrobot>"
        elif b"rw-set-params 50 100" in data:
            self._in_buffer += b"Random walk parameters updated\nrobot>"
        elif b"export-states 1" in data:
            self._in_buffer += b"--- BEGIN STATE EXPORT ---\n0.1,0.2,0.3\n--- END STATE EXPORT ---\nrobot>"
        elif b"get-energy-stats" in data:
            self._in_buffer += b"--- Energy Consumption Statistics ---\nPeak Current (A): 1.234\nAverage Current (A): 0.567\nTotal Samples: 100\n-------------------------------------\nrobot>"
        elif b"this_is_not_a_command" in data:
            self._in_buffer += b"Unknown command\nrobot>"

        # --- Emulate Feetech Responses ---
        elif data.startswith(b'\xFF\xFF'):
            servo_id = data[2]
            instruction = data[4]

            # WRITE or REG_WRITE
            if instruction == 0x03 or instruction == 0x04:
                # Check if it's a write to the goal position register (42)
                if len(data) > 6 and data[5] == 42:
                    pos = struct.unpack('<H', data[6:8])[0]
                    self.servo_positions[servo_id] = pos
                # For WRITE, send a status packet back
                if instruction == 0x03:
                    checksum = (~(servo_id + 0x02 + 0x00)) & 0xFF
                    self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x02, 0x00, checksum])

            # READ
            elif instruction == 0x02:
                read_len = data[6]
                reg_addr = data[5]
                if reg_addr == 101: # Invalid response test
                    self._in_buffer += b'bad data'
                elif read_len == 2:
                    val = self.servo_positions.get(servo_id, 0) # Get pos or default to 0
                    val_low = val & 0xFF
                    val_high = (val >> 8) & 0xFF
                    error = 0
                    checksum = (~(servo_id + 0x04 + error + val_low + val_high)) & 0xFF
                    if self.send_bad_checksum:
                        checksum += 1
                    self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x04, error, val_low, val_high, checksum])
                elif read_len == 1:
                    error = 0
                    val = 0 # Dummy value for moving status
                    checksum = (~(servo_id + 0x03 + error + val)) & 0xFF
                    if self.send_bad_checksum:
                        checksum += 1
                    self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x03, error, val, checksum])

            # SYNC_WRITE
            elif instruction == 0x83:
                if len(data) > 7:
                    # reg_addr = data[5] # Not needed for mock
                    data_len_per_servo = data[6]

                    # Loop through the payload
                    i = 7
                    while i < len(data) - 1: # -1 for checksum
                        sync_servo_id = data[i]
                        servo_data = data[i+1 : i+1+data_len_per_servo]
                        if data_len_per_servo == 2: # Assuming word
                            pos = struct.unpack('<H', bytearray(servo_data))[0]
                            self.servo_positions[sync_servo_id] = pos
                        i += 1 + data_len_per_servo

            # RESET
            elif instruction == 0x06:
                checksum = (~(servo_id + 0x02 + 0x00)) & 0xFF
                self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x02, 0x00, checksum])

            # PING
            elif instruction == 0x01:
                checksum = (~(servo_id + 0x02 + 0x00)) & 0xFF
                self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x02, 0x00, checksum])

            # SYNC_READ
            elif instruction == 0x82:
                # This is a complex command to mock. For now, we'll just acknowledge it
                # by not sending anything back, which the test is designed to handle.
                pass
        return len(data)

    def read(self, size=1):
        if not self._in_buffer:
            time.sleep(0.1) # Emulate timeout
            return b''
        data = self._in_buffer[:size]
        self._in_buffer = self._in_buffer[size:]
        return data

    def readline(self):
        # Find the first newline character
        newline_pos = self._in_buffer.find(b'\n')
        if newline_pos != -1:
            line = self._in_buffer[:newline_pos+1]
            self._in_buffer = self._in_buffer[newline_pos+1:]
            return line
        # If no newline, return what's left after a short delay (emulating timeout)
        time.sleep(0.1)
        line = self._in_buffer
        self._in_buffer = b''
        return line

# --- Configuration ---
MCP_PORT = 8888
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 2  # seconds

class MCPClient:
    """A client to interact with the ESP32's MCP TCP server."""
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self, use_mock=False, mock_server=None):
        """Connects to the TCP server."""
        if use_mock:
            self.host = 'localhost' # Mock server is always local
            print(f"  [MCP] Connecting to MOCK server at {self.host}:{self.port}...")
        else:
            print(f"  [MCP] Connecting to REAL server at {self.host}:{self.port}...")

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            print("  [MCP] Connection successful.")
            return True
        except (socket.error, socket.timeout) as e:
            print(f"  [MCP] ERROR: Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnects from the server."""
        if self.sock:
            try:
                # Send a close command, but don't wait for a response.
                self.sock.sendall((json.dumps({"command": "close"}) + '\n').encode('utf-8'))
            except (socket.error, socket.timeout):
                pass # Ignore errors, the goal is to close the socket anyway.
            finally:
                self.sock.close()
                self.sock = None
                print("  [MCP] Disconnected.")

    def _send_request(self, request_obj):
        """Sends a JSON request and waits for a JSON response."""
        if not self.sock:
            print("  [MCP] ERROR: Not connected.")
            return None
        try:
            # Send the request with a newline terminator
            self.sock.sendall((json.dumps(request_obj) + '\n').encode('utf-8'))

            # Receive the response
            response_data = b""
            while True:
                chunk = self.sock.recv(1024)
                if not chunk:
                    break
                response_data += chunk
                if b'\n' in chunk:
                    break

            response_str = response_data.decode('utf-8').strip()
            if not response_str:
                return None
            return json.loads(response_str)
        except (socket.error, json.JSONDecodeError, socket.timeout) as e:
            print(f"  [MCP] ERROR: Request failed: {e}")
            return None

    def list_tools(self):
        """Sends a 'list_tools' command."""
        print("  [MCP] Requesting tool list...")
        return self._send_request({"command": "list_tools"})

    def call_tool(self, tool_name, arguments=None):
        """Sends a 'call_tool' command."""
        print(f"  [MCP] Calling tool '{tool_name}' with args: {arguments}")
        request = {"command": "call_tool", "tool_name": tool_name}
        if arguments:
            request["arguments"] = arguments
        return self._send_request(request)

class ConsoleClient:
    """A client to interact with the ESP32's command-line interface."""
    def __init__(self, port):
        self.port = port
        self.ser = None

    def connect(self, use_mock=False):
        """Connects to the serial port."""
        if use_mock:
            print(f"\n  [CONSOLE] Connecting to MOCK port {self.port}...")
            self.ser = MockSerial(self.port)
            return True

        print(f"\n  [CONSOLE] Connecting to REAL port {self.port}...")
        try:
            self.ser = serial.Serial(self.port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
            time.sleep(2) # Wait for device to settle
            self.ser.flushInput()
            print("  [CONSOLE] Connection successful.")
            return True
        except serial.SerialException as e:
            print(f"  [CONSOLE] ERROR: Could not open port {self.port}: {e}")
            return False

    def disconnect(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("  [CONSOLE] Disconnected.")

    def send_command(self, command, prompt="robot>"):
        """Sends a command and reads the output until the next prompt."""
        if not self.ser:
            print("  [CONSOLE] ERROR: Not connected.")
            return ""

        print(f"  [CONSOLE] Sending command: '{command}'")
        self.ser.write((command + '\n').encode('utf-8'))

        # Read lines until we see the prompt again
        output = ""
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                if not line:
                    break # Timeout
                output += line
                if prompt in line:
                    break
            except serial.SerialException as e:
                print(f"  [CONSOLE] ERROR: Read failed: {e}")
                break
        return output

class FeetechClient:
    """A client to emulate a host sending Feetech servo commands."""
    def __init__(self, port):
        self.port = port
        self.ser = None

    def connect(self, use_mock=False):
        """Connects to the serial port."""
        if use_mock:
            print(f"\n  [FEETECH] Connecting to MOCK port {self.port}...")
            self.ser = MockSerial(self.port)
            return True

        print(f"\n  [FEETECH] Connecting to REAL port {self.port}...")
        try:
            self.ser = serial.Serial(self.port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
            time.sleep(1)
            print("  [FEETECH] Connection successful.")
            return True
        except serial.SerialException as e:
            print(f"  [FEETECH] ERROR: Could not open port {self.port}: {e}")
            return False

    def disconnect(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("  [FEETECH] Disconnected.")

    def _send_packet(self, servo_id, instruction, params=None):
        """Constructs, sends a Feetech packet, and returns the response."""
        if params is None:
            params = []

        length = len(params) + 2 # instruction + checksum
        packet = [0xFF, 0xFF, servo_id, length, instruction] + params

        checksum = sum(packet[2:]) & 0xFF
        packet.append(~checksum & 0xFF)

        print(f"  [FEETECH] Sending packet: {' '.join([f'{b:02X}' for b in packet])}")
        self.ser.write(bytearray(packet))

        # Read response
        response = self.ser.read(16) # Read up to 16 bytes
        print(f"  [FEETECH] Received response: {' '.join([f'{b:02X}' for b in response])}")
        # Basic validation
        if len(response) > 0 and response[0:2] != b'\xFF\xFF':
            return b''
        return response

    def write_word(self, servo_id, reg_addr, value):
        """Sends a WRITE_WORD command."""
        low_byte = value & 0xFF
        high_byte = (value >> 8) & 0xFF
        params = [reg_addr, low_byte, high_byte]
        return self._send_packet(servo_id, 0x03, params) # 0x03 = INST_WRITE

    def read_word(self, servo_id, reg_addr):
        """Sends a READ_WORD command."""
        params = [reg_addr, 2] # Read 2 bytes
        return self._send_packet(servo_id, 0x02, params) # 0x02 = INST_READ

    def read_byte(self, servo_id, reg_addr):
        """Sends a READ_BYTE command."""
        params = [reg_addr, 1] # Read 1 byte
        return self._send_packet(servo_id, 0x02, params) # 0x02 = INST_READ

    def ping(self, servo_id):
        """Sends a PING command."""
        return self._send_packet(servo_id, 0x01) # 0x01 = INST_PING

    def reg_write_word(self, servo_id, reg_addr, value):
        """Sends a REG_WRITE_WORD command."""
        low_byte = value & 0xFF
        high_byte = (value >> 8) & 0xFF
        params = [reg_addr, low_byte, high_byte]
        return self._send_packet(servo_id, 0x04, params) # 0x04 = INST_REG_WRITE

    def action(self):
        """Sends a broadcast ACTION command."""
        return self._send_packet(0xFE, 0x05) # 0xFE = broadcast, 0x05 = INST_ACTION

    def reset(self, servo_id):
        """Sends a RESET command."""
        return self._send_packet(servo_id, 0x06)

    def sync_write(self, reg_addr, data_len_per_servo, servo_data):
        """
        Sends a SYNC_WRITE command.
        servo_data is a list of tuples, e.g., [(id1, [d1, d2]), (id2, [d1, d2])]
        """
        params = [reg_addr, data_len_per_servo]
        for servo_id, data_bytes in servo_data:
            params.append(servo_id)
            params.extend(data_bytes)
        return self._send_packet(0xFE, 0x83, params) # 0xFE = broadcast, 0x83 = INST_SYNC_WRITE

    def sync_read(self, servo_ids, reg_addr, data_len_per_servo):
        """Sends a SYNC_READ command."""
        params = [reg_addr, data_len_per_servo] + servo_ids
        return self._send_packet(0xFE, 0x82, params) # 0xFE = broadcast, 0x82 = INST_SYNC_READ
