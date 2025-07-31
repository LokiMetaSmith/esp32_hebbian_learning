import socket
import json
import time
import serial
import argparse
import struct
import sys
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
        self.tool_list = [
            {"name": "set_torque"}, {"name": "set_acceleration"},
            {"name": "get_status"}, {"name": "get_current"},
            {"name": "get_power"}, {"name": "get_torque"},
            {"name": "export_data"}, {"name": "export_nn"},
            {"name": "import_nn"}, {"name": "import_nn_json"},
            {"name": "calibrate_servo"}, {"name": "babble_start"},
            {"name": "babble_stop"}, {"name": "set_pos"}, {"name": "get_pos"}
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
                            elif tool_name == "babble_start":
                                response["result"] = "babble task started"
                            elif tool_name == "babble_stop":
                                response["result"] = "babble task stopped"

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
        self.servo_positions = {} # Store servo positions
        # Pre-populate with a prompt for the console client
        self.write(b"robot>")

    def close(self):
        self.is_open = False

    def flushInput(self):
        self._in_buffer = b''

    def write(self, data):
        self._out_buffer += data
        # --- Emulate Console Responses ---
        if b"get_pos 1" in data:
            pos = self.servo_positions.get(1, 1234) # Get servo 1 pos or default
            self._in_buffer += f"Servo 1 current position: {pos}\nrobot>".encode('utf-8')
        elif b"set-learning motors on" in data:
            self._in_buffer += b"Motor learning loop set to on\nrobot>"
        elif b"set-learning states off" in data:
            self._in_buffer += b"State learning loop set to off\nrobot>"
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
                # Check if it's a read from the present position register (56)
                if len(data) > 5 and data[5] == 56:
                    val = self.servo_positions.get(servo_id, 0) # Get pos or default to 0
                    val_low = val & 0xFF
                    val_high = (val >> 8) & 0xFF
                    checksum = (~(servo_id + 0x04 + 0x00 + val_low + val_high)) & 0xFF
                    self._in_buffer += bytes([0xFF, 0xFF, servo_id, 0x04, 0x00, val_low, val_high, checksum])

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

def run_mcp_tests(host, port, use_mock=False):
    """Runs a suite of tests against the MCP server."""
    mock_server = None
    print("\n" + "="*50)
    if use_mock:
        print(">>> RUNNING MCP SERVER UNIT TESTS (Mocked)")
        mock_server = MockMCPServer(host, port)
        mock_server.start()
        time.sleep(0.1) # Give server time to start
    else:
        print(">>> RUNNING MCP SERVER INTEGRATION TESTS (Wi-Fi)")
    print("="*50)

    client = MCPClient(host, port)
    if not client.connect(use_mock=use_mock):
        if mock_server: mock_server.stop()
        return False

    try:
        # Test 1: List tools
        response = client.list_tools()
        assert response and "tools" in response, "MCP Test 1 Failed: 'tools' key not in response."
        tool_names = [t['name'] for t in response['tools']]
        # Check for new tools
        assert "set_torque" in tool_names, "MCP Test 1 Failed: Missing set_torque tool."
        assert "set_acceleration" in tool_names, "MCP Test 1 Failed: Missing set_acceleration tool."
        assert "get_status" in tool_names, "MCP Test 1 Failed: Missing get_status tool."
        print("  [MCP] Test 1 (list_tools): PASSED")

        # Test 2: Set/Get Position
        servo_id, test_pos = 1, 2048
        response = client.call_tool("set_pos", {"id": servo_id, "pos": test_pos})
        assert response and response.get("result") == "OK", "MCP Test 2 Failed: Did not get 'OK' on set_pos."
        time.sleep(0.5)
        response = client.call_tool("get_pos", {"id": servo_id})
        read_pos = response.get("result")
        assert isinstance(read_pos, int) and abs(read_pos - test_pos) < 50, \
            f"MCP Test 2 Failed: Position mismatch. Expected ~{test_pos}, got {read_pos}."
        print(f"  [MCP] Test 2 (set_pos/get_pos): PASSED (Read back {read_pos})")

        # Test 2a: Servo Control
        servo_id = 1
        response = client.call_tool("set_torque", {"id": servo_id, "torque": False})
        assert response and response.get("result") == "OK", "MCP Test 2 Failed: set_torque off."
        time.sleep(0.5)
        response = client.call_tool("set_torque", {"id": servo_id, "torque": True})
        assert response and response.get("result") == "OK", "MCP Test 2 Failed: set_torque on."
        time.sleep(0.5)
        response = client.call_tool("set_acceleration", {"id": servo_id, "accel": 100})
        assert response and response.get("result") == "OK", "MCP Test 2 Failed: set_acceleration."
        print("  [MCP] Test 2 (Servo Control): PASSED")


        # Test 3: Get Data
        response = client.call_tool("get_status", {"id": servo_id})
        assert response and "result" in response, "MCP Test 3 Failed: get_status."
        status = response["result"]
        assert "moving" in status and "pos" in status, "MCP Test 3 Failed: get_status content."

        response = client.call_tool("get_current", {"id": servo_id})
        assert response and "result" in response, "MCP Test 3 Failed: get_current."

        response = client.call_tool("get_power", {"id": servo_id})
        assert response and "result" in response, "MCP Test 3 Failed: get_power."

        response = client.call_tool("get_torque", {"id": servo_id})
        assert response and "result" in response, "MCP Test 3 Failed: get_torque."
        print("  [MCP] Test 3 (Get Data): PASSED")

        # Test 4: Export Data
        response = client.call_tool("export_data")
        assert response and "result" in response, "MCP Test 4 Failed: export_data."
        print("  [MCP] Test 4 (Export Data): PASSED")

        # Test 5: Export/Import NN
        response = client.call_tool("export_nn")
        assert response and "result" in response and response["result"], "MCP Test 5 Failed: export_nn."
        nn_data = response["result"]

        response = client.call_tool("import_nn", {"data": nn_data})
        assert response and response.get("result") == "OK", "MCP Test 5 Failed: import_nn."
        print("  [MCP] Test 5 (Export/Import NN): PASSED")

        # Test 6: Import NN from JSON
        # This requires a JSON object representing the neural network.
        # For now, we'll just send a dummy object.
        nn_json = {
            "hidden_layer": {
                "weights": [[0.1] * 20] * 10,
                "biases": [0.1] * 10
            },
            "output_layer": {
                "weights": [[0.1] * 10] * 5,
                "biases": [0.1] * 5
            },
            "prediction_layer": {
                "weights": [[0.1] * 5] * 2,
                "biases": [0.1] * 2
            }
        }
        response = client.call_tool("import_nn_json", {"nn_data": nn_json})
        assert response and response.get("result") == "OK", "MCP Test 6 Failed: import_nn_json."
        print("  [MCP] Test 6 (Import NN JSON): PASSED")

        # Test 7: Calibrate Servo (Interactive)
        # This test will require manual interaction.
        # It's here to ensure the command can be called.
        response = client.call_tool("calibrate_servo", {"id": 1})
        assert response and "prompt" in response, "MCP Test 7 Failed: calibrate_servo."
        print("  [MCP] Test 7 (Calibrate Servo): PASSED (prompt received)")

        # Test 8: Babble Start/Stop
        response = client.call_tool("babble_start")
        assert response and "started" in response.get("result", ""), "MCP Test 8 Failed: babble_start."
        print("  [MCP] Test 8 (babble_start): PASSED")
        time.sleep(1)
        response = client.call_tool("babble_stop")
        assert response and "stopped" in response.get("result", ""), "MCP Test 8 Failed: babble_stop."
        print("  [MCP] Test 8 (babble_stop): PASSED")


    except AssertionError as e:
        print(f"  [MCP] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()
        if mock_server:
            mock_server.stop()
            mock_server.join()

    print("\n>>> MCP TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_console_tests(port, use_mock=False):
    """Runs a suite of tests against the console CLI."""
    print("\n" + "="*50)
    if use_mock:
        print(">>> RUNNING CONSOLE CLI UNIT TESTS (Mocked)")
    else:
        print(">>> RUNNING CONSOLE CLI INTEGRATION TESTS (USB)")
    print("="*50)

    client = ConsoleClient(port)
    if not client.connect(use_mock=use_mock):
        return False

    try:
        # Test 1: Get Position
        output = client.send_command("get_pos 1")
        assert "current position" in output, "Console Test 1 Failed: 'get_pos' output incorrect."
        print("  [CONSOLE] Test 1 (get_pos): PASSED")

        # Test 2: Random Walk Start/Stop
        if not use_mock: # This test requires real hardware interaction
            output = client.send_command("rw_start")
            assert "Random Walk task created" in output or "resumed/started" in output, "Console Test 2 Failed: 'rw_start' output incorrect."
            print("  [CONSOLE] Test 2 (rw_start): PASSED")
            time.sleep(1)
            output = client.send_command("rw_stop")
            assert "stopped" in output, "Console Test 2 Failed: 'rw_stop' output incorrect."
            print("  [CONSOLE] Test 2 (rw_stop): PASSED")

        # Test 3: Invalid Command
        output = client.send_command("this_is_not_a_command")
        assert "Unknown command" in output, "Console Test 3 Failed: Did not get error for invalid command."
        print("  [CONSOLE] Test 3 (invalid command): PASSED")

        # Test 4: Set Learning
        output = client.send_command("set-learning motors on")
        assert "Motor learning loop set to on" in output, "Console Test 4 Failed: 'set-learning motors on' output incorrect."
        output = client.send_command("set-learning states off")
        assert "State learning loop set to off" in output, "Console Test 4 Failed: 'set-learning states off' output incorrect."
        print("  [CONSOLE] Test 4 (set-learning): PASSED")

        # Test 5: State Learning Loop Moves Servos
        if not use_mock: # This test requires real hardware interaction
            output = client.send_command("get_pos 1")
            start_pos_str = output.split("position:")[1].strip()
            start_pos = int(start_pos_str)

            client.send_command("set-learning states on")
            time.sleep(1) # Let it run for a second
            client.send_command("set-learning states off")

            output = client.send_command("get_pos 1")
            end_pos_str = output.split("position:")[1].strip()
            end_pos = int(end_pos_str)

            assert start_pos != end_pos, "Console Test 5 Failed: State learning loop did not move the servo."
            print(f"  [CONSOLE] Test 5 (state learning moves servos): PASSED (pos changed from {start_pos} to {end_pos})")

    except (AssertionError, IndexError, ValueError) as e:
        print(f"  [CONSOLE] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()

    print("\n>>> CONSOLE TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_feetech_tests(port, use_mock=False):
    """Runs tests emulating a Feetech host application."""
    print("\n" + "="*50)
    if use_mock:
        print(">>> RUNNING FEETECH PROTOCOL UNIT TESTS (Mocked)")
    else:
        print(">>> RUNNING FEETECH PROTOCOL INTEGRATION TESTS (USB)")
    print("="*50)

    client = FeetechClient(port)
    if not client.connect(use_mock=use_mock):
        return False

    try:
        servo_id, test_pos = 2, 1024

        # Test 1: Write Position
        response = client.write_word(servo_id, 42, test_pos) # 42 = REG_GOAL_POSITION
        # A valid status packet is 6 bytes: FF FF ID 02 00 CHK
        assert len(response) >= 6 and response[0:2] == b'\xFF\xFF', "Feetech Test 1 Failed: Invalid status packet on write."
        assert response[2] == servo_id, "Feetech Test 1 Failed: Wrong servo ID in response."
        assert response[4] == 0x00, "Feetech Test 1 Failed: Error bit set in status packet."
        print("  [FEETECH] Test 1 (write_word): PASSED")
        time.sleep(0.5)

        # Test 2: Read Position
        response = client.read_word(servo_id, 56) # 56 = REG_PRESENT_POSITION
        # A valid read response is 8 bytes: FF FF ID 04 00 D_L D_H CHK
        assert len(response) >= 8 and response[0:2] == b'\xFF\xFF', "Feetech Test 2 Failed: Invalid packet on read."
        assert response[4] == 0x00, "Feetech Test 2 Failed: Error bit set in read response."

        # Unpack the little-endian word
        read_val = struct.unpack('<H', response[5:7])[0]
        assert abs(read_val - test_pos) < 50, \
            f"Feetech Test 2 Failed: Position mismatch. Expected ~{test_pos}, got {read_val}."
        print(f"  [FEETECH] Test 2 (read_word): PASSED (Read back {read_val})")

        # Test 3: REG_WRITE and ACTION
        test_pos_3 = 3000
        client.reg_write_word(servo_id, 42, test_pos_3) # No response expected for REG_WRITE
        time.sleep(0.1)
        client.action() # No response for broadcast ACTION
        time.sleep(0.5)
        response = client.read_word(servo_id, 56)
        read_val_3 = struct.unpack('<H', response[5:7])[0]
        assert abs(read_val_3 - test_pos_3) < 50, \
            f"Feetech Test 3 Failed: REG_WRITE/ACTION position mismatch. Expected ~{test_pos_3}, got {read_val_3}."
        print(f"  [FEETECH] Test 3 (reg_write/action): PASSED (Read back {read_val_3})")

        # Test 4: SYNC_WRITE
        servo_data = [
            (1, [1024 & 0xFF, (1024 >> 8) & 0xFF]),
            (2, [3072 & 0xFF, (3072 >> 8) & 0xFF])
        ]
        client.sync_write(42, 2, servo_data) # No response for SYNC_WRITE
        time.sleep(0.5)
        response1 = client.read_word(1, 56)
        read_val1 = struct.unpack('<H', response1[5:7])[0]
        assert abs(read_val1 - 1024) < 50, f"Feetech Test 4 Failed: SYNC_WRITE pos mismatch for servo 1. Got {read_val1}"
        response2 = client.read_word(2, 56)
        read_val2 = struct.unpack('<H', response2[5:7])[0]
        assert abs(read_val2 - 3072) < 50, f"Feetech Test 4 Failed: SYNC_WRITE pos mismatch for servo 2. Got {read_val2}"
        print(f"  [FEETECH] Test 4 (sync_write): PASSED (Read back {read_val1}, {read_val2})")

        # Test 5: RESET
        response = client.reset(servo_id)
        assert len(response) >= 6 and response[4] == 0x00, "Feetech Test 5 Failed: Invalid status packet on RESET."
        print("  [FEETECH] Test 5 (reset): PASSED")


    except (AssertionError, IndexError, struct.error) as e:
        print(f"  [FEETECH] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()

    print("\n>>> FEETECH TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_unit_tests():
    """Runs all test suites against mock objects."""
    print("\n" + "#"*60)
    print("### RUNNING ALL UNIT TESTS (NO HARDWARE REQUIRED)")
    print("#"*60)
    mcp_ok = run_mcp_tests('localhost', MCP_PORT, use_mock=True)
    console_ok = run_console_tests('mock_console_port', use_mock=True)
    feetech_ok = run_feetech_tests('mock_feetech_port', use_mock=True)
    return all([mcp_ok, console_ok, feetech_ok])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32 Hebbian Robot Test Suite")
    parser.add_argument("--mode", choices=['unit', 'integration'], default='integration', help="Test mode: 'unit' (mocked) or 'integration' (real hardware).")
    parser.add_argument("--ip_address", help="IP address of the ESP32 device for integration tests.")
    parser.add_argument("--console_port", help="Serial port for the Console CLI for integration tests.")
    parser.add_argument("--feetech_port", help="Serial port for the Feetech Protocol Proxy for integration tests.")
    args = parser.parse_args()

    success = False
    if args.mode == 'unit':
        success = run_unit_tests()
    else:
        if not all([args.ip_address, args.console_port, args.feetech_port]):
            parser.error("For 'integration' mode, --ip_address, --console_port, and --feetech_port are required.")

        mcp_ok = run_mcp_tests(args.ip_address, MCP_PORT)
        console_ok = run_console_tests(args.console_port)
        feetech_ok = run_feetech_tests(args.feetech_port)
        success = all([mcp_ok, console_ok, feetech_ok])

    print("\n" + "="*50)
    print(">>> FINAL TEST SUMMARY")
    print("="*50)
    print(f"  Mode: {args.mode.upper()}")
    print(f"  Overall Result: {'PASSED' if success else 'FAILED'}")
    print("="*50)

    if not success:
        sys.exit(1)
