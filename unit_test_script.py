import socket
import json
import time
import serial
import argparse
import struct
import sys

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

    def connect(self):
        """Connects to the TCP server."""
        print(f"  [MCP] Connecting to {self.host}:{self.port}...")
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

    def connect(self):
        """Connects to the serial port."""
        print(f"\n  [CONSOLE] Connecting to {self.port}...")
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

    def connect(self):
        """Connects to the serial port."""
        print(f"\n  [FEETECH] Connecting to {self.port}...")
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

def run_mcp_tests(host, port):
    """Runs a suite of tests against the MCP server."""
    print("\n" + "="*50)
    print(">>> RUNNING MCP SERVER TESTS (Wi-Fi)")
    print("="*50)
    
    client = MCPClient(host, port)
    if not client.connect():
        return False

    try:
        # Test 1: List tools
        response = client.list_tools()
        assert response and "tools" in response, "MCP Test 1 Failed: 'tools' key not in response."
        tool_names = [t['name'] for t in response['tools']]
        assert "set_pos" in tool_names and "get_pos" in tool_names, "MCP Test 1 Failed: Missing required tools."
        print("  [MCP] Test 1 (list_tools): PASSED")

        # Test 2: Set position
        servo_id, test_pos = 1, 2048
        response = client.call_tool("set_pos", {"id": servo_id, "pos": test_pos})
        assert response and response.get("result") == "OK", "MCP Test 2 Failed: Did not get 'OK' on set_pos."
        print("  [MCP] Test 2 (set_pos): PASSED")
        time.sleep(0.5)

        # Test 3: Get position
        response = client.call_tool("get_pos", {"id": servo_id})
        read_pos = response.get("result")
        assert isinstance(read_pos, int) and abs(read_pos - test_pos) < 50, \
            f"MCP Test 3 Failed: Position mismatch. Expected ~{test_pos}, got {read_pos}."
        print(f"  [MCP] Test 3 (get_pos): PASSED (Read back {read_pos})")

        # Test 4: Babble start/stop
        response = client.call_tool("babble_start")
        assert response and "started" in response.get("result", ""), "MCP Test 4 Failed: babble_start."
        print("  [MCP] Test 4 (babble_start): PASSED")
        time.sleep(1)
        response = client.call_tool("babble_stop")
        assert response and "stopped" in response.get("result", ""), "MCP Test 4 Failed: babble_stop."
        print("  [MCP] Test 4 (babble_stop): PASSED")

    except AssertionError as e:
        print(f"  [MCP] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()
    
    print("\n>>> MCP TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_console_tests(port):
    """Runs a suite of tests against the console CLI."""
    print("\n" + "="*50)
    print(">>> RUNNING CONSOLE CLI TESTS (USB)")
    print("="*50)
    
    client = ConsoleClient(port)
    if not client.connect():
        return False
        
    try:
        # Test 1: Get Position
        output = client.send_command("get_pos 1")
        assert "current position" in output, "Console Test 1 Failed: 'get_pos' output incorrect."
        print("  [CONSOLE] Test 1 (get_pos): PASSED")

        # Test 2: Random Walk Start/Stop
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

    except AssertionError as e:
        print(f"  [CONSOLE] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()
        
    print("\n>>> CONSOLE TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_feetech_tests(port):
    """Runs tests emulating a Feetech host application."""
    print("\n" + "="*50)
    print(">>> RUNNING FEETECH PROTOCOL TESTS (USB)")
    print("="*50)
    
    client = FeetechClient(port)
    if not client.connect():
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

    except (AssertionError, IndexError, struct.error) as e:
        print(f"  [FEETECH] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()

    print("\n>>> FEETECH TESTS COMPLETED SUCCESSFULLY <<<")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32 Hebbian Robot Test Suite")
    parser.add_argument("ip_address", help="IP address of the ESP32 device for MCP tests.")
    parser.add_argument("console_port", help="Serial port for the Console CLI (e.g., /dev/ttyACM0).")
    parser.add_argument("feetech_port", help="Serial port for the Feetech Protocol Proxy (e.g., /dev/ttyACM1).")
    args = parser.parse_args()

    # Run all test suites
    mcp_ok = run_mcp_tests(args.ip_address, MCP_PORT)
    console_ok = run_console_tests(args.console_port)
    feetech_ok = run_feetech_tests(args.feetech_port)

    print("\n" + "="*50)
    print(">>> FINAL TEST SUMMARY")
    print("="*50)
    print(f"  MCP Server (Wi-Fi):      {'PASSED' if mcp_ok else 'FAILED'}")
    print(f"  Console CLI (USB):       {'PASSED' if console_ok else 'FAILED'}")
    print(f"  Feetech Protocol (USB):  {'PASSED' if feetech_ok else 'FAILED'}")
    print("="*50)

    if not all([mcp_ok, console_ok, feetech_ok]):
        sys.exit(1) # Exit with error code if any test failed
