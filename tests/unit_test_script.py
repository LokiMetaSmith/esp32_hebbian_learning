import argparse
import sys
import time
import struct

from robot_client import MCPClient, ConsoleClient, FeetechClient, MockMCPServer, MockSerial

def go_to_home_position(client):
    """Commands the robot to a safe, upright home position."""
    print("  [MCP] Moving to home position...")
    home_positions = {
        1: 2045,  # Base
        2: 1500,  # Shoulder
        3: 1500,  # Elbow
        4: 2045,  # Wrist
        5: 2045,  # Wrist rotation
        6: 1000,  # Gripper
    }
    for servo_id, pos in home_positions.items():
        client.call_tool("set_pos", {"id": servo_id, "pos": pos})
        time.sleep(0.1)
    time.sleep(1) # Wait for moves to complete
    print("  [MCP] Arrived at home position.")

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
        # Set a safe, low acceleration for all tests
        print("  [MCP] Setting safe acceleration for tests...")
        response = client.call_tool("set_acceleration", {"accel": 100})
        assert response and response.get("result") == "OK", "MCP Pre-Test Failed: Could not set safe acceleration."

        if not use_mock:
            go_to_home_position(client)

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

        # Test 8: Move Towards Goal Embedding
        goal_embedding = [0.1] * 16
        response = client.call_tool("move_towards_goal_embedding", {"goal_embedding": goal_embedding})
        assert response and response.get("result") == "OK", "MCP Test 8 Failed: move_towards_goal_embedding."
        # This is a bit of a hack, but we'll check that the mock server received the goal embedding
        assert mock_server.last_action_vector == goal_embedding, "MCP Test 8 Failed: Mock server did not receive goal embedding."
        print("  [MCP] Test 8 (Move Towards Goal Embedding): PASSED")

        # Test 9: Set Goal Embedding
        goal_embedding = [0.2] * 16
        response = client.call_tool("set_goal_embedding", {"goal_embedding": goal_embedding})
        assert response and response.get("status") == "OK", "MCP Test 9 Failed: set_goal_embedding."
        print("  [MCP] Test 9 (Set Goal Embedding): PASSED")

        # Test 10: Execute Behavior
        behavior_sequence = {
            "embeddings": [
                [0.1] * 16,
                [0.2] * 16
            ]
        }
        response = client.call_tool("execute_behavior", behavior_sequence)
        assert response and response.get("status") == "OK", "MCP Test 10 Failed: execute_behavior."
        print("  [MCP] Test 10 (Execute Behavior): PASSED")

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

def go_to_home_position_console(client):
    """Commands the robot to a safe, upright home position using the console."""
    print("  [CONSOLE] Moving to home position...")
    home_positions = {
        1: 2045,  # Base
        2: 1500,  # Shoulder
        3: 1500,  # Elbow
        4: 2045,  # Wrist
        5: 2045,  # Wrist rotation
        6: 1000,  # Gripper
    }
    for servo_id, pos in home_positions.items():
        client.send_command(f"set_pos {servo_id} {pos}")
    time.sleep(1) # Wait for moves to complete
    print("  [CONSOLE] Arrived at home position.")

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
        # Set a safe, low acceleration for all tests
        print("  [CONSOLE] Setting safe acceleration for tests...")
        output = client.send_command("set_accel 100")
        assert "acceleration set" in output, "Console Pre-Test Failed: Could not set safe acceleration."

        if not use_mock:
            go_to_home_position_console(client)

        # Test 1: Get Position
        output = client.send_command("get_pos 1")
        assert "current position" in output, "Console Test 1 Failed: 'get_pos' output incorrect."
        output = client.send_command("get_pos 1 --arm 1")
        assert "current position" in output, "Console Test 1 Failed: 'get_pos' with arm_id output incorrect."
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

        # Test 5: Set Mode
        output = client.send_command("set-mode 2")
        assert "Operating mode set to: 2" in output, "Console Test 5 Failed: 'set-mode' output incorrect."
        print("  [CONSOLE] Test 5 (set-mode): PASSED")

        # Test 6: Set Random Walk Params
        output = client.send_command("rw-set-params 50 100")
        assert "Random walk parameters updated" in output, "Console Test 6 Failed: 'rw-set-params' output incorrect."
        print("  [CONSOLE] Test 6 (rw-set-params): PASSED")

        # Test 7: Export States
        output = client.send_command("export-states")
        assert "--- BEGIN STATE EXPORT ---" in output and "--- END STATE EXPORT ---" in output, "Console Test 7 Failed: 'export-states' output markers missing."
        print("  [CONSOLE] Test 7 (export-states): PASSED")

        # Test 8: Import States
        output = client.send_command("import-states '{\"centroids\":[]}'")
        assert "Successfully imported 0 state tokens" in output, "Console Test 8 Failed: 'import-states' with empty list output incorrect."
        print("  [CONSOLE] Test 8 (import-states): PASSED")

        # Test 9: Set Mode
        output = client.send_command("set-mode 2")
        assert "Operating mode set to: 2" in output, "Console Test 9 Failed: 'set-mode' output incorrect."
        print("  [CONSOLE] Test 9 (set-mode): PASSED")

        # Test 10: Set Random Walk Params
        output = client.send_command("rw-set-params 50 100")
        assert "Random walk parameters updated" in output, "Console Test 10 Failed: 'rw-set-params' output incorrect."
        print("  [CONSOLE] Test 10 (rw-set-params): PASSED")

        # Test 11: Export States
        output = client.send_command("export-states 1")
        assert "--- BEGIN STATE EXPORT ---" in output and "--- END STATE EXPORT ---" in output, "Console Test 11 Failed: 'export-states' output markers missing."
        print("  [CONSOLE] Test 11 (export-states): PASSED")

        # Test 12: Get Energy Stats
        output = client.send_command("get-energy-stats")
        assert "Energy Consumption Statistics" in output, "Console Test 12 Failed: 'get-energy-stats' output incorrect."
        assert "Peak Current (A)" in output, "Console Test 12 Failed: 'get-energy-stats' output incorrect."
        assert "Average Current (A)" in output, "Console Test 12 Failed: 'get-energy-stats' output incorrect."
        assert "Total Samples" in output, "Console Test 12 Failed: 'get-energy-stats' output incorrect."
        print("  [CONSOLE] Test 12 (get-energy-stats): PASSED")

        # Test 9: State Learning Loop Moves Servos
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
        test_pos_3 = 2045
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

        # Test 6: PING
        response = client.ping(servo_id)
        assert len(response) >= 6 and response[4] == 0x00, "Feetech Test 6 Failed: Invalid status packet on PING."
        print("  [FEETECH] Test 6 (ping): PASSED")

    except (AssertionError, IndexError, struct.error) as e:
        print(f"  [FEETECH] !!! TEST FAILED: {e}")
        return False
    finally:
        client.disconnect()

    print("\n>>> FEETECH TESTS COMPLETED SUCCESSFULLY <<<")
    return True

def run_logic_tests():
    """Runs tests on pure logic functions that don't require hardware/mocks."""
    print("\n" + "="*50)
    print(">>> RUNNING PURE LOGIC UNIT TESTS")
    print("="*50)

    # Test Correction Map Logic
    # Re-implementing the C logic in Python to test its correctness.
    def get_corrected_position_py(commanded_pos, points):
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i+1]
            if commanded_pos >= p1['commanded'] and commanded_pos <= p2['commanded']:
                fraction = (commanded_pos - p1['commanded']) / (p2['commanded'] - p1['commanded'])
                return p1['actual'] + fraction * (p2['actual'] - p1['actual'])
        if commanded_pos < points[0]['commanded']:
            return points[0]['actual']
        else:
            return points[-1]['actual']

    mock_points = [
        {'commanded': 0, 'actual': 100},
        {'commanded': 1000, 'actual': 1100},
        {'commanded': 2000, 'actual': 2100},
        {'commanded': 3000, 'actual': 3100},
        {'commanded': 4000, 'actual': 4100},
    ]

    try:
        assert abs(get_corrected_position_py(500, mock_points) - 600) < 1, "Logic Test 1 Failed: Interpolation."
        assert abs(get_corrected_position_py(2000, mock_points) - 2100) < 1, "Logic Test 1 Failed: Exact point."
        assert abs(get_corrected_position_py(-100, mock_points) - 100) < 1, "Logic Test 1 Failed: Below range."
        assert abs(get_corrected_position_py(5000, mock_points) - 4100) < 1, "Logic Test 1 Failed: Above range."
        print("  [LOGIC] Test 1 (Correction Map): PASSED")
        return True
    except AssertionError as e:
        print(f"  [LOGIC] !!! TEST FAILED: {e}")
        return False

def run_unit_tests():
    """Runs all test suites against mock objects."""
    print("\n" + "#"*60)
    print("### RUNNING ALL UNIT TESTS (NO HARDWARE REQUIRED)")
    print("#"*60)
    mcp_ok = run_mcp_tests('localhost', 8888, use_mock=True)
    console_ok = run_console_tests('mock_console_port', use_mock=True)
    feetech_ok = run_feetech_tests('mock_feetech_port', use_mock=True)
    logic_ok = run_logic_tests()
    return all([mcp_ok, console_ok, feetech_ok, logic_ok])

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

        mcp_ok = run_mcp_tests(args.ip_address, 8888)
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
