import json
import socket
import struct

# Use a 4-byte unsigned integer for the length prefix. '!' means network byte order.
# This can handle messages up to 4GB in size (2**32 - 1 bytes).
LENGTH_PREFIX_FORMAT = "!I"

def send_json(sock: socket.socket, data: dict):
    """
    Serializes a dictionary to JSON, prefixes it with its length, and sends it.
    """
    try:
        # 1. Serialize the dictionary to a JSON string, then encode to bytes.
        json_bytes = json.dumps(data).encode('utf-8')

        # 2. Pack the length of the JSON data into a 4-byte header.
        length_prefix = struct.pack(LENGTH_PREFIX_FORMAT, len(json_bytes))

        # 3. Send the length prefix, followed by the actual JSON data.
        sock.sendall(length_prefix)
        sock.sendall(json_bytes)

    except (BrokenPipeError, ConnectionResetError):
        print("Client connection closed.")
    except Exception as e:
        print(f"Error sending data: {e}")


def recv_json(sock: socket.socket) -> dict | None:
    """
    Receives a length-prefixed JSON message and deserializes it.
    """
    try:
        # 1. Read the 4-byte length prefix.
        len_prefix_bytes = sock.recv(struct.calcsize(LENGTH_PREFIX_FORMAT))
        if not len_prefix_bytes:
            # Connection closed gracefully by the other side.
            return None

        # 2. Unpack the length prefix to get the message length.
        message_length = struct.unpack(LENGTH_PREFIX_FORMAT, len_prefix_bytes)[0]

        # 3. Read the full message data in a loop.
        # This is the crucial part for handling large messages that may be fragmented.
        received_payload = b""
        remaining_payload_size = message_length

        while remaining_payload_size > 0:
            chunk = sock.recv(min(remaining_payload_size, 4096))
            if not chunk:
                # This would be an unexpected connection close.
                raise ConnectionError("Socket connection broken before all data was received.")
            received_payload += chunk
            remaining_payload_size -= len(chunk)

        # 4. Decode the UTF-8 payload and deserialize the JSON.
        return json.loads(received_payload.decode('utf-8'))

    except (ConnectionResetError, ConnectionError, struct.error):
        # Handle cases where the connection is dropped or data is malformed.
        print("Connection closed or data was malformed.")
        return None
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None
