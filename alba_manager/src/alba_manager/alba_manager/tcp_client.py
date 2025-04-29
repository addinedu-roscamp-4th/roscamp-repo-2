# File: alba_planner/alba_manager/tcp_client.py

import socket
import json

class AlbaTCPClient:
    """
    TCP client for communicating with the RoboDine Service.
    """
    def __init__(self, host: str = "192.168.0.156", port: int = 8001, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout

    def send_data(self, data: dict) -> str:
        """
        Send JSON-serializable data to the RoboDine Service over TCP.

        Args:
            data: Dictionary of data to send. Must include 'id'.
        Returns:
            The response string received from the server.
        """
        payload = json.dumps(data).encode('utf-8')
        response = ""

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.timeout)
                sock.connect((self.host, self.port))
                sock.sendall(payload)
                # Receive up to 4096 bytes
                resp_bytes = sock.recv(4096)
                response = resp_bytes.decode('utf-8')
        except Exception as e:
            raise ConnectionError(f"TCP connection or send/receive failed: {e}")

        return response
