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

    def recv_all(self, sock: socket.socket, count: int) -> bytes:
        """정확히 count 바이트를 받을 때까지 recv() 반복"""
        buf = bytearray()
        while len(buf) < count:
            chunk = sock.recv(count - len(buf))
            if not chunk:
                raise ConnectionError("데이터 수신 중 연결이 끊어졌습니다.")
            buf.extend(chunk)
        return bytes(buf)

    def send_payload(self, payload: dict):
        raw = json.dumps(payload).encode('utf-8')
        # 1) 4바이트 big-endian 헤더
        header = len(raw).to_bytes(4, byteorder='big')
        # 2) 헤더 + 바디를 한 번에 전송
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((self.host, self.port))
            sock.sendall(header + raw)
            raw_len = self.recv_all(sock, 4)
            total_len = int.from_bytes(raw_len, byteorder='big')
            # 3) 실제 페이로드(바디)만 읽기
            body = self.recv_all(sock, total_len)
            data = json.loads(body.decode('utf-8'))
            return data

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
