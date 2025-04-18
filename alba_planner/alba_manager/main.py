#alba_manager/main.py

from tcp_client import AlbaTCPClient

tcp_client = AlbaTCPClient()
tcp_client.connect()

# 예시 데이터
data = {
  "id": "string4",
  "mac_address": "1sdsdddds23ssdsd34",
  "ip_address": "string",
  "status": "string",
  "location": "string",
  "battery_level": 0,
  "timestamp": "2025-04-18T03:23:12.765Z"
}

tcp_client.send_data(data)
tcp_client.close()
