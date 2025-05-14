import requests

url= 'http://192.168.0.156:8000/api/chat/history?page=1&per_page=2'
data = requests.get(url).json()

print(f"user_query : {data['messages'][0]['question']}, response : {data['messages'][0]['answer']}")

