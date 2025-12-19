import requests
import json

try:
    print("Sending Voice Command Test...")
    res = requests.post(
        "http://localhost:8000/api/voice-command", 
        json={"cmd": "TEST", "text": "Hello World"}
    )
    print(f"Status: {res.status_code}")
    print(f"Response: {res.json()}")
except Exception as e:
    print(f"Error: {e}")
