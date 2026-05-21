import urllib.request
import json
import random
import time

# Make sure this matches the ID of the buoy you added in the dashboard!
# If you added '05', change it to '05'.
NODE_ID = "02" 

URL = "http://127.0.0.1:5000/api/telemetry/add"

print(f"📡 Simulating Iridium transmission for Buoy {NODE_ID}...")

# 1. Generate fake sensor data with slight random drift
data = {
    "node_id": NODE_ID,
    "lat": -62.5 + random.uniform(-0.05, 0.05),
    "lon": -58.8 + random.uniform(-0.05, 0.05),
    "hs": round(random.uniform(1.5, 4.5), 2),
    "moments": round(random.uniform(0.1, 1.0), 2),
    "battery": random.randint(85, 100),
    "rssi": random.randint(-120, -90)
}

# 2. Package it as JSON and send it to our Flask API
req = urllib.request.Request(
    URL, 
    data=json.dumps(data).encode('utf-8'), 
    headers={'Content-Type': 'application/json'}
)

try:
    # 3. Fire the request
    response = urllib.request.urlopen(req)
    print("✅ Success! Server responded:", response.read().decode('utf-8'))
    print(f"Data sent: {data}")
except Exception as e:
    print("❌ Error:", e)