import json
import os

def route_to_file(message: dict, filename="data/captures/mock_cam_message.json"):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, "w") as f:
        json.dump(message, f, indent=2)
    print(f"[✓] Message written to {filename}")
