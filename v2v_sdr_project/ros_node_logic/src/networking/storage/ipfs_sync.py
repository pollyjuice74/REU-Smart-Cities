import ipfshttpclient
import json
import time

class IPFSSync:
    def __init__(self, topic="cav_sync"):
        self.client = ipfshttpclient.connect('/dns/localhost/tcp/5001/http')
        self.topic = topic

    def publish_metadata(self, metadata: dict):
        data = json.dumps(metadata)
        self.client.pubsub.publish(self.topic, data.encode())

    def listen(self, callback):
        sub = self.client.pubsub.subscribe(self.topic)
        for msg in sub:
            metadata = json.loads(msg['data'].decode())
            callback(metadata)

# Example usage
if __name__ == "__main__":
    ipfs = IPFSSync()

    def on_receive(data):
        print("[IPFS] Got metadata:", data)

    # Start listener in background
    import threading
    threading.Thread(target=ipfs.listen, args=(on_receive,), daemon=True).start()

    while True:
        ipfs.publish_metadata({"id": "car001", "pos": [1,2], "timestamp": time.time()})
        time.sleep(3)
