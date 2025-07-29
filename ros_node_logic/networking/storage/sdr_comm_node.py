import socket
import time

class SDRCommNode:
    def __init__(self, port=5005, peer_ip="192.168.1.2"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer = (peer_ip, port)
        self.sock.bind(("", port))
        self.last_handshake = 0

    def send_hello(self):
        msg = f"HELLO {time.time()}"
        self.sock.sendto(msg.encode(), self.peer)
        print("[SDR] Sent HELLO")

    def listen(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            print("[SDR] Received:", data.decode())

    def run(self):
        import threading
        threading.Thread(target=self.listen, daemon=True).start()

        while True:
            now = time.time()
            if now - self.last_handshake > 2.0:
                self.send_hello()
                self.last_handshake = now
            time.sleep(1)

# Run
if __name__ == "__main__":
    node = SDRCommNode()
    node.run()
