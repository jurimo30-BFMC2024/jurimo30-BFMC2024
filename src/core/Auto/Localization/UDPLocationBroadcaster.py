import socket
import threading
import time

class UDPLocationBroadcaster:
    def __init__(self, localization, port=4202, rate_hz=5):
        self.localization = localization
        self.port = port
        self.interval = 1.0 / rate_hz
        self.running = False
        self.thread = threading.Thread(target=self._broadcast_loop, daemon=True)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _broadcast_loop(self):
        while self.running:
            pos = self.localization.get_location()
            if pos:
                message = f"{pos[0]:.3f},{pos[1]:.3f}".encode("utf-8")
                self.sock.sendto(message, ('<broadcast>', self.port))
            time.sleep(self.interval)
