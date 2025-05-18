import socket
import threading
import time
import json

class UDPLocationBroadcaster:
    def __init__(self, port=5001, rate_hz=5):
        self.port = port
        self.interval = 1.0 / rate_hz
        self.running = True
        self.send_thread = threading.Thread(target=self._broadcast_loop, daemon=True)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.last_received = None
        self.last_location = None
        self.send_thread.start()
        self.receive_thread.start()

    def stop(self):
        self.running = False

    def update_location(self, x, y):
        """
        Update the current location to be broadcasted.
        
        Args:
            x (float): The x-coordinate of the location.
            y (float): The y-coordinate of the location.
        """
        self.last_location = (x, y)

    def _broadcast_loop(self):
        while self.running:
            if self.last_location:
                # Format as a JSON array [x, y]
                coordinates = [round(self.last_location[0], 2), round(self.last_location[1], 2)]
                data = json.dumps(coordinates).encode('utf-8')
                
                # Send to the sender
                self.sock.sendto(data, ('<broadcast>', self.port))
            time.sleep(self.interval)

    def _receive_loop(self):
        """
        Listen for incoming data on the specified port.
        """
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.bind(("0.0.0.0", self.port))
        print(f"Listening for incoming data on port {self.port}...")

        while self.running:
            try:
                data, addr = receive_socket.recvfrom(1024)
                received_data = json.loads(data.decode('utf-8'))
                self.last_received = received_data
                print(f"Received data from {addr}: {received_data}")
            except Exception as e:
                print(f"Error receiving data: {e}")

    def get_location(self):
        """
        Return the last received x, y coordinates.
        """
        return self.last_received['x'], self.last_received['y'] if self.last_received else None
