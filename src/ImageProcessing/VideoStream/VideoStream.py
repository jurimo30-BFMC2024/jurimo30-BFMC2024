import cv2
import socket

class VideoStream:
    def __init__(self, feed_id: int, multicast_group: str = "224.0.0.1", port: int = 4201):
        self.feed_id = feed_id
        self.multicast_group = multicast_group
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

    def display(self, frame):
        """
        Encodes the frame and sends it over multicast with the feed ID prepended.
        """
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        data = bytes([self.feed_id]) + buffer.tobytes()
        self.sock.sendto(data, (self.multicast_group, self.port))

    def close(self):
        self.sock.close()