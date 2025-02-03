import socket
import sys

UDP_IP = "0.0.0.0"
UDP_PORT = 12345
msg = sys.argv[1]
MESSAGE = bytes(msg, "utf-8")

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM,
                     ) # UDP
sock.setsockopt(socket.IPPROTO_IP, socket.IP_TTL, 2)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))