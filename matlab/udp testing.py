import socket
import time

print(len(b"ACK"))

UDP_IP = "192.168.4.1"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))

lastSent = 0
while(True):
    timeNow = time.time()
    if(timeNow - lastSent > 2):
        sock.sendto(b"ACK", (UDP_IP, UDP_PORT))
        lastSent = timeNow
        print("SENT ACK", timeNow, lastSent)

