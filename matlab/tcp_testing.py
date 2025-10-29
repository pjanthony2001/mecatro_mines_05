import socket
import struct
import numpy as np
import threading
import time

# ---------- TCP Setup ----------
TCP_IP = '192.168.4.1'
TCP_PORT = 5000

def tcp_heartbeat():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.connect((TCP_IP, TCP_PORT))
    print("Connected to Arduino via TCP")
    try:
        while True:
            heartbeat = bytes([0x02, 0x01])
            tcp_sock.sendall(heartbeat)
            time.sleep(4)
    except KeyboardInterrupt:
        pass
    finally:
        tcp_sock.close()


tcp_heartbeat()