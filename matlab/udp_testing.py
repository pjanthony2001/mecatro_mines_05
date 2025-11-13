import socket
import struct
import numpy as np
import threading
import time

# ---------- UDP Setup ----------
UDP_IP = "192.168.4.1"  # Arduino IP
UDP_PORT = 1234

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('', UDP_PORT))  # bind to UDP PORT

batch_size = 400
channels = 2

def udp_listener():
    print("Starting UDP listener on port", UDP_PORT)
    while True:
        data, addr = udp_sock.recvfrom(SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE + 50)  
        if len(data) == batch_size * channels * 4:
            floats = struct.unpack('<' + 'f' * (batch_size * channels), data)
            arr = np.array(floats).reshape(batch_size, channels)
            print("Received batch:")
            print(arr)


udp_listener()  

    