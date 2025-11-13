import struct
from collections import Counter
import serial
import csv
import socket
import struct
import numpy as np
import threading
import time



# -----------------------------
# 1. Define SampleDataType class
# -----------------------------
class SampleDataType:
    def __init__(self, id: int, byte_size: int, fmt: str, field_names):

        self.id = id
        self.byte_size = byte_size
        self.fmt = fmt
        self.field_names = field_names
    
    def __int__(self):
        return self.id
    

# ---------------------------------
# 2. Define sample types (like constexprs)
# ---------------------------------
gyro_labels = ['gyro_x', 'gyro_y', 'gyro_z']
accel_labels = ['accel_x', 'accel_y', 'accel_z']
imu_data_labels = ['gyro_x', 'gyro_y', 'gyro_z'] + ['accel_x', 'accel_y', 'accel_z']


def compute_data_types(fmt: int):
    data_types = []
    if ((fmt >> 7) & 1):
        data_types.append(TIME_STAMP)
    if ((fmt >> 6) & 1):
        data_types.append(IMU_GYRO)
    if ((fmt >> 5) & 1):
        data_types.append(IMU_ACC)
    if (fmt >> 4) & 1:
        data_types.append(LEFT_AS6500_DATA)
    if (fmt >> 3) & 1:
        data_types.append(RIGHT_AS6500_DATA)
    if (fmt >> 2) & 1:
        data_types.append(FLOAT_DATA)

    return data_types

def compute_sample_byte_size(structure):
    return sum([data.byte_size for data in structure])

def generate_csv_headers():
    
    counts = Counter([s.id for s in SAMPLE_STRUCTURE])
    type_indices = {id: 0 for id in counts}
    headers = []

    for s in SAMPLE_STRUCTURE:
        idx = type_indices[s.id]
        type_indices[s.id] += 1

        for f in s.field_names:
            headers.append(f"{f}_{idx}")

    return headers

def decode_sample(bytestream: bytes):
    offset = 0
    decoded = []
    for s in SAMPLE_STRUCTURE:
        part = bytestream[offset: offset + s.byte_size]
        offset += s.byte_size
        decoded += struct.unpack(s.fmt, part)
    
    return decoded

def process_hex_str(data):
   
    processed_data = []
    for offset in range(0, len(data), SAMPLE_BYTE_SIZE):
        sample_bytes = data[offset: offset + SAMPLE_BYTE_SIZE]
        decoded_data = decode_sample(sample_bytes)
        processed_data.append(decoded_data)
        
    return processed_data

IMU_GYRO  = SampleDataType(0, 12,'<3f', gyro_labels)   # 3 floats = 12 bytes
IMU_ACC   = SampleDataType(1, 12, '<3f', accel_labels)
IMU_DATA  = SampleDataType(2, 24, '<6f', imu_data_labels)
LEFT_AS6500_DATA  = SampleDataType(3, 4, '<l', ['left_as6500'])   # uint16 is it little endian?
RIGHT_AS6500_DATA = SampleDataType(4, 4, '<l', ['right_as6500'])
FLOAT_DATA        = SampleDataType(5, 4, '<f', ['float'])
TIME_STAMP = SampleDataType(6, 4, '<L', ['time_stamp'])

# DEVICE SAMPLE STRUCTURE : [sampleTime][gyroData][accelData][leftEncoder][rightEncoder][float_1][0][0]
DEVICE_SAMPLE_FMT = 0b10010100
SAMPLE_BATCH_SIZE = 30
SAMPLE_STRUCTURE = compute_data_types(DEVICE_SAMPLE_FMT)
SAMPLE_BYTE_SIZE = compute_sample_byte_size(SAMPLE_STRUCTURE)


samples_to_collect = 1000
sample_count = 0

payload_buffer = []

USB = False
WIFI = True

if USB:
    arduino = serial.Serial('COM7', 115200)

    try:
        while sample_count < samples_to_collect:
            line = arduino.readline().decode(errors='ignore').strip()
            if line.startswith("DATA: "):
                payload = line[len("DATA: "): ]
                data = bytes.fromhex(payload) 
                payload_buffer.append(data)
                sample_count += SAMPLE_BATCH_SIZE
                
            elif line.startswith("DEBUG"):
                print(line)
            
    finally :
        arduino.close()
        
elif WIFI:
    UDP_IP = "192.168.4.1"  # Arduino IP
    UDP_PORT = 1234

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('', UDP_PORT))
    print("connected")
    
    try:
        while sample_count < samples_to_collect:
            data, addr = udp_sock.recvfrom(SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE + 50)  # buffer 50 bytes
            
            payload_buffer.append(data)
            sample_count += SAMPLE_BATCH_SIZE
    finally :
        udp_sock.close()
    
    

    
decoded_payload = []
for payload in payload_buffer:
    processed_samples = process_hex_str(payload)
    for sample in processed_samples:
        decoded_payload.append(sample)


with open('imu_batch.csv', 'w+', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(generate_csv_headers())
    writer.writerows(decoded_payload)

        


        



