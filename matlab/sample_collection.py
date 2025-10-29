import struct
from collections import Counter
import serial
import csv



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


def compute_sample_byte_size():
    return sum(s.byte_size for s in SAMPLE_STRUCTURE)

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

def process_hex_str(hex_string):
    data = bytes.fromhex(hex_string)
    
    processed_data = []
    for offset in range(0, len(data), SAMPLE_BYTE_SIZE):
        sample_bytes = data[offset: offset + SAMPLE_BYTE_SIZE]
        decoded_data = decode_sample(sample_bytes)
        processed_data.append(decoded_data)
        
    return processed_data

IMU_GYRO  = SampleDataType(0, 12,'<3f', gyro_labels)   # 3 floats = 12 bytes
IMU_ACC   = SampleDataType(1, 12, '<3f', accel_labels)
IMU_DATA  = SampleDataType(2, 24, '<6f', imu_data_labels)
LEFT_AS6500_DATA  = SampleDataType(3, 2, '<H', ['left_as6500'])   # uint16 is it little endian?
RIGHT_AS6500_DATA = SampleDataType(4, 2, '<H', ['right_as6500'])
FLOAT_DATA        = SampleDataType(5, 4, '<f', ['float'])

SAMPLE_STRUCTURE = [LEFT_AS6500_DATA]
SAMPLE_BYTE_SIZE = compute_sample_byte_size()
SAMPLE_BATCH_SIZE = 5;


print(f"Total byte size = {SAMPLE_BYTE_SIZE}")
hex_string = "3f8000004000000040400000" 



# Example with pyserial
arduino = serial.Serial('COM7', 115200)
samples_to_collect = 200
sample_count = 0

payload_buffer = []

try:
    while sample_count < samples_to_collect:
        line = arduino.readline().decode(errors='ignore').strip()
        if line.startswith("DATA: "):
            payload = line[len("DATA: "): ]
            payload_buffer.append(payload)
            sample_count += SAMPLE_BATCH_SIZE
            print(sample_count)
            
        elif line.startswith("DEBUG"):
            print(line)
        
finally :
    arduino.close()

    
        
decoded_payload = [processed_sample for processed_sample in process_hex_str(payload) for payload in payload_buffer]

with open('imu_batch.csv', 'w+', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(generate_csv_headers())
    writer.writerows(decoded_payload)

        


        



