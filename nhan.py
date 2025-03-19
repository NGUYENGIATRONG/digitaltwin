'''
import serial
import serial.tools.list_ports
import logging

# Cấu hình logging
logging.basicConfig(level=logging.INFO)

# def find_serial_port():
#     """Tự động tìm cổng USB-TTL kết nối."""
#     ports = serial.tools.list_ports.comports()
#     for port in ports:
#         logging.info(f"Found port: {port.device} - {port.description}")
#         if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device:
#             logging.info(f"Matched port: {port.device}")
#             return port.device
#     return None

# Tìm cổng USB-TTL
USE_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
# if not USE_PORT:
#     logging.error("No suitable serial port found.")
#     exit()

# Cấu hình thông số Serial
# BAUD_RATE = 115200  # Thay đổi nếu cần
TIMEOUT = 1         # Thời gian chờ dữ liệu

# try:
#     ser = serial.Serial(USE_PORT, BAUD_RATE, timeout=TIMEOUT)
#     logging.info(f"Connected to {USE_PORT} at {BAUD_RATE} baud.")
# except serial.SerialException as e:
#     logging.error(f"Failed to open serial port: {e}")
#     exit()

# def read_data():
#     """Đọc dữ liệu từ cổng nối tiếp và in dữ liệu nhận được"""
#     buffer = []
#     flag = False  
#     arr_data = []

#     while ser.in_waiting > 0:
#         byte = ord(ser.read(1))  
#         print(f"Received byte: {byte}")  # In từng byte để kiểm tra

#         if byte == 255:  
#             buffer = [byte]  
#             flag = True  
#         elif byte == 254 and flag:  
#             buffer.append(byte) 
#             if len(buffer[1:-1]) == 8:  
#                 arr_data = buffer[1:-1]
#                 print("Final arr_data:", arr_data)  # In mảng dữ liệu nhận được
#                 buffer = []  
#                 flag = False 
#             else:
#                 print("Incorrect data")
#         elif flag:
#             buffer.append(byte)

#     return arr_data
try:
    ser = serial.Serial(USE_PORT, BAUD_RATE, timeout=1)
    print(f"Kết nối thành công với {USE_PORT} ở {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"❌ Lỗi khi mở cổng serial: {e}")
    exit()
print("📡 Đang chờ dữ liệu...")

def read_data():
    data = []
    start_flag = False  # Cờ để kiểm tra khi nào bắt đầu ghi dữ liệu

    while ser.in_waiting > 0:
        byte = ord(ser.read(1))  
        
        # data = [byte]  # Xóa dữ liệu c
        data.append(byte)  # Lưu byte vào danh sách
        if len(data)==8:    
            print(f"Dữ liệu hiện tại: {data}")  # In dữ liệu đang nhận

    return data

# Chạy thử
while True:
    try:
        read_data()
        # if received_data:
        #     print(f"📥 Dữ liệu nhận được: {received_data}")
    except KeyboardInterrupt:
        print("\n⏹ Dừng chương trình.")
        ser.close()
        break
'''
import serial
import logging
import serial.tools.list_ports

logging.basicConfig(level=logging.INFO)

def find_serial_port():
    """Tìm cổng nối tiếp thích hợp"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        logging.info(f"Found port: {port.device} - {port.description}")
        if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device:
            logging.info(f"Using port: {port.device}")
            return port.device
    return None

USE_PORT = find_serial_port()
BAUD_RATE = 115200  # Thử đổi baud rate nếu cần
ser = None

if USE_PORT:
    try:
        ser = serial.Serial(USE_PORT, BAUD_RATE, timeout=2)
        logging.info("Serial port opened successfully.")
    except serial.SerialException as e:
        logging.error(f"Failed to open serial port: {e}")
        ser = None
else:
    logging.error("No suitable serial port found.")

if ser:
    while True:
        if ser.in_waiting > 0:
            raw_data = ser.read(ser.in_waiting)  # Đọc toàn bộ buffer
            logging.info(f"Raw data received: {raw_data}")  # Hiển thị dữ liệu dạng byte



