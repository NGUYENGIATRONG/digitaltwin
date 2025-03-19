"""import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold
import numpy as np
import time
from test_spot_policy import motor_angles_list
import serial
import logging
import serial.tools.list_ports

logging.basicConfig(level=logging.INFO)

def find_serial_port():
    # Lấy danh sách tất cả các cổng nối tiếp có sẵn
    ports = serial.tools.list_ports.comports()

    # Kiểm tra các cổng nối tiếp và tìm cổng phù hợp
    for port in ports:
        logging.info(f"Found port: {port.device} - {port.description}")
        if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device:
            logging.info(f"Matched port: {port.device}")
            return port.device
    return None

# Tìm cổng nối tiếp phù hợp
USE_PORT = find_serial_port()
if USE_PORT:
    logging.info(f"Found the serial port: {USE_PORT}")
else:
    logging.error("No suitable serial port found.")

BAUD_RATE = 115200

# Thử mở kết nối với cổng nối tiếp đã tìm thấy
try:
    if USE_PORT:
        ser = serial.Serial(USE_PORT, BAUD_RATE, timeout=1)
        logging.info("Serial port opened successfully.")
        ON = 1
    else:
        raise serial.SerialException("Invalid serial port.")
except serial.SerialException as e:
    logging.error(f"Failed to open serial port: {e}")
    ser = None
    ON = 0

def send_data(data):
 
    if ser and ser.is_open:
        try:
            # Dữ liệu cần truyền
            byte_data = bytearray([255]) + bytearray(data) + bytearray([254])
            ser.write(byte_data)  # Gửi dữ liệu
            logging.info(f"Sent data: {byte_data}")
        except serial.SerialException as e:
            logging.error(f"Error writing to serial port: {e}")
    else:
        logging.error("Serial port is not available or opened....")


def read_data():

    buffer = []
    flag = False  
    data_list = [] 

    while ser.in_waiting > 0:
        byte = ord(ser.read(1))  
        if byte == 255:  
            buffer = [byte]  
            flag = True  
        elif byte == 254 and flag:  
            buffer.append(byte) 
   
            if len(buffer[1:-1]) == 8: 
                arr_data = buffer[1:-1]
                logging.info(f"Received data between 255 and 254: {arr_data}")
                buffer = []  
                flag = False 
            else:
                logging.error("Incorrect data")
        elif flag:
            buffer.append(byte)

    return arr_data
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

# Các tham số chương trình
parser.add_argument('--FrictionCoeff', help='hệ số ma sát chân được đặt', type=float, default=1.6)
parser.add_argument('--WedgeIncline', help='độ nghiêng của mặt phẳng nghiêng', type=int, default=15)
parser.add_argument('--MotorStrength', help='cường độ động cơ tối đa được áp dụng', type=float, default=7.0)
args = parser.parse_args()

# Khởi tạo môi trường mô phỏng
env = spot.SpotEnv(render=True,
                    on_rack=True,
                    gait='trot',
                    default_pos=(-1, 0, 0.2))


state = env.reset()


while True:
    data_to_send = list(map(int, input("Type: ").split()))  # Ví dụ dữ liệu muốn gửi
    send_data(data_to_send)  # Gửi dữ liệu
    time.sleep(0.01)

    arr = read_data()
    # Góc động cơ cố định mà bạn chọn (thay đổi các giá trị này theo nhu cầu)
    fixed_motor_angles = [np.radians(arr[0] - 20), np.radians( 40 + (180 - arr[1]) ),np.radians(arr[2]- 20),np.radians(40 + (180 - arr[1]) ),np.radians(arr[0] -20),np.radians(40 + (180 - arr[1]) ),np.radians( arr[0] -20),np.radians(40 + (180 - arr[1]))]  # Giá trị góc theo radian (có thể thay đổi)
    #a - 20
    #40 +(180 - a)
    #hip knee

    


  
    # Robot giữ nguyên góc động cơ cố định
    t_r = 0

    while 1:
        # Sử dụng góc động cơ cố định cho mỗi bước
        state, r, done, info = env.step_motorangles(fixed_motor_angles)
        t_r += r
        # motor_angles_list=env.get_motor_angles()
        # print(np.degrees(motor_angles_list))
        # In thông tin góc động cơ tại mỗi bước (nếu cần)"""
        

import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold
import numpy as np
import time
from test_spot_policy import motor_angles_list
import serial
import logging
import serial.tools.list_ports

logging.basicConfig(level=logging.INFO)

def find_serial_port():
    # Lấy danh sách tất cả các cổng nối tiếp có sẵn
    ports = serial.tools.list_ports.comports()

    # Kiểm tra các cổng nối tiếp và tìm cổng phù hợp
    for port in ports:
        logging.info(f"Found port: {port.device} - {port.description}")
        if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device:
            logging.info(f"Matched port: {port.device}")
            return port.device
    return None

# Tìm cổng nối tiếp phù hợp
USE_PORT = find_serial_port()
if USE_PORT:
    logging.info(f"Found the serial port: {USE_PORT}")
else:
    logging.error("No suitable serial port found.")

BAUD_RATE = 115200

# Thử mở kết nối với cổng nối tiếp đã tìm thấy
try:
    if USE_PORT:
        ser = serial.Serial(USE_PORT, BAUD_RATE, timeout=1)
        logging.info("Serial port opened successfully.")
        ON = 1
    else:
        raise serial.SerialException("Invalid serial port.")
except serial.SerialException as e:
    logging.error(f"Failed to open serial port: {e}")
    ser = None
    ON = 0

def send_data(data):
    """Gửi dữ liệu với định dạng 255 ... 254"""
    if ser and ser.is_open:
        try:
            # Dữ liệu cần truyền
            byte_data = bytearray([255]) + bytearray(data) + bytearray([254])
            ser.write(byte_data)  # Gửi dữ liệu
            logging.info(f"Sent data: {byte_data}")
        except serial.SerialException as e:
            logging.error(f"Error writing to serial port: {e}")
    else:
        logging.error("Serial port is not available or opened....")

def read_data():
    """Đọc dữ liệu từ cổng nối tiếp và trả về dữ liệu hợp lệ dưới dạng list"""
    buffer = []
    flag = False  # Biến kiểm tra nếu bắt đầu và kết thúc đúng
    data_list = []  # Danh sách để chứa các dữ liệu hợp lệ
    arr_data = []

    while ser.in_waiting > 0:
        byte = ord(ser.read(1))  
        if byte == 255:  
            buffer = [byte]  
            flag = True  
        elif byte == 254 and flag:  
            buffer.append(byte) 
   
            if len(buffer[1:-1]) == 8: 
                arr_data = buffer[1:-1]
                logging.info(f"Received data between 255 and 254: {arr_data}")
                buffer = []  
                flag = False 
            else:
                logging.error("Incorrect data")
        elif flag:
            buffer.append(byte)

    return arr_data

# Khởi tạo đối số từ dòng lệnh
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

# Các tham số chương trình
parser.add_argument('--FrictionCoeff', help='hệ số ma sát chân được đặt', type=float, default=1.6)
parser.add_argument('--WedgeIncline', help='độ nghiêng của mặt phẳng nghiêng', type=int, default=15)
parser.add_argument('--MotorStrength', help='cường độ động cơ tối đa được áp dụng', type=float, default=7.0)
args = parser.parse_args()

# Khởi tạo môi trường mô phỏng
env = spot.SpotEnv(render=True,
                    on_rack=True,
                    gait='trot',
                    default_pos=(-1, 0, 0.2))

state = env.reset()

# Vòng lặp chính để nhập dữ liệu và điều khiển mô phỏng
motor_angles_history = [[] for _ in range(8)]
while True:
    # Nhập dữ liệu từ người dùng
    # data_to_send = list(map(int, input("Type (space separated values): ").split()))  # Dữ liệu muốn gửi
    # send_data(data_to_send)  # Gửi dữ liệu
    # time.sleep(0.01)
    # print("aaa")
    # Đọc dữ liệu trả về từ cổng nối tiếp
    arr = read_data()
    # print(f"arr {arr}")
    last_valid_angles = [0, 0, 0, 0, 0, 0, 0, 0]
    if len(arr) == 8:
        # Nếu có dữ liệu hợp lệ, sử dụng nó trong mô phỏng
        fixed_motor_angles = [
            np.deg2rad(-(arr[1] - 220) ), #12
            np.deg2rad(- arr[0] + 160) , #11
            np.deg2rad(arr[2] - 100), #21
            np.deg2rad(arr[3] - 140), #22
            np.deg2rad(- arr[7] + 210), #42
            np.deg2rad(- arr[6] + 140), #41
            np.deg2rad(arr[4] - 180 + 30) , #31
            np.deg2rad(arr[5] - 60) #32
            # 0, 0, 0, 0, 0, 0, 0 , 0
        ]
        last_valid_angles = fixed_motor_angles[:]
    else:
        fixed_motor_angles = last_valid_angles
    
    if fixed_motor_angles:
        # Lưu từng phần tử vào danh sách riêng
        for i in range(8):
            motor_angles_history[i].append(fixed_motor_angles[i])

            # Lưu từng danh sách vào file .npy riêng biệt
            np.save(f"motor_angle_{i+1}.npy", np.array(motor_angles_history[i]))

        logging.info("Saved motor angles to separate files.")
    # print(arr)
    # Robot giữ nguyên góc động cơ cố định
    t_r = 0
    while t_r < 10:
        # Sử dụng góc động cơ cố định cho mỗi bước
        state, r, done, info = env.step_motorangles(fixed_motor_angles)
        print(f"goc dong co {fixed_motor_angles}")
        t_r+=1
        print(f"Angles: {np.degrees(env.get_motor_angles()) }")
        print(f"goc nhan{arr}")

    # continue_input = input("Do you want to continue? (y/n): ").strip().lower()
    # # Câu hỏi nếu muốn tiếp tục hay thoát chương trình
    # if continue_input != 'y':
    #     logging.info("Exiting the program.")
    #     break  # Dừng vòng lặp chính nếu người dùng chọn thoát
