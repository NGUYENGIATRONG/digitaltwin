
import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold
import numpy as np
import time
from test_spot_policy import motor_angles_list
import serial
import logging
import serial.tools.list_ports
from collections import deque
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

class SignalFilter:
    """Bộ lọc tín hiệu kết hợp nhiều phương pháp lọc nhiễu"""
    
    def __init__(self, signal_size=8, window_size=5, outlier_threshold=2.5):
        self.signal_size = signal_size  # Số lượng giá trị trong mảng tín hiệu
        self.window_size = window_size  # Kích thước cửa sổ trượt cho bộ lọc trung bình động
        self.outlier_threshold = outlier_threshold  # Ngưỡng phát hiện ngoại lệ (số lần độ lệch chuẩn)
        
        # Cửa sổ trượt cho mỗi kênh tín hiệu
        self.signal_history = [deque(maxlen=window_size) for _ in range(signal_size)]
        
        # Biến trạng thái cho bộ lọc Kalman đơn giản
        self.kalman_state = np.zeros(signal_size)
        self.kalman_p = np.ones(signal_size)  # Độ không chắc chắn của ước lượng
        self.kalman_q = 0.1  # Nhiễu quá trình (process noise)
        self.kalman_r = 1.0  # Nhiễu đo lường (measurement noise)
        
        # Giá trị cuối cùng hợp lệ
        self.last_valid_signal = np.zeros(signal_size)
        self.initialized = False
        
    def update(self, new_signal):
        """Cập nhật bộ lọc với tín hiệu mới và trả về tín hiệu đã lọc"""
        if len(new_signal) != self.signal_size:
            return self.last_valid_signal
        
        # Chuyển đổi tín hiệu sang numpy array để xử lý
        signal = np.array(new_signal)
        
        # Khởi tạo trạng thái nếu đây là lần đầu tiên
        if not self.initialized:
            self.kalman_state = signal.copy()#sao chep du lieu
            self.last_valid_signal = signal.copy()
            for i in range(self.signal_size):
                self.signal_history[i].append(signal[i])##luu tin hieu vao lich su
            self.initialized = True#danh dau la da khoi tao
            return signal
        
        # Kiểm tra và loại bỏ ngoại lệ
        filtered_signal = self._outlier_removal(signal)
        
        # Cập nhật lịch sử tín hiệu
        for i in range(self.signal_size):
            self.signal_history[i].append(filtered_signal[i])
        
        # Áp dụng bộ lọc trung bình động
        ma_signal = self._moving_average()#tra ve list 8 gia tri trung binh cong cua 5 deque hop le
        
        # Áp dụng bộ lọc Kalman đơn giản
        final_signal = self._kalman_filter(ma_signal)
        
        # Lưu giá trị cuối cùng
        self.last_valid_signal = final_signal
        
        return final_signal
    
    def _outlier_removal(self, signal):
        """Phát hiện và loại bỏ các giá trị ngoại lệ"""
        result = signal.copy()
        
        for i in range(self.signal_size):
            if len(self.signal_history[i]) >= 3:  # Cần ít nhất 3 giá trị để phát hiện ngoại lệ
                # Tính toán trung bình và độ lệch chuẩn của lịch sử
                history = np.array(list(self.signal_history[i]))
                mean = np.mean(history)
                std = np.std(history)
                
                # Kiểm tra nếu giá trị mới nằm ngoài ngưỡng
                if std > 0 and abs(signal[i] - mean) > self.outlier_threshold * std:
                    # Thay thế giá trị ngoại lệ bằng giá trị cuối cùng hợp lệ
                    result[i] = self.last_valid_signal[i]
                    logging.warning(f"Outlier detected in channel {i}: {signal[i]}, replaced with {result[i]}")
        
        return result
    
    def _moving_average(self):
        """Tính toán trung bình động cho mỗi kênh tín hiệu"""
        result = np.zeros(self.signal_size)
        
        for i in range(self.signal_size):
            result[i] = np.mean(self.signal_history[i])
        
        return result
    
    def _kalman_filter(self, measurement):
        """Áp dụng bộ lọc Kalman đơn giản cho mỗi kênh tín hiệu"""
        result = np.zeros(self.signal_size)
        
        for i in range(self.signal_size):
            # Bước dự đoán
            # (ở đây giả định rằng trạng thái không thay đổi nhiều giữa các lần cập nhật)
            x_pred = self.kalman_state[i]
            p_pred = self.kalman_p[i] + self.kalman_q
            
            # Bước cập nhật
            k = p_pred / (p_pred + self.kalman_r)  # Kalman gain
            self.kalman_state[i] = x_pred + k * (measurement[i] - x_pred)
            self.kalman_p[i] = (1 - k) * p_pred
            
            result[i] = self.kalman_state[i]
        
        return result
    
    def get_last_valid(self):
        """Trả về tín hiệu đã lọc cuối cùng"""
        return self.last_valid_signal
    
signal_filter = SignalFilter(signal_size=8, 
                             window_size=5,
                             outlier_threshold=1.5)
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

while True:
    # Đọc dữ liệu từ cổng nối tiếp
    arr = read_data()
    
    # Xử lý dữ liệu nếu hợp lệ
    if len(arr) == 8:
        # Nếu có dữ liệu hợp lệ, sử dụng nó trong mô phỏng
        filtered_arr=signal_filter.update(arr)
        fixed_motor_angles = [
            np.deg2rad(-(arr[1] - 115)), #12
            np.deg2rad(- arr[0] + 160), #11
            np.deg2rad(arr[2] - 100), #21
            np.deg2rad(arr[3] - 105), #22
            np.deg2rad(- arr[7] + 210), #42
            np.deg2rad(- arr[6] + 140), #41
            np.deg2rad(arr[4] - 140), #31
            np.deg2rad(arr[5] - 50) #32
        ]
        last_valid_angles = fixed_motor_angles[:]
    else:
        fixed_motor_angles = last_valid_angles
    
    # Thực hiện một bước mô phỏng
    state, r, done, info = env.step_motorangles(fixed_motor_angles)
    # print(f"Góc động cơ: {fixed_motor_angles}")
    
    # Tùy chọn: thêm một khoảng thời gian nhỏ để giảm tải CPU
    # time.sleep(0.01)
# Vòng lặp chính để nhập dữ liệu và điều khiển mô phỏng
# while True:
#     # Nhập dữ liệu từ người dùng
#     # data_to_send = list(map(int, input("Type (space separated values): ").split()))  # Dữ liệu muốn gửi
#     # send_data(data_to_send)  # Gửi dữ liệu
#     # time.sleep(0.01)
#     # print("aaa")
#     # Đọc dữ liệu trả về từ cổng nối tiếp
#     arr = read_data()
#     # print(f"arr {arr}")
#     last_valid_angles = [0, 0, 0, 0, 0, 0, 0, 0]
#     if len(arr) == 8:
#         # Nếu có dữ liệu hợp lệ, sử dụng nó trong mô phỏng
#         fixed_motor_angles = [
#             np.deg2rad(-(arr[1] - 115) ), #12
#             np.deg2rad(- arr[0] + 160) , #11
#             np.deg2rad(arr[2] - 90), #21
#             np.deg2rad(arr[3] - 100), #22
#             np.deg2rad(- arr[7] + 210), #42
#             np.deg2rad(- arr[6] + 140), #41
#             np.deg2rad(arr[4] - 140) , #31
#             np.deg2rad(arr[5] - 50) #32
#             # 0, 0, 0, 0, 0, 0, 0 , 0
#         ]
#         last_valid_angles = fixed_motor_angles[:]
#     else:
#         fixed_motor_angles = last_valid_angles
    
#     # print(arr)
#     # Robot giữ nguyên góc động cơ cố định
#     t_r = 0
#     while t_r < 50:
#         # Sử dụng góc động cơ cố định cho mỗi bước
#         state, r, done, info = env.step_motorangles(fixed_motor_angles)
#         print(f"goc dong co {fixed_motor_angles}")
#         t_r+=1
#         # print(f"Angles: {np.degrees(env.get_motor_angles()) }")
#         # print(f"goc nhan{arr}")
#     # while True:
#     #     state, r, done, info = env.step_motorangles(fixed_motor_angles)
        
#     #     arr = read_data()  # Kiểm tra xem có dữ liệu mới không
#     #     if arr:  # Nếu có dữ liệu mới, thoát vòng lặp để cập nhật
#     #         break  

#     # continue_input = input("Do you want to continue? (y/n): ").strip().lower()
#     # # Câu hỏi nếu muốn tiếp tục hay thoát chương trình
#     # if continue_input != 'y':
#     #     logging.info("Exiting the program.")
#     #     break  # Dừng vòng lặp chính nếu người dùng chọn thoát

