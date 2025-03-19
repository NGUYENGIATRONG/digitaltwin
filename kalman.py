import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold, yellow, cyan
import numpy as np
import time
from test_spot_policy import motor_angles_list
import serial
import logging
import serial.tools.list_ports
from collections import deque

logging.basicConfig(level=logging.INFO)

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        logging.info(f"Found port: {port.device} - {port.description}")
        if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device : #or "ttyS" in port.device
            logging.info(f"Matched port: {port.device}")
            return port.device
    return None


USE_PORT = find_serial_port()
if USE_PORT:
    logging.info(f"Found the serial port: {USE_PORT}")
else:
    logging.error("No suitable serial port found.")

BAUD_RATE = 115200
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
    flag = False  
    data_list = []  
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

class ResponsiveKalmanFilter:
    """Bộ lọc Kalman với khả năng phản hồi tốt với chuyển động thực"""
    
    def __init__(self, signal_size=8, dt=0.1, 
                 process_variance=0.5,      
                 measurement_variance=0.5,   # thaaps thif nhieu nhieux
                 motion_threshold=15,       # Ngưỡng phát hiện chuyển động có chủ ý Đặt giá trị lớn hơn  để chỉ phản ứng với những thay đổi đáng kể
                 adaptive_factor=12.0):       # Hệ số thích ứng khi phát hiện chuyển động Nếu có chuyển động đột ngột (vượt motion_threshold), bộ lọc sẽ tạm thời tăng process_variance lên gấp 5 lần để bám theo tín hiệu.
        
        self.signal_size = signal_size
        self.dt = dt
        self.base_process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.motion_threshold = motion_threshold
        self.adaptive_factor = adaptive_factor
        
        # Biến lưu trữ tạm thời
        self.prev_measurements = np.zeros(signal_size)
        self.current_process_variance = np.full(signal_size, process_variance)
        
        # Trạng thái (x) cho mỗi kênh tín hiệu [vị trí, vận tốc]
        self.x = np.zeros((signal_size, 2))
        
        # Ma trận hiệp phương sai (P) cho mỗi kênh
        self.P = np.array([np.eye(2) for _ in range(signal_size)])
        
        # Ma trận chuyển trạng thái (A)
        self.A = np.array([[1, dt], [0, 1]])
        
        # Ma trận đo lường (H)
        self.H = np.array([[1, 0]])
        
        # Ma trận nhiễu đo lường (R)
        self.R = np.array([self.measurement_variance])
        
        # Giá trị đã lọc
        self.filtered_values = np.zeros(signal_size)
        self.initialized = False
        
    def detect_movement(self, current, previous, threshold):
        """Phát hiện chuyển động có chủ ý dựa trên sự thay đổi giữa các mẫu"""
        return abs(current - previous) > threshold
    
    def update(self, measurements):
        """Cập nhật bộ lọc Kalman với phát hiện chuyển động thích ứng"""
        if len(measurements) != self.signal_size:
            return self.filtered_values
        
        # Chuyển sang numpy array
        z = np.array(measurements, dtype=float)
        
        # Khởi tạo nếu cần
        if not self.initialized:
            for i in range(self.signal_size):
                self.x[i, 0] = z[i]
                self.filtered_values[i] = z[i]
                self.prev_measurements[i] = z[i]
            self.initialized = True
            return self.filtered_values
        
        # Kiểm tra chuyển động cho mỗi kênh tín hiệu
        for i in range(self.signal_size):
            # Phát hiện chuyển động có chủ ý
            if self.detect_movement(z[i], self.prev_measurements[i], self.motion_threshold):
                # Tăng process_variance tạm thời để theo sát chuyển động
                self.current_process_variance[i] = self.base_process_variance * self.adaptive_factor
            else:
                # Giảm dần process_variance về giá trị cơ bản
                self.current_process_variance[i] = max(
                    self.base_process_variance,
                    self.current_process_variance[i] * 0.9  # Giảm dần 10% mỗi lần
                )
        
        # Lưu các giá trị đo lường hiện tại cho lần sau
        self.prev_measurements = z.copy()
        
        # Dự đoán và cập nhật cho từng kênh với process_variance tùy chỉnh
        for i in range(self.signal_size):
            # Cập nhật ma trận Q dựa trên process_variance tùy chỉnh
            Q = np.array([
                [self.dt**4/4, self.dt**3/2],
                [self.dt**3/2, self.dt**2]
            ]) * self.current_process_variance[i]
            
            # Giai đoạn dự đoán
            self.x[i] = self.A @ self.x[i]
            self.P[i] = self.A @ self.P[i] @ self.A.T + Q
            
            # Giai đoạn cập nhật
            y = z[i] - self.H @ self.x[i]
            S = self.H @ self.P[i] @ self.H.T + self.R
            K = self.P[i] @ self.H.T / S
            self.x[i] = self.x[i] + K.flatten() * y
            self.P[i] = (np.eye(2) - np.outer(K, self.H)) @ self.P[i]
            
            # Lưu giá trị đã lọc (vị trí)
            self.filtered_values[i] = self.x[i, 0]
        
        return self.filtered_values
    
    def reset(self):
        """Reset trạng thái của bộ lọc"""
        self.x = np.zeros((self.signal_size, 2))
        self.P = np.array([np.eye(2) for _ in range(self.signal_size)])
        self.filtered_values = np.zeros(self.signal_size)
        self.prev_measurements = np.zeros(self.signal_size)
        self.current_process_variance = np.full(self.signal_size, self.base_process_variance)
        self.initialized = False
# Khởi tạo đối số từ dòng lệnh
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

# Các tham số chương trình
parser.add_argument('--FrictionCoeff', help='hệ số ma sát chân được đặt', type=float, default=1.6)
parser.add_argument('--WedgeIncline', help='độ nghiêng của mặt phẳng nghiêng', type=int, default=15)
parser.add_argument('--MotorStrength', help='cường độ động cơ tối đa được áp dụng', type=float, default=7.0)

# Tham số lọc tín hiệu Kalman
parser.add_argument('--DeltaTime', help='bước thời gian giữa các mẫu', type=float, default=0.1)
parser.add_argument('--ProcessVariance', help='phương sai nhiễu quá trình (Q)', type=float, default=1.5)
parser.add_argument('--MeasurementVariance', help='phương sai nhiễu đo lường (R)', type=float, default=5.0)
args = parser.parse_args()

# Khởi tạo môi trường mô phỏng
env = spot.SpotEnv(render=True,
                    on_rack=True,
                    gait='trot',
                    default_pos=(-1, 0, 0.2))

state = env.reset()

# Khởi tạo bộ lọc Kalman
kalman_filter = ResponsiveKalmanFilter(
    signal_size=8,
    dt=args.DeltaTime,
    process_variance=args.ProcessVariance,## Tăng để bộ lọc nhạy hơn với thay đổi Nếu giá trị lớn, bộ lọc phản ứng nhanh hơn với thay đổi.Nếu giá trị nhỏ, bộ lọc sẽ mượt hơn nhưng có thể bỏ qua các thay đổi nhỏ.
    measurement_variance=args.MeasurementVariance#thaaps thif nhieu nhieux
)

# Giá trị mặc định khi không có tín hiệu
default_motor_angles = [0, 0, 0, 0, 0, 0, 0, 0]
fixed_motor_angles = default_motor_angles.copy()

# Số bước mô phỏng cho mỗi lần đọc dữ liệu
sim_steps = args.SimStep  

# Vòng lặp chính để nhập dữ liệu và điều khiển mô phỏng

    
while True:
    # Đọc dữ liệu mới
    raw_arr = read_data()
        
    if len(raw_arr) == 8:
            # Lọc tín hiệu với bộ lọc Kalman
            filtered_arr = kalman_filter.update(raw_arr)
            
            print(red(f"Raw data from STM: {raw_arr}"))
            print(cyan(f"Kalman filtered data: {filtered_arr}"))
            
            # Chuyển đổi giá trị góc đã lọc thành các góc cho động cơ
            fixed_motor_angles = [
                np.deg2rad(-(filtered_arr[1] - 115)), #12
                np.deg2rad(- filtered_arr[0] + 160), #11
                np.deg2rad(filtered_arr[2] - 100), #21
                np.deg2rad(filtered_arr[3] - 105), #22
                np.deg2rad(- filtered_arr[7] + 210), #42
                np.deg2rad(- filtered_arr[6] + 140), #41
                np.deg2rad(filtered_arr[4] - 140), #31
                np.deg2rad(filtered_arr[5] - 50) #32
        ]
            
            print(blue(f"Data to pybullet: {np.degrees(fixed_motor_angles)}"))
    elif len(raw_arr) > 0:
            print(yellow(f"Received incomplete data from STM: {raw_arr}"))      
    state, r, done, info = env.step_motorangles(fixed_motor_angles)
            
        

