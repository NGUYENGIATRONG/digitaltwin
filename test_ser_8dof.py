import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold, yellow
import numpy as np
from collections import deque
import serial
import logging
import struct
import serial.tools.list_ports
import time

# Cấu hình logging
logging.basicConfig(level=logging.INFO)

def find_serial_port():
    # Lấy danh sách tất cả các cổng nối tiếp có sẵn
    ports = serial.tools.list_ports.comports()

    # Kiểm tra các cổng nối tiếp và tìm cổng phù hợp
    for port in ports:
        logging.info(f"Found port: {port.device} - {port.description}")
        # Kiểm tra điều kiện cổng nối tiếp
        if "USB" in port.description or "ttyUSB" in port.device or "ttyACM" in port.device:
            logging.info(f"Matched port: {port.device}")
            return port.device

    # Trả về None nếu không tìm thấy cổng phù hợp
    return None

def send_data(data):
    if ser and ser.is_open:
        try:
            data_send = bytearray([255, 254]) + bytearray(data) #+ bytearray([254])
            ser.write(data_send)
            logging.info(f"Send angles:{blue(data_send)}")
            logging.info(f"Send angles:{blue(list(data_send))}")
            logging.info(f"Length of data send: {red(len(data_send))}")
        except serial.SerialException as e:
            logging.error(f"Error writing to serial port: {e}")
    else:
        logging.error("Serial is not available .....")

def read_data():
    """Đọc dữ liệu từ cổng nối tiếp và xác nhận đúng định dạng 255...254"""
    buffer = []
    flag = False  # Biến kiểm tra nếu bắt đầu và kết thúc đúng

    while ser.in_waiting > 0:
        byte = ord(ser.read(1))  
        if byte == 255:  
            buffer = [byte]  
            flag = True  
        elif byte == 254 and flag:  
            buffer.append(byte) 
   
            if len(buffer[1:-1]) == 8: 
                logging.info(f"Received data between 255 and 254: {yellow(buffer[1:-1])}")
                buffer = []  
                flag = False 
            else:
                logging.error("Incorrect data")
        elif flag:
           
            buffer.append(byte)


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


step_length =[0.08,0.08,0.08,0.08]
# Hàm cập nhật dữ liệu góc khớp cho mỗi bước mô phỏng
def update_joint_data():
    # action = policy.dot(state_deque[-1])  # Tính toán hành động từ policy và trạng thái hiện tại
    state, r, _, angle = env.step(step_length)  # Bước mô phỏng
    t_r_deque.append(t_r_deque[-1] + r)  # Cập nhật tổng reward
    state_deque.append(state)  # Cập nhật trạng thái

    # Lấy thông tin góc của các khớp từ robot
    motor_angles = env.get_motor_angles()
    motor_angles[0] = np.degrees(motor_angles[0]) + 20    # 0 2 4 6 hip
    motor_angles[2] = np.degrees(motor_angles[2]) + 20
    motor_angles[4] = np.degrees(motor_angles[4]) + 20
    motor_angles[6] = np.degrees(motor_angles[6]) + 20
    motor_angles[1] = np.degrees(motor_angles[1]) - 40 + 180  #1 3 5 7 knee
    motor_angles[3] = np.degrees(motor_angles[3]) - 40 + 180
    motor_angles[5] = np.degrees(motor_angles[5]) - 40 + 180
    motor_angles[7] = np.degrees(motor_angles[7]) - 40 + 180
    # Lưu các góc khớp vào danh sách (chuyển từ radian sang độ)
    print(bold("Motor angles: "), green(motor_angles))
    motor_angles_byte_arr = [round(i) for i in motor_angles]
    # print("\n max: ", max(motor_angles_byte_arr))
    # print("\n min: ", min(motor_angles_byte_arr))
    send_data(motor_angles_byte_arr)
   
    # read_data()

    for j in range(8):
        joint_angles[j].append(motor_angles[j])  # Lưu trực tiếp nếu cần radian021

if __name__ == '__main__':
    if ON == 1:
        parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        parser.add_argument('--PolicyDir', help='directory of the policy to be tested', type=str, default='23.04.1.j')
        parser.add_argument('--FrictionCoeff', help='foot friction value to be set', type=float, default=1.6)
        parser.add_argument('--WedgeIncline', help='wedge incline degree of the wedge', type=int, default=15)
        parser.add_argument('--WedgeOrientation', help='wedge orientation degree of the wedge', type=float, default=0)
        parser.add_argument('--MotorStrength', help='maximum motor Strength to be applied', type=float, default=7.0)
        parser.add_argument('--RandomTest', help='flag to sample test values randomly', type=bool, default=False)
        parser.add_argument('--seed', help='seed for the random sampling', type=float, default=100)
        parser.add_argument('--EpisodeLength', help='number of gait steps of an episode', type=int, default=10000)
        parser.add_argument('--PerturbForce',
                            help='perturbation force to applied perpendicular to the heading direction of the robot',
                            type=float, default=0.0)
        parser.add_argument('--Downhill', help='should robot walk downhill?', type=bool, default=False)
        parser.add_argument('--Stairs', help='test on staircase', type=bool, default=False)
        parser.add_argument('--AddImuNoise', help='flag to add noise in IMU readings', type=bool, default=False)
        parser.add_argument('--Test', help='Test without data', type=bool, default=False)

        args = parser.parse_args()
        policy = np.load("experiments/" + args.PolicyDir + "/iterations/zeros12x11.npy")

        WedgePresent = True

        if args.WedgeIncline == 0 or args.Stairs:
            WedgePresent = False
        elif args.WedgeIncline < 0:
            args.WedgeIncline = -1 * args.WedgeIncline
            args.Downhill = True

        env = spot.SpotEnv(render=True,
                        wedge=WedgePresent,
                        stairs=args.Stairs,
                        downhill=args.Downhill,
                        seed_value=args.seed,
                        on_rack=True,
                        gait='trot',
                        imu_noise=args.AddImuNoise,
                        test=args.Test,
                        default_pos=(-1, 0, 0.2))

        if args.RandomTest:
            env.set_randomization(default=False)
        else:
            env.incline_deg = args.WedgeIncline
            env.incline_ori = np.radians(args.WedgeOrientation)
            env.set_foot_friction(args.FrictionCoeff)
            env.clips = args.MotorStrength
            env.perturb_steps = 300
            env.y_f = args.PerturbForce

        state = env.reset()

        # if args.Test:
        #     print(bold(blue("\nTest with out data\n")))

        # print(
        #     bold(blue("\nTest Parameters:\n")),
        #     green('\nWedge Inclination:'), red(env.incline_deg),
        #     green('\nWedge Orientation:'), red(np.degrees(env.incline_ori)),
        #     green('\nCoeff. of friction:'), red(env.friction),
        #     green('\nMotor saturation torque:'), red(env.clips)
        # )

        # Để lưu trữ các góc của 8 khớp (joint angles)
        joint_angles = [[] for _ in range(8)]
        t_r_deque = deque([0])  # Tổng phần thưởng
        state_deque = deque([state])  # Trạng thái ban đầu

        # Chạy mô phỏng và cập nhật góc khớp
        for i in range(args.EpisodeLength):
            update_joint_data()  # Cập nhật dữ liệu cho mỗi bước mô phỏng


        # Lưu từng góc khớp vào một file riêng biệt
        # for i in range(8):
        #     np.save(f"joint_angle_{i + 1}.npy", joint_angles[i])
        #     print(bold(blue(f"\nJoint {i + 1} angles saved to 'joint_angle_{i + 1}.npy'.\n")))
