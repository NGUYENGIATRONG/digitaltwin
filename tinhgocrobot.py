import matplotlib.pyplot as plt
import numpy as np
from utils import spot_kinematic  # Giả sử bạn đã có module này

spot = spot_kinematic.SpotKinematics()
[l1, l2, l3, l4] = spot.link_parameters
base1 = spot.base_pivot1
base2 = spot.base_pivot2


def calculate_and_draw(x, y):
    # Tính toán góc động cơ từ tọa độ (x, y)
    valid, angles = spot.inverse2d(ee_pos=[x, y])

    if not valid:
        print("Điểm nằm ngoài không gian làm việc")
        return

    motor_hip = angles[0]
    motor_knee = angles[1]

    # Tính toán vị trí của các khớp để vẽ
    x1 = l1 * np.cos(motor_hip)
    y1 = l1 * np.sin(motor_hip)

    x2 = x1 + l2 * np.cos(motor_hip + angles[2])  # motor_hip + q3
    y2 = y1 + l2 * np.sin(motor_hip + angles[2])

    x3 = base2[0] + l3 * np.cos(angles[1])  # góc q2
    y3 = base2[1] + l3 * np.sin(angles[1])

    x4 = x3 + l4 * np.cos(angles[3])  # góc q4
    y4 = y3 + l4 * np.sin(angles[3])

    # Vẽ các liên kết của chân robot
    plt.figure()
    plt.plot([0, x1], [0, y1], 'ro-', label='Liên kết l1')
    plt.plot([x1, x2], [y1, y2], 'go-', label='Liên kết l2')
    plt.plot([base2[0], x3], [0, y3], 'bo-', label='Liên kết l3')
    plt.plot([x3, x4], [y3, y4], 'mo-', label='Liên kết l4')
    plt.scatter([x], [y], color='black', label='Điểm mong muốn (x, y)')

    plt.title('Cấu trúc cơ cấu chân robot')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.xlim(-0.2, 0.3)
    plt.ylim(-0.4, 0.1)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.grid(True)
    plt.show()


# Nhập tọa độ từ người dùng
x = float(input("Nhập tọa độ x của bàn chân (m): "))
y = float(input("Nhập tọa độ y của bàn chân (m): "))

# Tính toán và vẽ cơ cấu
calculate_and_draw(x, y)
