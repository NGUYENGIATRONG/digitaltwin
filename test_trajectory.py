import matplotlib.pyplot as plt
import numpy as np
from utils import spot_kinematic  # Giả sử bạn có module này như đã nêu trước đó

spot = spot_kinematic.SpotKinematics()
base1 = spot.base_pivot1
base2 = spot.base_pivot2
[l1, l2, l3, l4] = spot.link_parameters#0.11, 0.25, 0.11, 0.2
pi = np.pi

def draw(p1, p2):
    valid, [q1, q2, q3, q4] = spot.inverse2d(ee_pos=[p1, p2])
    if not valid:
        print("Điểm nằm ngoài phạm vi làm việc")
        return

    if q1 > 0:
        q1 = -2 * np.pi + q1

    # Tọa độ khớp
    x_00, y_00 = 0, 0
    x_01, y_01 = base2[0], 0#0.05 0

    x1, y1 = l1 * np.cos(q1), l1 * np.sin(q1)
    x2, y2 = x1 + l2 * np.cos(q3), y1 + l2 * np.sin(q3)
    x3, y3 = base2[0] + l3 * np.cos(q2), base2[1] + l3 * np.sin(q2)
    x4, y4 = x3 + l4 * np.cos(q4), y3 + l4 * np.sin(q4)

    # Vẽ chân robot
    plt.plot([x_00, x_01], [y_00, y_01], 'bo-')
    plt.plot([x_00, x1], [y_00, y1], 'ro-', label='l1')
    plt.plot([x1, x2], [y1, y2], 'go-', label='l2')
    plt.plot([x_01, x3], [y_01, y3], 'co-', label='l3')
    plt.plot([x3, x4], [y3, y4], 'mo-', label='l4')
    plt.plot(rotated_x, rotated_y, color='blue', label='Quỹ đạo elip xoay 30°')  # Vẽ elip xoay

    # Cài đặt đồ thị
    plt.title('Quỹ đạo chân robot với elip xoay 30 độ')
    plt.xlabel('X-axis (m)')
    plt.ylabel('Y-axis (m)')
    plt.xlim(-0.15, 0.2)
    plt.ylim(-0.35, 0.01)
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.pause(0.000000001)
    plt.clf()

# Tạo hình elip ban đầu
no_of_points = 100
step_length = 0.1
radius = step_length / 2
theta = 0
y_center = -0.26
foot_clearance = 0.05
x_shift = 0.01
y_shift = 0
count = 5

# Danh sách lưu trữ các điểm của elip
x = []
y = []

# Góc xoay (30 độ)
angle_deg = 30
angle_rad = np.radians(angle_deg)

# Ma trận xoay
rotation_matrix = np.array([
    [np.cos(angle_rad), -np.sin(angle_rad)],
    [np.sin(angle_rad), np.cos(angle_rad)]
])

# Tạo và xoay các điểm của elip
rotated_x = []
rotated_y = []
for _ in range(count * 80):
    leg_theta = (theta / (2 * no_of_points)) * 2 * np.pi

    t = -radius * np.cos(leg_theta) + x_shift#dao động từ -radius đến radius
    x.append(t)
    if leg_theta > np.pi:
        flag = 0
    else:
        flag = 1
    z = foot_clearance * np.sin(leg_theta) * flag + y_center + y_shift
    #foot_clearance độ cao tối đa mà chân robot có thể nâng lên khi di chuyển. Nó xác định khoảng cách nâng chân khỏi mặt đất.
    y.append(z)

    # Xoay điểm (t, z) bằng ma trận xoay
    rotated_point = np.dot(rotation_matrix, np.array([t, z]))
    rotated_x.append(rotated_point[0])
    rotated_y.append(rotated_point[1])

    # Vẽ quỹ đạo robot
    draw(rotated_point[0], rotated_point[1])

    theta += 2.5
    theta = np.fmod(theta, 2 * no_of_points)

# Vẽ hình elip đã xoay cuối cùng
plt.plot(rotated_x, rotated_y, color='blue', label='Elip xoay 30°')
plt.title('Quỹ đạo chân robot với elip xoay 30 độ')
plt.xlabel('X-axis (m)')
plt.ylabel('Y-axis (m)')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.show()
