import numpy as np


def find_initial_coordinates(xnew, ynew, angle_deg=30):
    # Chuyển đổi góc từ độ sang radian
    angle_rad = np.radians(angle_deg)

    # Ma trận xoay ngược (30 độ)
    rotation_matrix_inverse = np.array([
        [np.cos(angle_rad), np.sin(angle_rad)],
        [-np.sin(angle_rad), np.cos(angle_rad)]
    ])

    # Tọa độ sau khi xoay
    new_coords = np.array([xnew, ynew])

    # Tính toán tọa độ ban đầu
    initial_coords = np.dot(rotation_matrix_inverse, new_coords)
    return initial_coords[0], initial_coords[1]


# Nhập tọa độ mới từ bàn phím

