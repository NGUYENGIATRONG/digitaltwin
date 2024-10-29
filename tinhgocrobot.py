import numpy as np
from utils.spot_kinematic  import SpotKinematics
mang1="/home/trong/PycharmProjects/pythonProject/joint_angle_7.npy"
mang2="/home/trong/PycharmProjects/pythonProject/joint_angle_8.npy"
theta1=np.load(mang1)
theta3=np.load(mang2)
l=0.05
l1=0.11
l2=0.2
l3=0.05
def main():
    theta2_pos_vals = []
    theta2_neg_vals = []
    theta4_pos_vals = []
    theta4_neg_vals = []
    a = l + l3 * np.cos(theta3) - l1 * np.cos(theta1)
    b = l3*np.sin(theta3) - l1 * np.sin(theta1)
    sqrt_term = np.sqrt(-(a ** 2 + b ** 2) * (a ** 2 + b ** 2 - 4 * l2 ** 2))

    # Tính theta2 cho cả dấu cộng và dấu trừ
    theta2_pos_vals = 2 * np.arctan((2 * b * l2 + sqrt_term) / (a ** 2 + 2 * a * l2 + b ** 2))
    theta2_neg_vals = 2 * np.arctan((2 * b * l2 - sqrt_term) / (a ** 2 + 2 * a * l2 + b ** 2))

    # Tính theta4 cho cả dấu cộng và dấu trừ
    theta4_pos_vals = -2 * np.arctan((2 * b * l2 + sqrt_term) / (a ** 2 - 2 * a * l2 + b ** 2))
    theta4_neg_vals = -2 * np.arctan((2 * b * l2 - sqrt_term) / (a ** 2 - 2 * a * l2 + b ** 2))
    np.save("/home/trong/PycharmProjects/pythonProject/theta2_pos_BR.npy", theta2_pos_vals)
    np.save("/home/trong/PycharmProjects/pythonProject/theta4_pos_BR.npy", theta4_pos_vals)
    return (theta2_pos_vals, theta2_neg_vals), (theta4_pos_vals, theta4_neg_vals)


if __name__ == "__main__":
    main()
