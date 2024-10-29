import numpy as np
mang1="/home/trong/PycharmProjects/pythonProject/joint_angle_1.npy"
mang2="/home/trong/PycharmProjects/pythonProject/joint_angle_2.npy"
data1=np.load(mang1)
data2=np.load(mang2)
data_in_degrees = data * (180 / np.pi)
np.set_printoptions(threshold=np.inf)

print(data_in_degrees)
