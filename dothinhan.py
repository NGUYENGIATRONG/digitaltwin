import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV và đặt tên cột thủ công
file_path = '/home/giatrong/PycharmProjects/pythonProject/data_send.csv'
df = pd.read_csv(file_path, header=None)  # Không có tiêu đề, nên dùng header=None

# Đặt tên cho các cột (tương ứng với các góc động cơ)
df.columns = ['Motor_1', 'Motor_2', 'Motor_3', 'Motor_4', 'Motor_5', 'Motor_6', 'Motor_7', 'Motor_8']

# Thiết lập biểu đồ
plt.figure(figsize=(20, 10))
time_steps = df.index * 0.005
# Vẽ từng đồ thị cho mỗi động cơ
for i, motor in enumerate(df.columns):
    plt.subplot(2, 4, i + 1)  # Tạo lưới 2x4 cho 8 đồ thị
    plt.plot(time_steps, df[motor], label=f'{motor}', marker='.')
    plt.title(f'Angle Change of {motor}')
    plt.xlabel('Time Step')
    plt.ylabel('Angle (degrees)')
    plt.grid(True)
    plt.legend()
    print(df.index)

# Hiển thị biểu đồ
plt.tight_layout()
plt.show()
