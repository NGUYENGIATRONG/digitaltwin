import simulation.spot_pybullet_env as spot
import tkinter as tk
import numpy as np
import pybullet as p
import threading
import time

# Các góc động cơ cố định cho tư thế đứng
STANDING_MOTOR_ANGLES = [
    np.radians(146.12159007 - 20), 
    np.radians(40 - (180 - 61.16792781)), 
    np.radians(146.12159007 - 20), 
    np.radians(40 - (180 - 61.16792781)),
    np.radians(146.12159007 - 20), 
    np.radians(40 - (180 - 61.16792781)), 
    np.radians(146.12159007 - 20), 
    np.radians(40 - (180 - 61.16792781))
]

# Hành động di chuyển
FORWARD_ACTION = [0.03, 0.03, 0.03, 0.03]
BACKWARD_ACTION = [-0.03, -0.03, -0.03, -0.03]

# Biến toàn cục để điều khiển robot
robot_action = "STAND"  # Có thể là "STAND", "FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"
terminate_simulation = False

# Hằng số tốc độ xử lý
SIMULATION_SLEEP_TIME = 0.005  # Giảm thời gian ngủ để tăng tốc độ xử lý
MAX_SIMULATION_STEPS = 20000  # Tăng số bước mô phỏng

# Hàm thực hiện mô phỏng trong một thread riêng
def run_simulation():
    global robot_action, terminate_simulation
    
    print("Khởi tạo môi trường mô phỏng...")
    try:
        # Khởi tạo môi trường
        env = spot.SpotEnv(render=True, on_rack=False, gait='trot')
        
        # Reset môi trường
        state = env.reset()
        
        print("Môi trường đã được khởi tạo. Sử dụng nút để điều khiển robot.")
        
        # Vòng lặp chính
        t_r = 0
        steps = 0
        theta = 0  # Tham số chu kỳ cho quỹ đạo
        
        while not terminate_simulation and steps < MAX_SIMULATION_STEPS:
            # Cập nhật theta (tham số chu kỳ)
            theta += 7.5  # Đây là tốc độ cập nhật theta, có thể điều chỉnh
            theta = theta % 200  # Giữ theta trong phạm vi 0-199
            
            # Quyết định hành động dựa trên biến robot_action
            if robot_action == "FORWARD":
                # Di chuyển tiến
                state, r, done, _ = env.step(FORWARD_ACTION)
                if steps % 100 == 0:
                    print("Robot đang di chuyển tiến...")
            elif robot_action == "BACKWARD":
                # Di chuyển lùi
                state, r, done, _ = env.step(BACKWARD_ACTION)
                if steps % 100 == 0:
                    print("Robot đang di chuyển lùi...")
            elif robot_action == "TURN_LEFT":
                # Xoay trái - sử dụng mảng step_length với giá trị âm cho chân fr và bl
                turn_left_action = [0.03, -0.03, 0.03, -0.03]  # fl, fr, bl, br
                state, r, done, _ = env.step(turn_left_action)
                if steps % 100 == 0:
                    print("Robot đang xoay trái...")
            elif robot_action == "TURN_RIGHT":
                # Xoay phải - sử dụng mảng step_length với giá trị âm cho chân fl và br
                turn_right_action = [-0.03, 0.03, -0.03, 0.03]  # fl, fr, bl, br
                state, r, done, _ = env.step(turn_right_action)
                if steps % 100 == 0:
                    print("Robot đang xoay phải...")
            else:  # "STAND" hoặc bất kỳ hành động không xác định nào
                # Giữ robot đứng yên
                state, r, done, _ = env.step_motorangles(STANDING_MOTOR_ANGLES)
                if steps % 100 == 0:
                    print("Robot đứng yên.")
            
            # Cập nhật phần thưởng tích lũy
            t_r += r
            
            # Điều chỉnh góc camera
            env.pybullet_client.resetDebugVisualizerCamera(
                0.95, 0, -0, env.get_base_pos_and_orientation()[0])
            
            # Tạm nghỉ một chút
            time.sleep(SIMULATION_SLEEP_TIME)
            steps += 1
        
        print("Mô phỏng kết thúc. Tổng reward:", t_r)
        
    except Exception as e:
        print("Lỗi trong mô phỏng:", str(e))
    finally:
        # Ngắt kết nối PyBullet
        try:
            p.disconnect()
            print("PyBullet đã được ngắt kết nối.")
        except:
            pass

# Tạo giao diện điều khiển
def create_gui():
    global robot_action, terminate_simulation
    
    root = tk.Tk()
    root.title("Điều khiển Robot Spot")
    root.geometry("500x500")
    
    # Thiết lập màu sắc
    bg_color = "#f0f0f0"
    button_color = "#4CAF50"
    button_text_color = "white"
    button_active_bg = "#45a049"
    button_width = 15
    button_height = 2
    button_font = ("Arial", 14, "bold")
    
    root.configure(bg=bg_color)
    
    # Bảng thông tin
    info_frame = tk.Frame(root, bg=bg_color)
    info_frame.pack(pady=10)
    
    info_label = tk.Label(info_frame, text="ĐIỀU KHIỂN ROBOT SPOT", 
                         font=("Arial", 20, "bold"), bg=bg_color)
    info_label.pack()
    
    status_var = tk.StringVar()
    status_var.set("Trạng thái: Đứng yên")
    status_label = tk.Label(info_frame, textvariable=status_var, 
                           font=("Arial", 12), bg=bg_color)
    status_label.pack(pady=5)
    
    # Nút điều khiển
    control_frame = tk.Frame(root, bg=bg_color)
    control_frame.pack(pady=20)
    
    # Tạo grid cho các nút
    button_frame = tk.Frame(control_frame, bg=bg_color)
    button_frame.pack()
    
    # Hàm xử lý các nút điều khiển
    def set_action(action):
        global robot_action
        robot_action = action
        
        if action == "FORWARD":
            status_var.set("Trạng thái: Đang di chuyển tiến")
        elif action == "BACKWARD":
            status_var.set("Trạng thái: Đang di chuyển lùi")
        elif action == "TURN_LEFT":
            status_var.set("Trạng thái: Đang xoay trái")
        elif action == "TURN_RIGHT":
            status_var.set("Trạng thái: Đang xoay phải")
        else:  # "STAND"
            status_var.set("Trạng thái: Đứng yên")
    
    # Hàm xử lý khi nhả nút
    def release_button():
        global robot_action
        robot_action = "STAND"
        status_var.set("Trạng thái: Đứng yên")
    
    # Nút di chuyển tiến
    forward_button = tk.Button(button_frame, text="DI CHUYỂN TIẾN", font=button_font, 
                            bg=button_color, fg=button_text_color, 
                            activebackground=button_active_bg,
                            width=button_width, height=button_height)
    forward_button.grid(row=0, column=1, padx=10, pady=10)
    
    # Nút di chuyển lùi
    backward_button = tk.Button(button_frame, text="DI CHUYỂN LÙI", font=button_font, 
                             bg=button_color, fg=button_text_color, 
                             activebackground=button_active_bg,
                             width=button_width, height=button_height)
    backward_button.grid(row=2, column=1, padx=10, pady=10)
    
    # Nút xoay trái
    turn_left_button = tk.Button(button_frame, text="XOAY TRÁI", font=button_font, 
                              bg=button_color, fg=button_text_color, 
                              activebackground=button_active_bg,
                              width=button_width, height=button_height)
    turn_left_button.grid(row=1, column=0, padx=10, pady=10)
    
    # Nút xoay phải
    turn_right_button = tk.Button(button_frame, text="XOAY PHẢI", font=button_font, 
                               bg=button_color, fg=button_text_color, 
                               activebackground=button_active_bg,
                               width=button_width, height=button_height)
    turn_right_button.grid(row=1, column=2, padx=10, pady=10)
    
    # Nút thoát
    exit_button = tk.Button(button_frame, text="THOÁT", font=button_font,
                          bg="red", fg=button_text_color,
                          activebackground="#d32f2f",
                          width=button_width, command=lambda: [terminate_simulation_and_exit()])
    exit_button.grid(row=3, column=1, padx=10, pady=20)
    
    # Sự kiện cho các nút
    forward_button.bind("<ButtonPress>", lambda event: set_action("FORWARD"))
    forward_button.bind("<ButtonRelease>", lambda event: release_button())
    
    backward_button.bind("<ButtonPress>", lambda event: set_action("BACKWARD"))
    backward_button.bind("<ButtonRelease>", lambda event: release_button())
    
    turn_left_button.bind("<ButtonPress>", lambda event: set_action("TURN_LEFT"))
    turn_left_button.bind("<ButtonRelease>", lambda event: release_button())
    
    turn_right_button.bind("<ButtonPress>", lambda event: set_action("TURN_RIGHT"))
    turn_right_button.bind("<ButtonRelease>", lambda event: release_button())
    
    # Hàm thoát chương trình
    def terminate_simulation_and_exit():
        global terminate_simulation
        terminate_simulation = True
        root.destroy()
    
    # Hàm khi đóng cửa sổ
    def on_closing():
        terminate_simulation_and_exit()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Chạy giao diện
    root.mainloop()

if __name__ == "__main__":
    # Khởi tạo thread mô phỏng
    sim_thread = threading.Thread(target=run_simulation)
    sim_thread.daemon = True
    sim_thread.start()
    
    # Tạo giao diện điều khiển
    create_gui()