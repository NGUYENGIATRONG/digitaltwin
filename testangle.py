import pybullet as p

physicsClient = p.connect(p.GUI)  # Kết nối với PyBullet (GUI mode)
print("Nhấn phím để kiểm tra mã phím (ESC để thoát).")

while True:
    keys = p.getKeyboardEvents()
    if keys:
        if 65297 in keys:
            print("65297")  # In ra mã phím và trạng thái
        if 27 in keys:  # Nhấn ESC để thoát
            break
