import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Hàm cập nhật đồ thị cho mỗi bước mô phỏng
def update_joint_plot(i):
    action = policy.dot(state_deque[-1])
    state, r, _, angle = env.step(action)
    t_r_deque.append(t_r_deque[-1] + r)
    state_deque.append(state)

    # Lấy thông tin góc của các khớp (motor angles) từ robot
    motor_angles = env.get_motor_angles()

    # Lưu các góc khớp vào danh sách
    for j in range(8):
        joint_angles[j].append(np.degrees(motor_angles[j]))

    # Cập nhật dữ liệu đồ thị
    for j in range(8):
        joint_lines[j].set_data(range(len(joint_angles[j])), joint_angles[j])

    # Cập nhật trục đồ thị
    for ax in axes:
        ax.relim()
        ax.autoscale_view()

    return joint_lines

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--PolicyDir', help='directory of the policy to be tested', type=str, default='23.04.1.j')
    parser.add_argument('--FrictionCoeff', help='foot friction value to be set', type=float, default=1.6)
    parser.add_argument('--WedgeIncline', help='wedge incline degree of the wedge', type=int, default=15)
    parser.add_argument('--WedgeOrientation', help='wedge orientation degree of the wedge', type=float, default=0)
    parser.add_argument('--MotorStrength', help='maximum motor Strength to be applied', type=float, default=7.0)
    parser.add_argument('--RandomTest', help='flag to sample test values randomly ', type=bool, default=False)
    parser.add_argument('--seed', help='seed for the random sampling', type=float, default=100)
    parser.add_argument('--EpisodeLength', help='number of gait steps of a episode', type=int, default=30000)
    parser.add_argument('--PerturbForce',
                        help='perturbation force to applied perpendicular to the heading direction of the robot',
                        type=float, default=0.0)
    parser.add_argument('--Downhill', help='should robot walk downhill?', type=bool, default=False)
    parser.add_argument('--Stairs', help='test on staircase', type=bool, default=False)
    parser.add_argument('--AddImuNoise', help='flag to add noise in IMU readings', type=bool, default=False)
    parser.add_argument('--Test', help='Test without data', type=bool, default=False)

    args = parser.parse_args()
    policy = np.load("experiments/" + args.PolicyDir + "/iterations/best_policy.npy")

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
                       on_rack=False,
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

    if args.Test:
        print(bold(blue("\nTest with out data\n")))

    print(
        bold(blue("\nTest Parameters:\n")),
        green('\nWedge Inclination:'), red(env.incline_deg),
        green('\nWedge Orientation:'), red(np.degrees(env.incline_ori)),
        green('\nCoeff. of friction:'), red(env.friction),
        green('\nMotor saturation torque:'), red(env.clips)
    )

    # Để lưu trữ các góc của 8 khớp (joint angles)
    joint_angles = [[] for _ in range(8)]
    t_r_deque = deque([0])
    state_deque = deque([state])

    # Cài đặt đồ thị
    fig, axes = plt.subplots(8, 1, figsize=(10, 16))

    joint_lines = []
    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray']
    for i in range(8):
        line, = axes[i].plot([], [], label=f'Joint {i+1} Angle', color=colors[i])
        joint_lines.append(line)
        axes[i].set_ylabel(f'Joint {i+1} (degrees)')
        axes[i].legend()

    axes[7].set_xlabel('Time Steps')

    plt.tight_layout()

    # Tạo animation cho quá trình cập nhật
    ani = FuncAnimation(fig, update_joint_plot, frames=args.EpisodeLength, interval=50, blit=True)

    # Hiển thị đồ thị động
    plt.show()
