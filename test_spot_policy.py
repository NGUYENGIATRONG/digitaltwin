# Import spotenv
import simulation.spot_pybullet_env as spot
import argparse
from fabulous.color import blue, green, red, bold
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Các tham số của chương trình
    parser.add_argument('--PolicyDir', help='directory of the policy to be tested', type=str, default='23.04.1.j')
    parser.add_argument('--FrictionCoeff', help='foot friction value to be set', type=float, default=1.6)
    parser.add_argument('--WedgeIncline', help='wedge incline degree of the wedge', type=int, default=15)
    parser.add_argument('--WedgeOrientation', help='wedge orientation degree of the wedge', type=float, default=0)
    parser.add_argument('--MotorStrength', help='maximum motor Strength to be applied', type=float, default=7.0)
    parser.add_argument('--RandomTest', help='flag to sample test values randomly ', type=bool, default=False)
    parser.add_argument('--seed', help='seed for the random sampling', type=float, default=100)
    parser.add_argument('--EpisodeLength', help='number of gait steps of a episode', type=int, default=30000)
    parser.add_argument('--PerturbForce', help='perturbation force to applied perpendicular to the heading direction of the robot', type=float, default=0.0)
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

    # Khởi tạo môi trường mô phỏng
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
        print(bold(blue("\nTest without data\n")))

    print(
        bold(blue("\nTest Parameters:\n")),
        green('\nWedge Inclination:'), red(env.incline_deg),
        green('\nWedge Orientation:'), red(np.degrees(env.incline_ori)),
        green('\nCoeff. of friction:'), red(env.friction),
        green('\nMotor saturation torque:'), red(env.clips)
    )

    # Simulation starts
    t_r = 0
    # Thêm biến lưu lực ngẫu nhiên và bước đếm
    step_counter = 0
    random_force = [0, 0]  # Lực ngẫu nhiên ban đầu là 0
    force_interval = 1000  # Mỗi 30000 bước thì thay đổi lực ngẫu nhiên

    for i_step in range(args.EpisodeLength):
        action = policy.dot(state)
        state, r, _, angle = env.step(action)
        t_r += r
        if step_counter % force_interval == 0:
            # Tạo lực ngẫu nhiên mới sau mỗi 10 bước
            random_force = [np.random.uniform(-50, 50), np.random.uniform(-50, 50)]
        step_counter +=1
        # Áp dụng lực ngoại lực 50N hướng xuống robot (áp dụng vào link chính)
        env.apply_ext_force(random_force[0], random_force[1], visulaize=True)  # link_index=0 giả sử là thân chính của robot

        # Đặt lại camera sau mỗi bước
        # env.pybullet_client.resetDebugVisualizerCamera(0.95, 0, -0, env.get_base_pos_and_orientation()[0])

    print("Total reward: " + str(t_r) + ' -> ' + str(args.PolicyDir))
