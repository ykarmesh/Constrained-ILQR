import argparse

def add_arguments(parser):
    # ___________________ Carla Parameters ___________________ #
    parser.add_argument('--add_npc_agents', action="store_true", default=False, help='Should there be NPC agents in the simulator')

    parser.add_argument('--number_of_npc', type= int, default=10, help='Number of NPC vehicles')
    parser.add_argument('--camera_width', type= int, default=800, help='Width of the image rendered by the cameras')
    parser.add_argument('--camera_height', type= int, default=800, help='Height of the image rendered by the cameras')

    parser.add_argument('--debug_simulator', action="store_true", default=False, help='Use visualizations in simulator for debugging')


    # ___________________ Planning Parameters ___________________ #

    # ___________________ iLQR Parameters ___________________ #
    parser.add_argument('--timestep', type=float, default=0.01, help='Timestep at which forward and backward pass are done by iLQR')
    parser.add_argument('--horizon', type=int, default=100, help='Planning horizon for iLQR in num of steps (T=horizon*timesteps)')

    # ___________________ Cost Parameters ___________________ #

    parser.add_argument('--w_acc', type=float, default=1.00, help="Acceleration cost")
    parser.add_argument('--w_yawrate', type=float, default=1.00, help="Yaw rate cost")

    parser.add_argument('--w_pos', type=float, default=1.00, help="Path deviation cost")
    parser.add_argument('--w_vel', type=float, default=1.00, help="Velocity cost")

    parser.add_argument('--q1_acc', type=float, default=1.00, help="Barrier function q1, acc")
    parser.add_argument('--q2_acc', type=float, default=1.00, help="Barrier function q2, acc")

    parser.add_argument('--q1_yawrate', type=float, default=1.00, help="Barrier function q1, yawrate")
    parser.add_argument('--q2_yawrate', type=float, default=1.00, help="Barrier function q2, yawrate")

    parser.add_argument('--q1_obs', type=float, default=1.00, help="Barrier function q1, obs")
    parser.add_argument('--q2_obs', type=float, default=1.00, help="Barrier function q2, obs")


    # ___________________ Constraint Parameters ___________________ #
    parser.add_argument('--acc_limits', nargs="*", type=float, default=[-5.5, 3.0], help="Acceleration limits for the ego vehicle (min,max)")
    parser.add_argument('--steer_angle_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Steering Angle limits (rads) for the ego vehicle (min,max)")


    # ___________________ Ego Vehicle Parameters ___________________ #
    parser.add_argument('--wheelbase', type=float, default=3.00, help="Ego Vehicle's wheelbase")
    parser.add_argument('--steering_control_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Steering control input limits (min,max)")
    parser.add_argument('--throttle_control_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Throttle control input limits (min,max)")

    # ___________________ Obstacle Parameters ___________________ #
