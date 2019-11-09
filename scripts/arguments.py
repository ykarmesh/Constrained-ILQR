import argparse

def add_arguments(parser):
    # Planning Parameters

    # iLQR Parameters

    # Cost Parameters

    #Contstraints Parameters

    #Ego Vehicle Parameters
    parser.add_argument('--wheelbase', type=float, default=3.00, help="Ego Vehicle's wheelbase")
    parser.add_argument('--steering_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Steering control input limits (min,max)")
    parser.add_argument('--throttle_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Throttle control input limits (min,max)")

    # Obstacle Parameters
