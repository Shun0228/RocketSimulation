
def rocket_param():
    # Set rocket parameter (default setting)
    rocket_parameter = {
        'len': 1.50,    # length of rocket [m]
        'diam': 0.13,    # diameter of body [m]
        'mass_i': 6.00,    # initial mass [kg]
        'mass_f': 5.50,    # final mass [kg]
        'CGlen_i': 0.79,    # initial CG place from nose [m]
        'CGlen_f': 0.75,    # final CG place from nose [m]
        'CPlen': 0.97,    # CP place from nose [m]
        'Iyz_i': 1.1,    # initial inertia moment of pitching [kg*m^2]
        'Iyz_f': 1.0,    # final inertia moment of pitching [kg*m^2]
        'Ix_i': 0.02,    # initial inertia moment of rolling [kg*m^2]
        'Ix_f': 0.01,    # final inertia moment of rolling [kg*m^2]
        'vel_1st': 10.0,   # terminal velocity of 1st parachute [m/s]
        'op_flg': 0,    # parachute open flag (0:trajectory)
        'vel_2nd': 0.0,    # terminal velocity of 2nd parachute [m/s]
        'Cd': 0.50,    # drag coefficient (M = 0.30)
        'Cna': 8.00,    # normal coefficient (M = 0.30)
        'Cmq': -2.0,    # pitch & yaw damping moment coefficient (M = 0.30)
        'op_type_1st':0,    # 0:detect-peak, 1:fixed-time
        'op_type_2nd':0,    # 0:fixed-height, 1:fixed-time
        'op_time_1st': 5.0,    # parachute open time [sec]
        'op_time_2nd': 8.0,    # parachute open time [sec]
        'en_flg': 0,    # 0:Use reference, 1:Use exp value
        'motor_name': "Hypertek_J250"    # HyperTEK engine type
        }

    return rocket_parameter


def env_param():

    env_parameter = {
        'place': 0,    # Launch Place 0: Izu(Land), 1: Izu(Sea)
        'rail_len': 5.0,    # Izu:5m, Noshiro:10m
        'rail_elev': 89.0,    # Launcher angle (Vertical:90deg)
        'rail_azi': 0.0,    # East:0deg, North:90deg, West:180deg, South:270deg
        'wind_flg': 0,    # 0:View detail, 1:scatter plot
        'wind_ref': 3.0,    # Base wind velocity [m/s]
        'wind_dir': 180.0,    # Wind direction (East:0deg, North:90deg)
        'dt': 0.01,    # Time interval [sec]
        'end_time': 100.0    # Max calculation time [sec]
        }

    return env_parameter