"""
Rocket 6DoF Simulator (Roll dynamics is not working now)
Edited by Shun Tamura   2017/10/16

"""
import os
import sys
import math
import numpy as np
import scipy as sp
import numpy.linalg as nplin
from scipy.integrate import odeint

import thrust as th
import dynamics as dy
import quaternion as qt
import environment as env
import post_process as post


# Define computation setting ---------------------------
dt = 0.001        # time interval
end_time = 200.0        # computation end time

# Define wind pattern setting
wind_vel_st = 1.0    # minimum wind velocity [m/s]
wind_vel_interval = 0.5    # interval of wind velocity [m/s]
vel_pat = 8    # velocity range [m/s]
dir_pat = 16    # wind direction derivation (deg = 360.0/dir_pat)

# Define launcher elevation setting
elev_st = 87.0    # Launcher elevation start angle [deg]
elev_interval = 1.0    # Launche elevation angle interval [deg]
elev_pat = 3    # Launcher elevation range

# Define Launcher other setting
rail_len = 5.0    # Launcher length[m]
rail_azi = 0.0    # Azimuth [deg] (0deg:East,  90deg:North)

# Define vector ---------------------------------------
time_vec = np.arange(0, end_time, dt)
case_total = vel_pat * dir_pat
drop_point = np.zeros([2, dir_pat+1, vel_pat])
wind_case = np.array([wind_vel_st, wind_vel_interval, vel_pat, dir_pat])


# Generate rsim object ----------------------------------------
simulation = dy.RocketSim()
post_exec =  post.PostProcess()


# User input on console -------------------------------
print("==> Select Parameter Input Mode")
print("0: Use reference setting,  1: Use user setting")
param_mode = input('>> ')
param_mode = int(param_mode)

if param_mode == 0:
    filename = 'default.json'

elif param_mode == 1:
    print("")
    print("==> Select stdin file from below lists, and input filename")
    files = os.listdir("./input/")
    print(files)

    for num in range(5):
        filename = input('>> ')
    
        if filename in files:
            break

        else:
            print("Input wrong file name. Please input again.")

else:
    print("Wrong mode input.")
    sys.exit()

# Load parameter
simulation.set_param(filename)

print("")    
print("==> Select Simulation Mode")
print("0: Detail Result,  1: Scatter Result")
sim_mode = input('>> ')
sim_mode = int(sim_mode)

print("")    
print("==> Select Trajectory Mode")
print("0: Trajectory,  1: Open one stage parachute")
op_flg = input('>> ')
op_flg = int(op_flg)

if sim_mode == 1:
    print("")    
    print("==> Select Elevation Mode")
    print("0: Signle output,  1: Full output")    
    elev_mode = input('>> ')
    elev_mode = int(elev_mode)

else:
    elev_mode = 0

if elev_mode == 0:
    print("")    
    print("==> Select Launcher Elevation (Vertical:90deg)")    
    rail_elev = input('>> ')
    rail_elev = float(rail_elev)

    elev_pat = 1

elif elev_mode == 1:
    rail_elev = elev_st
    print("")
    print("Launcher elevation : {0} - {1} deg".format(rail_elev, rail_elev + elev_pat -1.0))


rail_cond = np.array([rail_len, rail_elev, rail_azi])




# Execute Rocket simulation ------------------------------------
if sim_mode == 0:
    # Detail mode

    # Set wind condition
    print("")
    print("==> Input wind velocity [m/s]")
    wind_vel = input('>> ')
    wind_vel = float(wind_vel)
    
    print("")    
    print("==> Input wind direction [deg] (East:0deg, North:90deg)")
    wind_dir = input('>> ')
    wind_dir = float(wind_dir)
    
    wind_cond = np.array([wind_vel, wind_dir])

    # Compute ode equation
    simulation.set_environment(rail_cond, wind_cond)
    simulation.set_orbit_condition(op_flg)
    result = simulation.execute(time_vec)

    # Post process
    post_exec.set_variety(result, wind_cond)
    post_exec.plot_detail()


elif sim_mode == 1:
    # Scatter plot mode

    print("")
    print("Calculation start")

    for ie in range(elev_pat):

        rail_cond[1] = rail_elev + ie

        print("")
        print("--- Elevation = {0} deg --------------------".format(rail_cond[1]))

        for iv in range(vel_pat):

            print("--------------------------------------------")

            for id in range(dir_pat):

                wind_vel = wind_vel_st + iv * wind_vel_interval
                wind_dir = id * (360.0/dir_pat)
                wind_cond = np.array([wind_vel, wind_dir])
                print('Wind vel:{0[0]:>4} m/s,   Wind dir:{0[1]:>6} deg'.format(wind_cond))

                # Compute ode equation
                simulation.set_environment(rail_cond, wind_cond)
                simulation.set_orbit_condition(op_flg)
                result = simulation.execute(time_vec)

                # Write place of landing point
                drop_point[:, id, iv] = result[-1, 4:6]

                # Store result
                post_exec.set_variety(result, wind_cond)

        # Copy overlapped value for plotting
        drop_point[:, -1, :] = drop_point[:, 0, :]

        # Post process
        post_exec.set_map("Izu_land")
        post_exec.plot_scatter(wind_case)


    print("")
    print("Calculation end!")

