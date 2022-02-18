#!/usr/bin python3
"""
Module Docstring
"""

__author__ = "Suman Subedi"
__version__ = "0.1.0"
__license__ = "MIT"

import argparse
import math
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch

def draw_error_band(ax, x, y, err, **kwargs):
    # Calculate normals via centered finite differences (except the first point
    # which uses a forward difference and the last point which uses a backward
    # difference).

    # dx = np.concatenate([[x[1] - x[0]], x[2:] - x[:-2], [x[-1] - x[-2]]])
    # dy = np.concatenate([[y[1] - y[0]], y[2:] - y[:-2], [y[-1] - y[-2]]])
    # l = np.hypot(dx, dy)
    # nx = dy / l
    # ny = -dx / l

    # end points of errors
    # xp = x + nx * err
    # yp = y + ny * err
    # xn = x - nx * err
    # yn = y - ny * err
    xp = x + err
    yp = y + err
    xn = x - err
    yn = y - err

    vertices = np.block([[xp, xn[::-1]],
                         [yp, yn[::-1]]]).T
    codes = np.full(len(vertices), Path.LINETO)
    codes[0] = Path.MOVETO
    # codes[len(xp)] = Path.MOVETO
    path = Path(vertices, codes)
    ax.add_patch(PathPatch(path, **kwargs))

def main():
    """ Main entry point of the app """
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", help="filename of the txt file")
    args = parser.parse_args()

    if args.filename == None:
        data = pd.read_csv('data.csv', delimiter='\t', header=None)
    else:
        data = pd.read_csv(args.filename, delimiter='\t', header=None)

    # print(data.head())

    # plotResults(data.head(100*300))
    plotResults(data)

    # N = 400
    # t = np.linspace(0, 2 * np.pi, N)
    # r = 0.5 + np.cos(t)
    # x, y = r * np.cos(t), r * np.sin(t)

    # fig, ax = plt.subplots()
    # # ax.plot(x, y, "k")
    # # ax.set(aspect=1)

    # # axs = (plt.figure(constrained_layout=True)
    # #    .subplots(1, 2, sharex=True, sharey=True))
    # # errs = [
    # #     (axs[0], "constant error", 0.05),
    # #     (axs[1], "variable error", 0.05 * np.sin(2 * t) ** 2 + 0.04),
    # # ]

    # ax.set(title=[], aspect=1, xticks=[], yticks=[])
    # ax.plot(x, y, "k")
    # draw_error_band(ax, x, y, 0.05 * np.sin(2 * t) ** 2 + 0.04,
    #                 facecolor=f"C{1}", edgecolor="none", alpha=.3)

    # plt.show()
    # fig.savefig("plot.pdf")

def M_OB(q0, q1, q2, q3) -> np.array:
    """ Direction cosine matrix """
    return np.array([[q0*q0 + q1*q1 - q2*q2 - q3*q3, 2.0*(q1*q2 - q0*q3), 2.0*(q0*q2 + q1*q3)],
                     [2.0*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2.0*(q2*q3 - q0*q1)],
                     [2.0*(q1*q3 - q0*q2), 2.0*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3]])

def plotResults(data):
    i=0
    # extract time stamps
    timestamp =         data[i].to_numpy()
    i+=1
    start_time = timestamp[0]
    timestamp = (timestamp - start_time)*0.001

    # extract truth sensor measurements
    true_accel_x =      data[i].to_numpy(float)
    i+=1
    true_accel_y =      data[i].to_numpy(float)
    i+=1
    true_accel_z =      data[i].to_numpy(float)
    i+=1
    true_gyro_x =       data[i].to_numpy(float)*180/np.pi
    i+=1
    true_gyro_y =       data[i].to_numpy(float)*180/np.pi
    i+=1
    true_gyro_z =       data[i].to_numpy(float)*180/np.pi
    i+=1

    # extract sensor measurements
    accel_x =           data[i].to_numpy(float)
    i+=1
    accel_y =           data[i].to_numpy(float)
    i+=1
    accel_z =           data[i].to_numpy(float)
    i+=1
    gyro_x =            data[i].to_numpy(float)*180/np.pi
    i+=1
    gyro_y =            data[i].to_numpy(float)*180/np.pi
    i+=1
    gyro_z =            data[i].to_numpy(float)*180/np.pi
    i+=1
    gps_pos_x =         data[i].to_numpy(float)
    i+=1
    gps_pos_y =         data[i].to_numpy(float)
    i+=1
    gps_pos_z =         data[i].to_numpy(float)
    i+=1
    gps_vel_x =         data[i].to_numpy(float)
    i+=1
    gps_vel_y =         data[i].to_numpy(float)
    i+=1
    gps_vel_z =         data[i].to_numpy(float)
    i+=1
    baro_alt  =         data[i].to_numpy(float)
    i+=1
    mag_flux_x  =         data[i].to_numpy(float)
    i+=1
    mag_flux_y  =         data[i].to_numpy(float)
    i+=1
    mag_flux_z  =         data[i].to_numpy(float)
    i+=1

    # extract ground truth states
    true_x =    data[i].to_numpy(float)
    i+=1
    true_y =    data[i].to_numpy(float)
    i+=1
    true_z =    data[i].to_numpy(float)
    i+=1
    true_q0 =   data[i].to_numpy(float)
    i+=1
    true_q1 =   data[i].to_numpy(float)
    i+=1
    true_q2 =   data[i].to_numpy(float)
    i+=1
    true_q3 =   data[i].to_numpy(float)
    i+=1
    true_pitch= data[i].to_numpy(float)*180/np.pi
    i+=1
    true_roll = data[i].to_numpy(float)*180/np.pi
    i+=1
    true_yaw =  data[i].to_numpy(float)*180/np.pi
    i+=1
    true_lin_vel_x =    data[i].to_numpy(float)
    i+=1
    true_lin_vel_y =    data[i].to_numpy(float)
    i+=1
    true_lin_vel_z =    data[i].to_numpy(float)
    i+=1

    # estimated ekf states
    estimated_x =       data[i].to_numpy(float)
    i+=1
    estimated_y =       data[i].to_numpy(float)
    i+=1
    estimated_z =       data[i].to_numpy(float)
    i+=1
    estimated_q0 =      data[i].to_numpy(float)
    i+=1
    estimated_q1 =      data[i].to_numpy(float)
    i+=1
    estimated_q2 =      data[i].to_numpy(float)
    i+=1
    estimated_q3 =      data[i].to_numpy(float)
    i+=1
    estimated_pitch =   data[i].to_numpy(float)*180/np.pi
    i+=1
    estimated_roll =    data[i].to_numpy(float)*180/np.pi
    i+=1
    estimated_yaw =     data[i].to_numpy(float)*180/np.pi
    i+=1
    esti_lin_vel_x =    data[i].to_numpy(float)
    i+=1
    esti_lin_vel_y =    data[i].to_numpy(float)
    i+=1
    esti_lin_vel_z =    data[i].to_numpy(float)
    i+=1
    bias_accel_x =      data[i].to_numpy(float) * 1000 / 9.80665
    i+=1
    bias_accel_y =      data[i].to_numpy(float) * 1000 / 9.80665
    i+=1
    bias_accel_z =      data[i].to_numpy(float) * 1000 / 9.80665
    i+=1
    bias_gyro_x =       data[i].to_numpy(float) * 180/np.pi
    i+=1
    bias_gyro_y =       data[i].to_numpy(float) * 180/np.pi
    i+=1
    bias_gyro_z =       data[i].to_numpy(float) * 180/np.pi
    i+=1
    bias_baro =         data[i].to_numpy(float)
    i+=1

    # variance of position
    cov_x =             (data[i].to_numpy(float))
    i+=1
    cov_y =             (data[i].to_numpy(float))
    i+=1
    cov_z =             (data[i].to_numpy(float))
    i+=1

    # variance of linear velocity
    cov_u =             (data[i].to_numpy(float))
    i+=1
    cov_v =             (data[i].to_numpy(float))
    i+=1
    cov_w =             (data[i].to_numpy(float))
    i+=1

    # extract covariance of angles
    cov_q0 =            (data[i].to_numpy(float))
    i+=1
    cov_q1 =            (data[i].to_numpy(float))
    i+=1
    cov_q2 =            (data[i].to_numpy(float))
    i+=1
    cov_q3 =            (data[i].to_numpy(float))
    i+=1
    cov_roll =          (data[i].to_numpy(float))*180*180/np.pi/np.pi
    i+=1
    cov_pitch =         (data[i].to_numpy(float))*180*180/np.pi/np.pi
    i+=1
    cov_yaw =           (data[i].to_numpy(float))*180*180/np.pi/np.pi
    i+=1

    # variance of imu bias
    cov_b_f_x =             (data[i].to_numpy(float)) * (1000 / 9.80665)* (1000 / 9.80665)
    i+=1
    cov_b_f_y =             (data[i].to_numpy(float)) * (1000 / 9.80665)* (1000 / 9.80665)
    i+=1
    cov_b_f_z =             (data[i].to_numpy(float)) * (1000 / 9.80665)* (1000 / 9.80665)
    i+=1

    # variance of gyro bias
    cov_b_omega_x =             (data[i].to_numpy(float)) * (180/np.pi)* (180/np.pi)
    i+=1
    cov_b_omega_y =             (data[i].to_numpy(float)) * (180/np.pi)* (180/np.pi)
    i+=1
    cov_b_omega_z =             (data[i].to_numpy(float)) * (180/np.pi)* (180/np.pi)
    i+=1

    # variance baro
    cov_b_baro =          data[i].to_numpy(float)
    i+=1

    # quaternion norm
    quat_norm =          data[i].to_numpy(float)
    i+=1

    # quaternion off diag cov
    cov_q0_q1 =          data[i].to_numpy(float)
    i+=1
    cov_q0_q2 =          data[i].to_numpy(float)
    i+=1
    cov_q0_q3 =          data[i].to_numpy(float)
    i+=1
    cov_q1_q2 =          data[i].to_numpy(float)
    i+=1
    cov_q1_q3 =          data[i].to_numpy(float)
    i+=1
    cov_q2_q3 =          data[i].to_numpy(float)
    i+=1

    # quaternion gyro bias cov
    cov_q0_omega_x =          data[i].to_numpy(float)
    i+=1
    cov_q0_omega_y =          data[i].to_numpy(float)
    i+=1
    cov_q0_omega_z =          data[i].to_numpy(float)
    i+=1
    cov_q1_omega_x =          data[i].to_numpy(float)
    i+=1
    cov_q1_omega_y =          data[i].to_numpy(float)
    i+=1
    cov_q1_omega_z =          data[i].to_numpy(float)
    i+=1
    cov_q2_omega_x =          data[i].to_numpy(float)
    i+=1
    cov_q2_omega_y =          data[i].to_numpy(float)
    i+=1
    cov_q2_omega_z =          data[i].to_numpy(float)
    i+=1
    cov_q3_omega_x =          data[i].to_numpy(float)
    i+=1
    cov_q3_omega_y =          data[i].to_numpy(float)
    i+=1
    cov_q3_omega_z =          data[i].to_numpy(float)
    i+=1

    # position error
    err_estimated_x = true_x - estimated_x
    err_estimated_y = true_y - estimated_y
    err_estimated_z = true_z - estimated_z

    # angle error
    err_estimated_pitch = true_pitch - estimated_pitch
    err_estimated_roll = true_roll - estimated_roll
    err_estimated_yaw = true_yaw - estimated_yaw

    # orientation error
    error_q0 = true_q0 - estimated_q0
    error_q1 = true_q1 - estimated_q1
    error_q2 = true_q2 - estimated_q2
    error_q3 = true_q3 - estimated_q3

    # orientation error


    # velocity error
    err_lin_vel_x = true_lin_vel_x - esti_lin_vel_x
    err_lin_vel_y = true_lin_vel_y - esti_lin_vel_y
    err_lin_vel_z = true_lin_vel_z - esti_lin_vel_z

    # sensor errors
    err_accel_x = true_accel_x - accel_x 
    err_accel_y = true_accel_y - accel_y 
    err_accel_z = true_accel_z - accel_z 
    err_gyro_x = true_gyro_x - gyro_x 
    err_gyro_y = true_gyro_y - gyro_y 
    err_gyro_z = true_gyro_z - gyro_z 
    err_gps_x = true_x - gps_pos_x 
    err_gps_y = true_y - gps_pos_y 
    err_gps_z = true_z - gps_pos_z 
    err_gps_vel_x = true_lin_vel_x - gps_vel_x 
    err_gps_vel_y = true_lin_vel_y - gps_vel_y 
    err_gps_vel_z = true_lin_vel_z - gps_vel_z 
    err_baro_alt = -1*true_z - baro_alt

    # print(gps_vel_z)

    start_index = np.where(np.int_(timestamp) == 5)[0][0]
    end_index = np.where(np.int_(timestamp) == 6)[0][0]

    # print("True pitch rate 1: ", true_gyro_x[start_index])
    # print("True pitch rate 2: ", true_gyro_x[end_index])

    # print("True pitch angle 1: ", true_roll[start_index])
    # print("Estimated pitch angle 1: ", estimated_roll[start_index])

    # print("True pitch angle 2: ", true_roll[end_index])
    # print("Estimated pitch angle 2: ", estimated_roll[end_index])

    # print("Time difference: ", timestamp[end_index] - timestamp[start_index])
    # print("Expected delta pitch angle: ", (true_gyro_x[start_index]+true_gyro_x[end_index])*0.5*(timestamp[end_index] - timestamp[start_index]))
    # print("AirSim delta pitch angle: ", (true_roll[end_index] - true_roll[start_index]))
    # print("Our delta pitch angle: ", (estimated_roll[end_index] - estimated_roll[start_index]))

    # start_index = np.where(np.int_(timestamp) == 6)[0][0]
    # end_index = np.where(np.int_(timestamp) == 7)[0][0]
    # print("Expected delta pitch angle: ", (true_gyro_x[start_index]+true_gyro_x[end_index])*0.5*(timestamp[end_index] - timestamp[start_index]))
    # print("AirSim delta pitch angle: ", (true_roll[end_index] - true_roll[start_index]))
    # print("Our delta pitch angle: ", (estimated_roll[end_index] - estimated_roll[start_index]))

    # k = np.array([[1, 2, 3],[4, 5, 6],[7, 8, 9]])
    # print(k)
    # print(k[2,1])

    theta_error = np.zeros(len(timestamp))
    phi_error = np.zeros(len(timestamp))
    psi_error = np.zeros(len(timestamp))
    # # print(psi_error)

    for i in range(0, len(timestamp)):
        ground = M_OB(true_q0[i],
                      true_q1[i],
                      true_q2[i],
                      true_q3[i])
        estimated = M_OB(estimated_q0[i],
                         estimated_q1[i],
                         estimated_q2[i],
                         estimated_q3[i])
        # print(ground_error)

        error_direction_cosine = ground*estimated.transpose()
        # print(error_direction_cosine[2, 0])

        #     euler = [atan2(mat(3,2),mat(3,3));
        #              -asin(mat(3,1));
        #              atan2(mat(2,1),mat(1,1))];

        phi_error[i]   = np.arctan2(error_direction_cosine[2, 1],error_direction_cosine[2, 2])*180/np.pi
        theta_error[i] = -np.arcsin(error_direction_cosine[2, 0])*180/np.pi
        psi_error[i]   = np.arctan2(error_direction_cosine[1, 0],error_direction_cosine[0, 0])*180/np.pi


    # apply style
    plt.style.use('./style.mplstyle')

    # define the plot
    fig, ax = plt.subplots(38, 1)

    # begin plotting
    i=0
    ax[i].plot(timestamp, accel_x, linestyle='solid', color='C0', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, accel_y, linestyle='dotted', color='C1', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, accel_z, linestyle='solid', color='C2', label="z")
    ax[i].legend()
    ax[i].set_ylabel('accel (m/s2)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_accel_x, linestyle='solid', color='C0', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_accel_y, linestyle='dotted', color='C1', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_accel_z, linestyle='solid', color='C2', label="z")
    ax[i].legend()
    ax[i].set_ylabel('accel err (m/s2)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, gyro_x, linestyle='solid', color='C0', label="roll")
    ax[i].legend()
    ax[i].plot(timestamp, gyro_y, linestyle='dotted', color='C1', label="pitch")
    ax[i].legend()
    ax[i].plot(timestamp, gyro_z, linestyle='solid', color='C2', label="yaw")
    ax[i].legend()
    ax[i].set_ylabel('gyro rates (deg/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_gyro_x, linestyle='solid', color='C0', label="roll")
    ax[i].legend()
    ax[i].plot(timestamp, err_gyro_y, linestyle='dotted', color='C1', label="pitch")
    ax[i].legend()
    ax[i].plot(timestamp, err_gyro_z, linestyle='solid', color='C2', label="yaw")
    ax[i].legend()
    ax[i].set_ylabel('gyro error(deg/s)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_gps_x, linestyle='solid', color='C0', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_y, linestyle='solid', color='C1', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_z, linestyle='solid', color='C2', label="z")
    ax[i].legend()
    ax[i].set_ylabel('gps position err(m)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_gps_vel_x, linestyle='solid', color='C0', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_vel_y, linestyle='solid', color='C1', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_vel_z, linestyle='solid', color='C2', label="z")
    ax[i].legend()
    ax[i].set_ylabel('gps velocity err(m/s)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, mag_flux_x, linestyle='solid', color='C0', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, mag_flux_y, linestyle='solid', color='C1', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, mag_flux_z, linestyle='solid', color='C2', label="z")
    ax[i].legend()
    ax[i].set_ylabel('Mag. flux (Gauss)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_pitch, linestyle='solid', color='k', label="true_pitch")
    ax[i].legend(loc='upper left')
    ax[i].plot(timestamp, estimated_pitch, linestyle='solid', color='C0', label="estimated_pitch")
    ax[i].legend(loc='upper left')
    ax[i].fill_between(timestamp, 
                    estimated_pitch-3*np.sqrt(cov_pitch), 
                    estimated_pitch+3*np.sqrt(cov_pitch), 
                    alpha=0.2,linewidth=0, color='C5')
    ax[i].set_ylabel('pitch(deg)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 40])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_roll, linestyle='solid', color='k', label="true_roll")
    ax[i].legend(loc='upper left')
    ax[i].plot(timestamp, estimated_roll, linestyle='solid', color='C1', label="estimated_roll")
    ax[i].legend(loc='upper left')
    ax[i].fill_between(timestamp, 
                    estimated_roll-3*np.sqrt(cov_roll), 
                    estimated_roll+3*np.sqrt(cov_roll), 
                    alpha=0.2,linewidth=0, color='C5')
    ax[i].set_ylabel('roll(deg)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 40])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_yaw, linestyle='solid', color='k', label="true_yaw")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_yaw, linestyle='solid', color='C2', label="estimated_yaw")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    estimated_yaw-3*np.sqrt(cov_yaw), 
                    estimated_yaw+3*np.sqrt(cov_yaw), 
                    alpha=0.2,linewidth=0, color='C5')
    ax[i].set_ylabel('yaw(deg)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-2, 2])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_x, linestyle='solid', color='k', label="true_x")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_x, linestyle='solid', color='C0', label="estimated_x")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        estimated_x-3*np.sqrt(cov_x), 
                        estimated_x+3*np.sqrt(cov_x), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('x position(m)')
    # ax[i].set_xlim([10, 15])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_y, linestyle='solid', color='k', label="true_y")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_y, linestyle='solid', color='C1', label="estimated_y")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        estimated_y-3*np.sqrt(cov_y), 
                        estimated_y+3*np.sqrt(cov_y), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('y position(m)')
    # ax[i].set_xlim([10, 15])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_z, linestyle='solid', color='k', label="true_z")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_z, linestyle='solid', color='C2', label="estimated_z")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        estimated_z-3*np.sqrt(cov_z), 
                        estimated_z+3*np.sqrt(cov_z), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('z position(m)')
    # ax[i].set_xlim([10, 15])
    ax[i].set_xlabel('Time (sec)')
    ax[i].grid(True)


    i+=1
    ax[i].plot(timestamp, err_estimated_pitch, linestyle='solid', color='C1', label="pitch")
    ax[i].legend()
    ax[i].plot(timestamp, err_estimated_roll, linestyle='solid', color='C2', label="roll")
    ax[i].legend()
    ax[i].set_ylabel('angles error(deg)')
    # ax[i].set_xlim([5, 6])
    # ax[i].set_ylim([-0.0001, 0.0001])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_estimated_yaw, linestyle='dotted', color='C5', label="yaw")
    ax[i].legend()
    ax[i].set_ylabel('angles error(deg)')
    # ax[i].set_xlim([5, 6])
    # ax[i].set_ylim([-0.0001, 0.0001])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, theta_error, linestyle='solid', color='C1', label="pitch")
    ax[i].legend()
    ax[i].plot(timestamp, phi_error, linestyle='solid', color='C2', label="roll")
    ax[i].legend()
    ax[i].set_ylabel('angles error dir cos(deg)')
    # ax[i].set_xlim([5, 6])
    # ax[i].set_ylim([-0.0001, 0.0001])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, psi_error, linestyle='dotted', color='C5', label="yaw")
    ax[i].legend()
    ax[i].set_ylabel('angles error dir cos(deg)')
    # ax[i].set_xlim([5, 6])
    # ax[i].set_ylim([-0.0001, 0.0001])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_estimated_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_estimated_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_estimated_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('position error(m)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-0.1, 0.1])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_lin_vel_x, linestyle='solid', color='k', label="true_x")
    ax[i].legend()
    ax[i].plot(timestamp, esti_lin_vel_x, linestyle='solid', color='C0', label="estimated_x")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        esti_lin_vel_x-3*np.sqrt(cov_u), 
                        esti_lin_vel_x+3*np.sqrt(cov_u), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('x lin vel(m/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_lin_vel_y, linestyle='solid', color='k', label="true_y")
    ax[i].legend()
    ax[i].plot(timestamp, esti_lin_vel_y, linestyle='solid', color='C1', label="estimated_y")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        esti_lin_vel_y-3*np.sqrt(cov_v), 
                        esti_lin_vel_y+3*np.sqrt(cov_v), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('y lin vel(m/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_lin_vel_z, linestyle='solid', color='k', label="true_z")
    ax[i].legend()
    ax[i].plot(timestamp, esti_lin_vel_z, linestyle='solid', color='C2', label="estimated_z")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                        esti_lin_vel_z-3*np.sqrt(cov_w), 
                        esti_lin_vel_z+3*np.sqrt(cov_w), 
                        alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('estimated lin vel(m/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_lin_vel_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_lin_vel_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_lin_vel_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('lin vel error(m/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-0.01, 0.01])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, quat_norm, linestyle='solid', color='C1', label="norm")
    ax[i].legend()
    ax[i].set_ylabel('quaternion norm')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_xlabel('Time (s)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, bias_accel_x, linestyle='solid', color='C0', label="accel_x")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_accel_x-3*np.sqrt(cov_b_f_x), 
                    bias_accel_x+3*np.sqrt(cov_b_f_x), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('x accel biases mili g')
    ax[i].grid(True)
    i+=1
    ax[i].plot(timestamp, bias_accel_y, linestyle='solid', color='C1', label="accel_y")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_accel_y-3*np.sqrt(cov_b_f_y), 
                    bias_accel_y+3*np.sqrt(cov_b_f_y), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('y accel biases mili g')
    ax[i].grid(True)
    i+=1
    ax[i].plot(timestamp, bias_accel_z, linestyle='solid', color='C2', label="accel_z")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_accel_z-3*np.sqrt(cov_b_f_z), 
                    bias_accel_z+3*np.sqrt(cov_b_f_z), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('z accel biases mili g')
    ax[i].grid(True)


    i+=1
    ax[i].plot(timestamp, bias_gyro_x, linestyle='solid', color='C0', label="gyro_x")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_gyro_x-3*np.sqrt(cov_b_omega_x), 
                    bias_gyro_x+3*np.sqrt(cov_b_omega_x), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('x gyro biases deg/s')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, bias_gyro_y, linestyle='solid', color='C1', label="gyro_y")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_gyro_y-3*np.sqrt(cov_b_omega_y), 
                    bias_gyro_y+3*np.sqrt(cov_b_omega_y), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('y gyro biases deg/s')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, bias_gyro_z, linestyle='solid', color='C2', label="gyro_z")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_gyro_z-3*np.sqrt(cov_b_omega_z), 
                    bias_gyro_z+3*np.sqrt(cov_b_omega_z), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('z gyro biases deg/s')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, bias_baro, linestyle='solid', color='C3', label="baro")
    ax[i].legend()
    ax[i].fill_between(timestamp, 
                    bias_baro-3*np.sqrt(cov_b_baro), 
                    bias_baro+3*np.sqrt(cov_b_baro), 
                    alpha=0.3,linewidth=0, color='C5')
    ax[i].set_ylabel('baro bias m')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_x), linestyle='solid', color='C0', label="std_dev_x")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_y), linestyle='dotted', color='C1', label="std_dev_y")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_z), linestyle='solid', color='C2', label="std_dev_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('std_dev_position(m)')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_u), linestyle='solid', color='C0', label="std_dev_u")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_v), linestyle='dotted', color='C1', label="std_dev_v")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_w), linestyle='solid', color='C2', label="std_dev_w")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('std_dev_lin_velocity(m/s)')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_q0), linestyle='solid', color='C3', label="std_dev_q0")
    ax[i].legend()
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('std_dev_orientation')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_q1), linestyle='solid', color='C0', label="std_dev_q1")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_q2), linestyle='dotted', color='C1', label="std_dev_q2")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_q3), linestyle='dotted', color='C2', label="std_dev_q3")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('std_dev_orientation')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_roll), linestyle='solid', color='C0', label="std_dev_roll")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_pitch), linestyle='solid', color='C1', label="std_dev_pitch")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_yaw), linestyle='dotted', color='C2', label="std_dev_yaw")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('std_dev_angles(deg)')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_b_f_x), linestyle='solid', color='C0', label="std_dev_b_f_x")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_b_f_y), linestyle='dotted', color='C1', label="std_dev_b_f_y")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_b_f_z), linestyle='solid', color='C2', label="std_dev_b_f_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('std_dev_imu_bias(milli g)')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_b_omega_x), linestyle='solid', color='C0', label="std_dev_b_omega_x")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_b_omega_y), linestyle='dotted', color='C1', label="std_dev_b_omega_y")
    ax[i].legend()
    ax[i].plot(timestamp, np.sqrt(cov_b_omega_z), linestyle='solid', color='C2', label="std_dev_b_omega_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('std_dev_gyro_bias(deg/s)')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, np.sqrt(cov_b_baro), linestyle='solid', color='C0', label="std_dev_b_baro")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('std_dev_baro_bias(m)')
    ax[i].grid(True)

    fig.savefig("figure.pdf")
    fig.show()

if __name__ == "__main__":
    """ This is executed when run from the command line """
    main()
