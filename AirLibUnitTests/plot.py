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

    plotResults(data)

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
    ground_truth_x =    data[i].to_numpy(float)
    i+=1
    ground_truth_y =    data[i].to_numpy(float)
    i+=1
    ground_truth_z =    data[i].to_numpy(float)
    i+=1
    ground_truth_q0 =   data[i].to_numpy(float)
    i+=1
    ground_truth_q1 =   data[i].to_numpy(float)
    i+=1
    ground_truth_q2 =   data[i].to_numpy(float)
    i+=1
    ground_truth_q3 =   data[i].to_numpy(float)
    i+=1
    ground_truth_pitch= data[i].to_numpy(float)*180/np.pi
    i+=1
    ground_truth_roll = data[i].to_numpy(float)*180/np.pi
    i+=1
    ground_truth_yaw =  data[i].to_numpy(float)*180/np.pi
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
    bias_accel_x =      data[i].to_numpy(float)
    i+=1
    bias_accel_y =      data[i].to_numpy(float)
    i+=1
    bias_accel_z =      data[i].to_numpy(float)
    i+=1
    bias_gyro_x =       data[i].to_numpy(float)
    i+=1
    bias_gyro_y =       data[i].to_numpy(float)
    i+=1
    bias_gyro_z =       data[i].to_numpy(float)
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

    # variance of imu bias
    cov_b_f_x =             (data[i].to_numpy(float))
    i+=1
    cov_b_f_y =             (data[i].to_numpy(float))
    i+=1
    cov_b_f_z =             (data[i].to_numpy(float))
    i+=1

    # variance of gyro bias
    cov_b_omega_x =             (data[i].to_numpy(float))
    i+=1
    cov_b_omega_y =             (data[i].to_numpy(float))
    i+=1
    cov_b_omega_z =             (data[i].to_numpy(float))
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
    err_estimated_x = ground_truth_x - estimated_x
    err_estimated_y = ground_truth_y - estimated_y
    err_estimated_z = ground_truth_z - estimated_z

    # angle error
    err_estimated_pitch = ground_truth_pitch - estimated_pitch
    err_estimated_roll = ground_truth_roll - estimated_roll
    err_estimated_yaw = ground_truth_yaw - estimated_yaw

    # orientation error
    error_q0 = ground_truth_q0 - estimated_q0
    error_q1 = ground_truth_q1 - estimated_q1
    error_q2 = ground_truth_q2 - estimated_q2
    error_q3 = ground_truth_q3 - estimated_q3

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
    err_gps_x = ground_truth_x - gps_pos_x 
    err_gps_y = ground_truth_y - gps_pos_y 
    err_gps_z = ground_truth_z - gps_pos_z 
    err_gps_vel_x = true_lin_vel_x - gps_vel_x 
    err_gps_vel_y = true_lin_vel_y - gps_vel_y 
    err_gps_vel_z = true_lin_vel_z - gps_vel_z 
    err_baro_alt = -1*ground_truth_z - baro_alt

    # print(gps_vel_z)

    start_index = np.where(np.int_(timestamp) == 5)[0][0]
    end_index = np.where(np.int_(timestamp) == 6)[0][0]

    # print("True pitch rate 1: ", true_gyro_x[start_index])
    # print("True pitch rate 2: ", true_gyro_x[end_index])

    # print("True pitch angle 1: ", ground_truth_roll[start_index])
    # print("Estimated pitch angle 1: ", estimated_roll[start_index])

    # print("True pitch angle 2: ", ground_truth_roll[end_index])
    # print("Estimated pitch angle 2: ", estimated_roll[end_index])

    # print("Time difference: ", timestamp[end_index] - timestamp[start_index])
    print("Expected delta pitch angle: ", (true_gyro_x[start_index]+true_gyro_x[end_index])*0.5*(timestamp[end_index] - timestamp[start_index]))
    print("AirSim delta pitch angle: ", (ground_truth_roll[end_index] - ground_truth_roll[start_index]))
    print("Our delta pitch angle: ", (estimated_roll[end_index] - estimated_roll[start_index]))

    start_index = np.where(np.int_(timestamp) == 6)[0][0]
    end_index = np.where(np.int_(timestamp) == 7)[0][0]
    print("Expected delta pitch angle: ", (true_gyro_x[start_index]+true_gyro_x[end_index])*0.5*(timestamp[end_index] - timestamp[start_index]))
    print("AirSim delta pitch angle: ", (ground_truth_roll[end_index] - ground_truth_roll[start_index]))
    print("Our delta pitch angle: ", (estimated_roll[end_index] - estimated_roll[start_index]))

    # k = np.array([[1, 2, 3],[4, 5, 6],[7, 8, 9]])
    # print(k)
    # print(k[2,1])

    theta_error = np.zeros(len(timestamp))
    phi_error = np.zeros(len(timestamp))
    psi_error = np.zeros(len(timestamp))
    # print(psi_error)

    for i in range(0, len(timestamp)):
        ground = M_OB(ground_truth_q0[i],
                      ground_truth_q1[i],
                      ground_truth_q2[i],
                      ground_truth_q3[i])
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
    fig, ax = plt.subplots(25, 1)

    # begin plotting
    i=0
    ax[i].plot(timestamp, accel_x, linestyle='dotted', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, accel_y, linestyle='solid', color='C3', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, accel_z, linestyle='dotted', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('accel (m/s2)')
    # ax[i].set_title('Measurements')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-15, 5])
    ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, gyro_x, linestyle='solid', color='C1', label="roll")
    # ax[i].legend()
    # ax[i].plot(timestamp, gyro_y, linestyle='solid', color='C2', label="pitch")
    # ax[i].legend()
    # ax[i].plot(timestamp, gyro_z, linestyle='dotted', color='C5', label="yaw")
    # ax[i].legend()
    # ax[i].set_ylabel('gyro error(deg/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-0.01, 0.01])
    # ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, err_baro_alt, linestyle='solid', color='C1')
    # ax[i].legend()
    # ax[i].set_ylabel('baro alt err(m)')
    # ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, gyro_x, linestyle='dotted', color='C1', label="roll")
    ax[i].legend()
    ax[i].plot(timestamp, gyro_y, linestyle='solid', color='C2', label="pitch")
    ax[i].legend()
    ax[i].plot(timestamp, gyro_z, linestyle='dotted', color='C5', label="yaw")
    ax[i].legend()
    ax[i].set_ylabel('gyro rates (deg/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_gps_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('gps position err(m)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, err_gps_vel_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_vel_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, err_gps_vel_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('gps velocity err(m/s)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, mag_flux_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, mag_flux_y, linestyle='solid', color='C3', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, mag_flux_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('Magnetic flux measurement (Gauss)')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, ground_truth_pitch, linestyle='solid', color='C1', label="ground_truth_pitch")
    ax[i].legend(loc='upper left')
    ax[i].plot(timestamp, estimated_pitch, linestyle='dotted', color='C6', label="estimated_pitch", lw='1.2')
    ax[i].legend(loc='upper left')
    ax[i].plot(timestamp, ground_truth_roll, linestyle='solid', color='C2', label="ground_truth_roll")
    ax[i].legend(loc='upper left')
    ax[i].plot(timestamp, estimated_roll, linestyle='dotted', color='C7', label="estimated_roll", lw='1.2')
    ax[i].legend(loc='upper left')
    ax[i].set_ylabel('angles(deg)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 40])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, ground_truth_yaw, linestyle='solid', color='C5', label="ground_truth_yaw")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_yaw, linestyle='dotted', color='C8', label="estimated_yaw")
    ax[i].legend()
    ax[i].set_ylabel('yaw(deg)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-2, 2])
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

    # i+=1
    # ax[i].plot(timestamp, theta_error, linestyle='solid', color='C1', label="pitch")
    # ax[i].legend()
    # ax[i].plot(timestamp, phi_error, linestyle='solid', color='C2', label="roll")
    # ax[i].legend()
    # ax[i].plot(timestamp, psi_error, linestyle='dotted', color='C5', label="yaw")
    # ax[i].legend()
    # ax[i].set_ylabel('angles error using Direc Cosine(deg)')
    # # ax[i].set_xlim([5, 6])
    # # ax[i].set_ylim([-0.0001, 0.0001])
    # ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, err_estimated_pitch, linestyle='solid', color='C1', label="pitch")
    # ax[i].legend()
    # ax[i].plot(timestamp, err_estimated_roll, linestyle='solid', color='C2', label="roll")
    # ax[i].legend()
    # ax[i].plot(timestamp, err_estimated_yaw, linestyle='dotted', color='C5', label="yaw")
    # ax[i].legend()
    # ax[i].set_ylabel('angles error(deg)')
    # ax[i].set_xlim([25, 27])
    # ax[i].set_ylim([-0.00001, 0.00007])
    # ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, err_estimated_pitch, linestyle='solid', color='C1', label="pitch")
    # ax[i].legend()
    # ax[i].plot(timestamp, err_estimated_roll, linestyle='solid', color='C2', label="roll")
    # ax[i].legend()
    # ax[i].plot(timestamp, err_estimated_yaw, linestyle='dotted', color='C5', label="yaw")
    # ax[i].legend()
    # ax[i].set_ylabel('angles error(deg)')
    # ax[i].set_xlim([27, 29])
    # # ax[i].set_ylim([-0.0001, 0.0001])
    # ax[i].grid(True)



    i+=1
    ax[i].plot(timestamp, estimated_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, estimated_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('estimated position(m)')
    # ax[i].set_xlim([10, 15])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, ground_truth_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, ground_truth_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, ground_truth_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('true position(m)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
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

    # i+=1
    # ax[i].plot(timestamp, ground_truth_y, linestyle='solid', color='C1', label="ground_truth")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_y, linestyle='solid', color='C2', label="estimated")
    # ax[i].legend()
    # # ax[i].plot(timestamp, gps_y, linestyle='dotted', color='C5', label="gps")
    # # ax[i].legend()
    # ax[i].set_ylabel('y_position (m)')
    # ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, ground_truth_z, linestyle='solid', color='C1', label="ground_truth")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_z, linestyle='solid', color='C2', label="estimated", lw='0.5')
    # ax[i].legend()
    # # ax[i].plot(timestamp, gps_z, linestyle='dotted', color='C5', label="gps")
    # # ax[i].legend()
    # ax[i].set_ylabel('z_position (m)')
    # ax[i].grid(True)



    i+=1
    ax[i].plot(timestamp, esti_lin_vel_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, esti_lin_vel_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, esti_lin_vel_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('estimated lin vel(m/s)')
    # ax[i].set_xlim([10, 15])
    # ax[i].set_ylim([-1, 10])
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, true_lin_vel_x, linestyle='solid', color='C1', label="x")
    ax[i].legend()
    ax[i].plot(timestamp, true_lin_vel_y, linestyle='solid', color='C2', label="y")
    ax[i].legend()
    ax[i].plot(timestamp, true_lin_vel_z, linestyle='solid', color='C5', label="z")
    ax[i].legend()
    ax[i].set_ylabel('true lin vel(m/s)')
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
    ax[i].plot(timestamp, bias_accel_x, linestyle='solid', color='C1', label="accel_x")
    ax[i].legend()
    ax[i].plot(timestamp, bias_accel_y, linestyle='solid', color='C2', label="accel_y")
    ax[i].legend()
    ax[i].plot(timestamp, bias_accel_z, linestyle='solid', color='C3', label="accel_z")
    ax[i].legend()
    ax[i].set_ylabel('biases')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, bias_gyro_x, linestyle='solid', color='C4', label="gyro_x")
    ax[i].legend()
    ax[i].plot(timestamp, bias_gyro_y, linestyle='solid', color='C5', label="gyro_y")
    ax[i].legend()
    ax[i].plot(timestamp, bias_gyro_z, linestyle='solid', color='C6', label="gyro_z")
    ax[i].legend()
    ax[i].plot(timestamp, bias_baro, linestyle='solid', color='C7', label="baro")
    ax[i].legend()
    ax[i].set_ylabel('biases')
    ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, ground_truth_roll, linestyle='solid', color='C1', label="ground_truth")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_roll, linestyle='dotted', color='black', label="estimated")
    # ax[i].legend()
    # ax[i].set_ylabel('roll')
    # ax[i].grid(True)

    # i+=1
    # ax[i].plot(timestamp, ground_truth_yaw, linestyle='solid', color='C1', label="ground_truth")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_yaw, linestyle='dotted', color='black', label="estimated")
    # ax[i].legend()
    # ax[i].set_ylabel('yaw')
    # ax[i].grid(True)

    
    # i+=1
    # ax[i].plot(timestamp, ground_truth_q0, linestyle='solid', color='C1', label="ground_truth_q0")
    # ax[i].legend()
    # ax[i].set_ylabel('ground_truth quaternions')
    # ax[i].grid(True)
    
    # i+=1
    # ax[i].plot(timestamp, ground_truth_q1, linestyle='dotted', color='C2', label="ground_truth_q1")
    # ax[i].legend()
    # ax[i].plot(timestamp, ground_truth_q2, linestyle='solid', color='C4', label="ground_truth_q2")
    # ax[i].legend()
    # ax[i].plot(timestamp, ground_truth_q3, linestyle='dotted', color='C5', label="ground_truth_q3")
    # ax[i].legend()
    # ax[i].set_ylabel('ground_truth quaternions')
    # ax[i].grid(True)
    
    # i+=1
    # ax[i].plot(timestamp, estimated_q0, linestyle='solid', color='C1', label="estimated_q0")
    # ax[i].legend()
    # ax[i].set_ylabel('estimated quaternions')
    # ax[i].grid(True)
    
    # i+=1
    # ax[i].plot(timestamp, estimated_q1, linestyle='dotted', color='C2', label="estimated_q1")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_q2, linestyle='solid', color='C4', label="estimated_q2")
    # ax[i].legend()
    # ax[i].plot(timestamp, estimated_q3, linestyle='dotted', color='C5', label="estimated_q3")
    # ax[i].legend()
    # ax[i].set_ylabel('estimated quaternions')
    # ax[i].grid(True)
    
    # i+=1
    # ax[i].plot(timestamp, error_q0, linestyle='solid', color='C1', label="error_q0")
    # ax[i].legend()
    # ax[i].set_ylabel('error quaternions')
    # ax[i].grid(True)
    
    # i+=1
    # ax[i].plot(timestamp, error_q1, linestyle='dotted', color='C2', label="error_q1")
    # ax[i].legend()
    # ax[i].plot(timestamp, error_q2, linestyle='solid', color='C4', label="error_q2")
    # ax[i].legend()
    # ax[i].plot(timestamp, error_q3, linestyle='dotted', color='C5', label="error_q3")
    # ax[i].legend()
    # ax[i].set_ylabel('error quaternions')
    # ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, cov_x, linestyle='solid', color='C1', label="cov_x")
    ax[i].legend()
    ax[i].plot(timestamp, cov_y, linestyle='dotted', color='black', label="cov_y")
    ax[i].legend()
    ax[i].plot(timestamp, cov_z, linestyle='solid', color='C3', label="cov_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('cov_position')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, cov_u, linestyle='solid', color='C1', label="cov_u")
    ax[i].legend()
    ax[i].plot(timestamp, cov_v, linestyle='dotted', color='black', label="cov_v")
    ax[i].legend()
    ax[i].plot(timestamp, cov_w, linestyle='solid', color='C3', label="cov_w")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('cov_linear_velocity')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, cov_q0, linestyle='solid', color='C1', label="cov_q0")
    ax[i].legend()
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('cov_orientation')
    ax[i].grid(True)

    i+=1
    ax[i].plot(timestamp, cov_q1, linestyle='solid', color='C2', label="cov_q1")
    ax[i].legend()
    ax[i].plot(timestamp, cov_q2, linestyle='dotted', color='C3', label="cov_q2")
    ax[i].legend()
    ax[i].plot(timestamp, cov_q3, linestyle='dotted', color='C4', label="cov_q3")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('cov_orientation')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, cov_b_f_x, linestyle='solid', color='C1', label="cov_b_f_x")
    ax[i].legend()
    ax[i].plot(timestamp, cov_b_f_y, linestyle='dotted', color='black', label="cov_b_f_y")
    ax[i].legend()
    ax[i].plot(timestamp, cov_b_f_z, linestyle='solid', color='C3', label="cov_b_f_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('cov_imu_bias')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, cov_b_omega_x, linestyle='solid', color='C1', label="cov_b_omega_x")
    ax[i].legend()
    ax[i].plot(timestamp, cov_b_omega_y, linestyle='dotted', color='black', label="cov_b_omega_y")
    ax[i].legend()
    ax[i].plot(timestamp, cov_b_omega_z, linestyle='solid', color='C3', label="cov_b_omega_z")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('cov_gyro_bias')
    ax[i].grid(True)
    
    i+=1
    ax[i].plot(timestamp, cov_b_baro, linestyle='solid', color='C1', label="cov_b_baro")
    ax[i].legend()
    ax[i].set_yscale("log")
    ax[i].set_ylabel('cov_baro_bias')
    ax[i].grid(True)

    fig.savefig("figure.pdf")
    fig.show()

if __name__ == "__main__":
    """ This is executed when run from the command line """
    main()
