#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab4_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
    # =================== Your code starts here ====================#
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))

    M = np.array([[0, -1, 0, .390],
                  [0, 0, -1, .401],
                  [1, 0, 0, .2155],
                  [0, 0, 0, 1]])

    S = np.array([[0, 0, 1, .150, .150, 0],
                  [0, 1, 0, -.162, 0, -.150],
                  [0, 1, 0, -.162, 0, .094],
                  [0, 1, 0, -.162, 0, .307],
                  [1, 0, 0, 0, .162, -.260],
                  [0, 1 ,0 , -.162, 0, .390]])
    # ==============================================================#
    return M, S

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    # Initialize the return_value
    return_value = [None, None, None, None, None, None]

    #print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    # Rearranging Screw Axis
    d_S = {}
    S = S.T
    for i in range(6):
        d_S[i] = np.array([[0, -S[2,i], S[1,i], S[3,i]],
                        [S[2,i], 0, -S[0,i], S[4,i]],
                        [-S[1,i], S[0,i], 0, S[5,i]],
                        [0, 0, 0, 0]])


# Forward Kinematics Eqn
    Tb = {}
    theta= np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    for i in range(6):
        Tb[i] = expm(d_S[i]*theta[i])

    T = Tb[0]@Tb[1]@Tb[2]@Tb[3]@Tb[4]@Tb[5]@M


    # ==============================================================#

    print("pose of the tool is \n", str(T) + "\n")

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # =================== Your code starts here ====================#
    L1 = 0.152
    L2 = 0.120
    L3 = 0.244
    L4 = 0.093
    L5 = 0.213
    L6 = 0.083
    L7 = 0.083
    L8 = 0.082
    L9 = 0.0535
    L10 = 0.059

    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0
    theta4 = 0.0
    theta5 = -np.pi/2
    theta6 = 0.0
    yaw_radians = yaw_WgripDegree * np.pi/180

    # =================== Your code starts here ====================#

    x_grip = xWgrip + 0.15
    y_grip = yWgrip - 0.15
    z_grip = zWgrip - 0.01


    x_cen = x_grip-L9*math.cos(yaw_radians)
    y_cen = y_grip-L9*math.sin(yaw_radians)
    z_cen = z_grip


    theta1 = math.atan2(y_cen, x_cen) - math.asin((L2-L4+L6)/((x_cen**2+y_cen**2)**0.5))

    theta6 = np.pi/2 - yaw_radians + theta1


    x_3end = x_cen - L7*math.cos(theta1) + (L6+0.027)*math.sin(theta1)
    y_3end = y_cen - L7*math.sin(theta1) - (L6+0.027)*math.cos(theta1)
    z_3end = z_cen + L10 + L8


    La = (x_3end**2+y_3end**2)**0.5
    Lb = z_3end-L1
    theta3 = np.pi - math.acos((L3**2+L5**2-Lb**2-La**2) / (2*L3*L5))

    alpha = math.acos((La**2 + Lb**2 + L1**2 - (z_3end**2+La**2))/(2*(La**2+Lb**2)**0.5*L1))
    beta  = math.acos((La**2 + Lb**2 + L3**2 - L5**2)/(2*(La**2+Lb**2)**0.5*L3))

 
    theta2 = np.pi/2 - alpha - beta
    theta4 = -(theta2 + theta3)


    theta1_d = theta1*180/np.pi
    theta2_d = theta2*180/np.pi
    theta3_d = theta3*180/np.pi
    theta4_d = theta4*180/np.pi
    theta5_d = theta5*180/np.pi
    theta6_d = theta6*180/np.pi

    print("Theta is", theta1_d, theta2_d, theta3_d, theta4_d, theta5_d, theta6_d)
    # ==============================================================#
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
