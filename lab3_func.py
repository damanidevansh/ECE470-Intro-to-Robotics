#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 390],
	       		  [0, 0, -1, 401],
				  [1, 0, 0, 215.5],
				  [0, 0, 0, 1]])

	S = np.array([[0, 0, 1, 150, 150, 0],
	              [0, 1, 0, -162, 0, -150],
				  [0, 1, 0, -162, 0, 94],
				  [0, 1, 0, -162, 0, 307],
				  [1, 0, 0, 0, 162, -260],
				  [0, 1 ,0 , -162, 0, 390]])
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")
	

	# =================== Your code starts here ====================#
	Get_MS()
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

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
