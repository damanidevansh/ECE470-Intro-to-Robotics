#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	# M = np.eye(4)
	# S = np.zeros((6,6))

	M = np.array([
		[0,	-1,	0,	0.390],		# -150 + 244 + 83 + 213 = 390 mm
		[0,	0,	-1,	0.401],		# 59 + 120 + 82 + 150	= 401 mm
		[1,	0,	0,	0.2155],		# 152 + 10 + 53.5		= 215.5 mm
		[0,	0,	0,	1]
    ])

 
	S = np.array([
		[0, 0, 1, 0.150,	0.150,	0		],		# 1		q = [-150,				150,			10		]
		[0, 1, 0, -0.162,	0,		-0.150	],		# 2		q = [-150,				150+120,		10+152	]
		[0, 1, 0, -0.162,	0,		0.094	],		# 3		q = [-150+244,			150+120,		10+152	]
		[0, 1, 0, -0.162,	0,		0.307	],		# 4		q = [-150+244+213,		150+120-93,		10+152	]
		[1, 0, 0, 0,		0.162,	-0.260	],		# 5		q = [-150+244+213,		150+120-93+83,	10+152	]
		[0, 1, 0, -0.162,	0,		0.390	]		# 6		q = [-150+244+213+83,	150+120-93+83,	10+152	]
    ])

	S_bracket1 = np.array([
		[0,			-S[0][2],	S[0][1],	S[0][3]],
		[S[0][2],	0,			-S[0][0],	S[0][4]],
		[-S[0][1],	S[0][0],	0,			S[0][5]],
		[0,			0,			0,			0		]
	])
	S_bracket2 = np.array([
		[0,			-S[1][2],	S[1][1],	S[1][3]],
		[S[1][2],	0,			-S[1][0],	S[1][4]],
		[-S[1][1],	S[1][0],	0,			S[1][5]],
		[0,			0,			0,			0		]
	])
	S_bracket3 = np.array([
		[0,			-S[2][2],	S[2][1],	S[2][3]],
		[S[2][2],	0,			-S[2][0],	S[2][4]],
		[-S[2][1],	S[2][0],	0,			S[2][5]],
		[0,			0,			0,			0		]
	])
	S_bracket4 = np.array([
		[0,			-S[3][2],	S[3][1],	S[3][3]],
		[S[3][2],	0,			-S[3][0],	S[3][4]],
		[-S[3][1],	S[3][0],	0,			S[3][5]],
		[0,			0,			0,			0		]
	])
	S_bracket5 = np.array([
		[0,			-S[4][2],	S[4][1],	S[4][3]],
		[S[4][2],	0,			-S[4][0],	S[4][4]],
		[-S[4][1],	S[4][0],	0,			S[4][5]],
		[0,			0,			0,			0		]
	])
	S_bracket6 = np.array([
		[0,			-S[5][2],	S[5][1],	S[5][3]],
		[S[5][2],	0,			-S[5][0],	S[5][4]],
		[-S[5][1],	S[5][0],	0,			S[5][5]],
		[0,			0,			0,			0		]
	])
	S_bracket = np.array([[S_bracket1], [S_bracket2], [S_bracket3], [S_bracket4], [S_bracket5], [S_bracket6]])



	# ==============================================================#
	return M, S_bracket


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	# theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	# T = np.eye(4)

	M, S_bracket = Get_MS()
 
	e1 = expm(S_bracket[0][0] * theta1)
	e2 = expm(S_bracket[1][0] * theta2)
	e3 = expm(S_bracket[2][0] * theta3)
	e4 = expm(S_bracket[3][0] * theta4)
	e5 = expm(S_bracket[4][0] * theta5)
	e6 = expm(S_bracket[5][0] * theta6)
 
	T  = e1 @ e2 @ e3 @ e4 @ e5 @ e6 @ M
	print (T)



	# ==============================================================#

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

# convert yaw to radians
	yaw_rad = np.deg2rad(yaw_WgripDegree)

	x_cen = (xWgrip+0.15) - (np.cos(yaw_rad) * L9)
	y_cen = (yWgrip-0.15) - (np.sin(yaw_rad) * L9)
	z_cen = (zWgrip-0.01)

# find theta 1
	theta1 = np.arctan2(y_cen, x_cen) - np.arctan2(0.11, np.sqrt( (x_cen*x_cen) + (y_cen*y_cen) - (0.11*0.11) )  )
# find theta 6
	theta6 = theta1 - yaw_rad + (np.pi/2)
# find theta 5
	theta5 = -np.pi/2

# find x_3end, y_3end, z_3end
	tempLength = L2-L4+L6
	x_3end = x_cen - (L7 * np.cos(theta1)) + (tempLength * np.sin(theta1))
	y_3end = y_cen - (L7 * np.sin(theta1)) - (tempLength * np.cos(theta1))
	z_3end = z_cen + L10 + L8

	z_3 = z_3end - L1
 
# theta3
	lengthEnd = np.sqrt( (x_3end*x_3end) + (y_3end*y_3end) + (z_3*z_3) )

	theta3 = np.pi - np.arccos(  ( (L3*L3) + (L5*L5) - (lengthEnd*lengthEnd) )/( 2*L3*L5 )  )

# theta2

	alpha = np.arcsin( (L5*np.sin(theta3))/(lengthEnd) )
	beta = np.arcsin( (z_3end - L1)/ (lengthEnd) )

	# print(alpha, beta)
	theta2 = -1 * ( alpha + beta)

# theta4
	theta4 = -(theta2 + theta3)
 
 
	print(theta1, theta2, theta3, theta4, theta5, theta6)

	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	# theta5 = 0.0
	# theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)