#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

Q1H = [2.374760389328003, -1.4447382132159632, 2.215402603149414, -2.33547288576235, -1.6491130034076136, -0.0003464857684534195]


# Hanoi tower locations

Q1T = np.radians([134.03, -81.09, 133.26, -139.7, -91.19, 13.77])
Q1M = np.radians([134.03, -72.56, 135.96, -150.90, -91.07, 13.75])
Q1B = np.radians([134.03, -62.19, 137.29, -162.65, -90.93, 13.76])


Q2H = np.radians([147.88, -94.61, 133.13, -125.86, -90.71, 27.66])
Q2T = np.radians([147.89, -85.98, 138.39, -139.75, -90.58, 27.62])
Q2M = np.radians([147.89, -76.79, 141.39, -151.90, -90.45, 27.60])
Q2B = np.radians([147.89, -62.37, 143.07, -166.03, -90.28, 27.67])

Q3H = np.radians([166.66, -99.57, 125.51, -113.28, -89.94, 46.48])
Q3T = np.radians([166.68, -83.29, 138.19, -142.23, -89.67, 46.37])
Q3M = np.radians([166.66, -74.42, 140.74, -153.65, -89.55, 46.36])
Q3B = np.radians([166.66, -63.87, 142.01, -165.47, -89.40, 46.36])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [[Q1H, Q1T, Q1M, Q1B],
     [Q2H, Q2T, Q2M, Q2B],
     [Q3H, Q3T, Q3M, Q3B]]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def suction_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN

############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""


def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if(abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and
                abs(thetas[5]-driver_msg.destination[5]) < 0.0005):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count > SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if(abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and
                abs(thetas[5]-driver_msg.destination[5]) < 0.0005):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count > SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height,
               end_loc, end_height):
    global Q

    # Hint: Use the Q array to map out your towers by location and "height".

    error = 0

    v = 4
    a = 4

    move_arm(pub_cmd, loop_rate, Q[start_loc][0], v, a) # Moving to start location
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], v, a) # Moving down to block
    gripper(pub_cmd, loop_rate, suction_on) # Turning suction on
    time.sleep(1.0) # Wait

    # NULL CHECK
    if(digital_in_0 == 0):
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, Q[start_loc][0], v, a)
        error = 1
        rospy.loginfo("No block detected. Halting system")
        sys.exit()
    else:
        move_arm(pub_cmd, loop_rate, Q[start_loc][0], v, a) # Moving back up
        move_arm(pub_cmd, loop_rate, Q[end_loc][0], v, a) # Move to end location
        move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], v, a) # Move down to final position
        gripper(pub_cmd, loop_rate, suction_off) # Turn suction off
        time.sleep(1.0) # Wait
        move_arm(pub_cmd, loop_rate, Q[end_loc][0], v, a) # Move back over end location

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber(
        'ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    gripper_input_message = rospy.Subscriber("ur3/gripper_input", gripper_input, suction_callback)
    ############### Your Code End Here ###############

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    # while(not input_done):
    #     input_string = input(
    #         "Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    loop_count = 1
    
    input_starting = 0
    input_start_loc = 0

    input_end = 0
    input_end_loc = 0

    # while(not input_done):
    #     input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")

# Start position

    while(not input_starting):
        input_starting_location = input("Enter starting tower. Enter 1, 2 or 3, or 0 to quit: ")
        print("Start at tower:" + input_starting_location + "\n")

        if(int(input_starting_location) == 1):
            input_starting = 1
            input_start_loc = 0
        elif (int(input_starting_location) == 2):
            input_starting = 1
            input_start_loc = 1
        elif (int(input_starting_location) == 3):
            input_starting = 1
            input_start_loc = 2
        else:
            print("Quitting")
            sys.exit()
# End position

    while(not input_end):
        input_ending_location = input("Enter ending tower. Enter 1, 2 or 3, or 0 to quit: ")
        print("End at tower:" + input_ending_location + "\n")

        if(int(input_ending_location) == 1):
            input_end = 1
            input_end_loc = 0
        elif (int(input_ending_location) == 2):
            input_end = 1
            input_end_loc = 1
        elif (int(input_ending_location) == 3):
            input_end = 1
            input_end_loc = 2
        else:
            print("Quitting")
            sys.exit()



    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        # rospy.loginfo("Sending goal 1 ...")
        # move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

        # gripper(pub_command, loop_rate, suction_on)
        # # Delay to make sure suction cup has grasped the block
        # time.sleep(1.0)

        # rospy.loginfo("Sending goal 2 ...")
        # move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

        # rospy.loginfo("Sending goal 3 ...")
        # move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)

    while(loop_count > 0):
        free_loc = (3 - (input_start_loc + input_end_loc))
        
        rospy.loginfo("Step 1")
        move_block(pub_command, loop_rate, input_start_loc, 1, input_end_loc, 3)

        rospy.loginfo("Step 2")
        move_block(pub_command, loop_rate, input_start_loc, 2, free_loc, 3)

        rospy.loginfo("Step 3")
        move_block(pub_command, loop_rate, input_end_loc, 3, free_loc, 2)

        rospy.loginfo("Step 4")
        move_block(pub_command, loop_rate, input_start_loc, 3, input_end_loc, 3)

        rospy.loginfo("Step 5")
        move_block(pub_command, loop_rate, free_loc, 2, input_start_loc, 3)

        rospy.loginfo("Step 6")
        move_block(pub_command, loop_rate, free_loc, 3, input_end_loc, 2)

        rospy.loginfo("Step 7")
        move_block(pub_command, loop_rate, input_start_loc, 3, input_end_loc, 1)
        
        loop_count = loop_count - 1
        gripper(pub_command, loop_rate, suction_off)
        
        print("Success!")




    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
