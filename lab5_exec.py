#!/usr/bin/env python3

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
xw_yw_Green = []
xw_yw_Orange = []


pos_green1 = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
pos_green2 = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

pos_orange1 = [135.3*PI/180.0, -33*PI/180.0, 63.88*PI/180.0, -117.92*PI/180.0, -90*PI/180.0, 51.11*PI/180.0] 
pos_orange2 = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

xw_yw_zw_Green_dest1=[0.20, -0.15, 0.036]
xw_yw_zw_Green_dest2=[0.20, -0.10, 0.036]

xw_yw_zw_Orange_dest1=[0.30, -0.15, 0.036]
xw_yw_zw_Orange_dest2=[0.30, -0.10, 0.036]

# Any other global variable you want to define
# Hints: where to put the blocks?


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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


"""
Function to control the suction cup on/off
"""
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

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
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

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # global variable1
    # global dest_xw_yw_Orange1
    # global variable2

    # start_xw_yw_high = [start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] +0.1]
    # target_xw_yw_high = [target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2] +0.1]
    print('start_xw_yw_zw is', start_xw_yw_zw)
    start_angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
    target_angles = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)

    start_angles_up = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.03, 0)
    target_angles_up = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.03, 0)


    move_arm(pub_cmd, loop_rate, start_angles_up, vel, accel)
    move_arm(pub_cmd, loop_rate, start_angles, vel, accel)
    gripper(pub_cmd, loop_rate, True)
    time.sleep(1.0)

    if digital_in_0 == 1:
        rospy.loginfo('suck success')
        error = 0
    else: 
        error = 1
        gripper(pub_cmd, loop_rate, False)
        rospy.loginfo("suck failed")
        sys.exit()

    move_arm(pub_cmd, loop_rate, start_angles_up, vel, accel)
    move_arm(pub_cmd, loop_rate, target_angles_up, vel, accel)
    move_arm(pub_cmd, loop_rate, target_angles, vel, accel)

    gripper(pub_cmd, loop_rate, False)
    rospy.loginfo("Block has been moved!! yay!")

    # ========================= Student's code ends here ===========================
    error = 0
    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_Green # store found green blocks in this list
        global xw_yw_Orange # store found yellow blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_Green & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_Green or xw_yw_Y.

        # Remember, xw_yw_Green & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_Green = blob_search(cv_image, "green")
        xw_yw_Orange = blob_search(cv_image, "orange")


"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_Orange
    global xw_yw_Green

    # global variable1
    global dest_xw_yw_Orange1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_Green, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    print("green1 is at ", type(xw_yw_Green), xw_yw_Green[0][0])
    xw_yw_zw_Green1 = [xw_yw_Green[0][0], xw_yw_Green[0][1], 0.035]
    print("green2 is at ", type(xw_yw_Green), xw_yw_Green[1][0])
    xw_yw_zw_Green2 = [xw_yw_Green[1][0], xw_yw_Green[1][1], 0.035]
    
    print("orange1 is at ", type(xw_yw_Orange), xw_yw_Orange[0][0])
    xw_yw_zw_Orange1 = [xw_yw_Orange[0][0], xw_yw_Orange[0][1], 0.035]
    print("orange2 is at ", type(xw_yw_Orange), xw_yw_Orange[1][0])
    xw_yw_zw_Orange2 = [xw_yw_Orange[1][0], xw_yw_Orange[1][1], 0.035]

    print(xw_yw_zw_Green1)
    move_block(pub_command, loop_rate, xw_yw_zw_Green1, xw_yw_zw_Green_dest1, vel, accel)
    print(xw_yw_zw_Green2)
    move_block(pub_command, loop_rate, xw_yw_zw_Green2, xw_yw_zw_Green_dest2, vel, accel)
    
    print(xw_yw_zw_Orange1)
    move_block(pub_command, loop_rate, xw_yw_zw_Orange1, xw_yw_zw_Orange_dest1, vel, accel)
    print(xw_yw_zw_Orange2)
    move_block(pub_command, loop_rate, xw_yw_zw_Orange2, xw_yw_zw_Orange_dest2, vel, accel)

    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
