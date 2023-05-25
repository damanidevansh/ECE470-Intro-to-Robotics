#!/usr/bin/env python3

import sys
import copy
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *
from math import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

xyPurple       = []
xyGreen     = []
xyYellow    = []
foundYellow = []        # matched yellow with the stick
foundGreen = []         # matched yellow with the stick

saveYellow  = []
savePurple  = []
saveGreen   = []



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


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel, yaw):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    rotate = 0
    yaw1 = 0
    yaw2 = 0
    yaw3 = 0
    # ========================= Student's code starts here =========================

    if rowCounter < 3:
        offsetZ = 0.035
    elif rowCounter >=3 and rowCounter < 5:
        offsetZ = 0.03
    else:
        offsetZ = 0.0255
        
    if yaw < -100:
        yaw1 = 0
        yaw3 = -90
        yaw2 = yaw + 90
        rotate = 1
    if yaw > 120:
        yaw1 = 0
        yaw3 = 90
        yaw2 = yaw - 90
        rotate = 1
    
    yaw1 = 0
    yaw2 = yaw
    print(yaw1)
    print(yaw2)
    print(yaw2)
        

    # print('start_xw_yw_zw is', start_xw_yw_zw)
    start_angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], yaw1)
    start_angles_up = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + 0.1, yaw1)

    start_angles3 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], yaw3)
    start_angles3_up = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + 0.1, yaw3)
    
    start_angles4 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
    start_angles4_up = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + 0.1, 0)
    
    target_angles = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], yaw2)
    target_angles_up = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2] + offsetZ, yaw2)


    move_arm(pub_cmd, loop_rate, start_angles_up, 4.0, 4.0)
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
    move_arm(pub_cmd, loop_rate, start_angles_up, 4.0, 4.0)

####
    if rotate == 1:
        move_arm(pub_cmd, loop_rate, start_angles3, vel, accel)
        gripper(pub_cmd, loop_rate, False)
        move_arm(pub_cmd, loop_rate, start_angles3_up, vel, accel)  # 90 yaw
        move_arm(pub_cmd, loop_rate, start_angles4_up, vel, accel)  # 0 yaw
        
        move_arm(pub_cmd, loop_rate, start_angles4, vel, accel)
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
        move_arm(pub_cmd, loop_rate, start_angles4_up, vel, accel)
        rotate = 0
####
        
        
    
    move_arm(pub_cmd, loop_rate, target_angles_up, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, target_angles, vel, accel)

    gripper(pub_cmd, loop_rate, False)
    move_arm(pub_cmd, loop_rate, target_angles_up, vel, accel)
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

        # global xw_yw_Green # store found green blocks in this list
        # global xw_yw_Orange # store found yellow blocks in this list
        
        global xyPurple
        global xyGreen
        global xyYellow
        global xyStick
        
        global xyPurple1
        global xyGreen1
        global xyYellow1
        global xyPurple2
        global xyGreen2
        global xyYellow2
        global xyPurple3
        global xyGreen3
        global xyYellow3
        global xyPurple4
        global xyGreen4
        global xyYellow4
        global xyPurple5
        global xyGreen5
        global xyYellow5

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

#####
        xyPurple1       = blob_search(cv_image, "purple")
        # print("PURPLEEEE11", len(xyPurple1))
        xyGreen1     = blob_search(cv_image, "green")
        # print("GREENNN11", len(xyGreen1))
        xyYellow1    = blob_search(cv_image, "yellow")
        # print("YELLOWWW11", len(xyYellow1))
        
        xyPurple2       = blob_search(cv_image, "purple")
        # print("PURPLEEEE22", len(xyPurple2))
        xyGreen2     = blob_search(cv_image, "green")
        # print("GREENNN22", len(xyGreen2))
        xyYellow2    = blob_search(cv_image, "yellow")
        # print("YELLOWWW22", len(xyYellow2))
        
        xyPurple3       = blob_search(cv_image, "purple")
        # print("PURPLEEEE33", len(xyPurple3))
        xyGreen3     = blob_search(cv_image, "green")
        # print("GREENNN33", len(xyGreen3))
        xyYellow3    = blob_search(cv_image, "yellow")
        # print("YELLOWWW33", len(xyYellow3))
        
        xyPurple4       = blob_search(cv_image, "purple")
        xyGreen4     = blob_search(cv_image, "green")
        xyYellow4    = blob_search(cv_image, "yellow")

        xyPurple5       = blob_search(cv_image, "purple")
        xyGreen5     = blob_search(cv_image, "green")
        xyYellow5    = blob_search(cv_image, "yellow")


######   
                


"""
Program run from here
"""
def main():

    global go_away
    
    global xyPurple
    global xyGreen
    global xyYellow


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

    vel = 1.0
    accel = 1.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)



### ================ ================ GET BLOCK LOCATION 5 TIMES FOR REDUNTANCY ================ ================ ###
    if len(xyPurple1) > len(xyPurple2) or len(xyPurple1) > len(xyPurple3) or len(xyPurple1) > len(xyPurple4) or len(xyPurple1) > len(xyPurple5):
        xyPurple = xyPurple1
    elif len(xyPurple2) > len(xyPurple1) or len(xyPurple2) > len(xyPurple3) or len(xyPurple2) > len(xyPurple4) or len(xyPurple2) > len(xyPurple5):
        xyPurple = xyPurple2
    elif len(xyPurple3) > len(xyPurple1) or len(xyPurple3) > len(xyPurple2) or len(xyPurple3) > len(xyPurple4) or len(xyPurple3) > len(xyPurple5):
        xyPurple = xyPurple3
    elif len(xyPurple4) > len(xyPurple1) or len(xyPurple4) > len(xyPurple2) or len(xyPurple4) > len(xyPurple3) or len(xyPurple4) > len(xyPurple5):
        xyPurple = xyPurple4
    else:
        xyPurple = xyPurple5

        
    if len(xyGreen1) > len(xyGreen2) or len(xyGreen1) > len(xyGreen3) or len(xyGreen1) > len(xyGreen4) or len(xyGreen1) > len(xyGreen5):
        xyGreen = xyGreen1
    elif len(xyGreen2) > len(xyGreen1) or len(xyGreen2) > len(xyGreen3) or len(xyGreen2) > len(xyGreen4) or len(xyGreen2) > len(xyGreen5):
        xyGreen = xyGreen2
    elif len(xyGreen3) > len(xyGreen1) or len(xyGreen3) > len(xyGreen2) or len(xyGreen3) > len(xyGreen4) or len(xyGreen3) > len(xyGreen5):
        xyGreen = xyGreen3
    elif len(xyGreen4) > len(xyGreen1) or len(xyGreen4) > len(xyGreen2) or len(xyGreen4) > len(xyGreen3) or len(xyGreen4) > len(xyGreen5):
        xyGreen = xyGreen4
    else:
        xyGreen = xyGreen5
        
    if len(xyYellow1) > len(xyYellow2) or len(xyYellow1) > len(xyYellow3) or len(xyYellow1) > len(xyYellow4) or len(xyYellow1) > len(xyYellow5):
        xyYellow = xyYellow1
    elif len(xyYellow2) > len(xyYellow1) or len(xyYellow2) > len(xyYellow3) or len(xyYellow2) > len(xyYellow4) or len(xyYellow2) > len(xyYellow5):
        xyYellow = xyYellow2
    elif len(xyYellow3) > len(xyYellow1) or len(xyYellow3) > len(xyYellow2) or len(xyYellow3) > len(xyYellow4) or len(xyYellow3) > len(xyYellow5):
        xyYellow = xyYellow3
    elif len(xyYellow4) > len(xyYellow1) or len(xyYellow4) > len(xyYellow2) or len(xyYellow4) > len(xyYellow3) or len(xyYellow4) > len(xyYellow5):
        xyYellow = xyYellow4
    else:
        xyYellow = xyYellow5
        
    print("PURPLE11", len(xyPurple1))
    print("GREEN11", len(xyGreen1))
    print("YELLOW11", len(xyYellow1), '\n')
    print("PURPLE22", len(xyPurple2))
    print("GREEN22", len(xyGreen2))
    print("YELLOW22", len(xyYellow2), '\n')
    print("PURPLE33", len(xyPurple3))
    print("GREEN33", len(xyGreen3))
    print("YELLOW33", len(xyYellow3), '\n')
    print("PURPLE44", len(xyPurple4))
    print("GREEN44", len(xyGreen4))
    print("YELLOW44", len(xyYellow4), '\n')
    print("PURPLE55", len(xyPurple5))
    print("GREEN55", len(xyGreen5))
    print("YELLOW55", len(xyYellow5), '\n')


### ======END====== ======END====== GET BLOCK LOCATION 5 TIMES FOR REDUNTANCY ======END====== ======END====== ###

### ==================== ==================== NUMBER OF STICKS ==================== ==================== ###
    blockSize = 0.025
    # numSticks = 11
    
# ask user how many stick
    userInput = input("YOOOO tell me how many sticks are there: ")
    print("You entered ", int(userInput))
    numSticks = int(userInput)


### ======END====== ======END====== NUMBER OF STICKS ======END====== ======END====== ###



### ==================== ==================== SAVING BLOCK LOCATIONS FROM SCAN AFTER 5 SECONDS ==================== ==================== ###
# save block location
    saveYellow  = []
    savePurple  = []
    saveGreen   = []
    saveYellow = xyYellow
    savePurple = xyPurple
    saveGreen = xyGreen
    
    print("Yellow Count = ", len(saveYellow))
    print("saveYellow", saveYellow)
    if len(saveYellow) < numSticks:                # sanity check for missing block detection
        rospy.loginfo("MISSING YELLOW BLOCK! THAT'S NOT GOOD :(")
        print("Use Ctrl+C to exit program")
        rospy.spin()
    
    print("Green Count = ", len(saveGreen))
    print("saveGreen", saveGreen)
    if len(saveGreen) < numSticks:
        rospy.loginfo("MISSING GREEN BLOCK! THAT'S NOT GOOD :(")
        print("Use Ctrl+C to exit program")
        rospy.spin()
        
    print("Purple Count = ", len(savePurple))
    print("savePurple", savePurple)
    if len(savePurple) < numSticks:
        rospy.loginfo("MISSING PURPLE BLOCK! THAT'S NOT GOOD :(")
        print("Use Ctrl+C to exit program")
        rospy.spin()
    

### ======END====== ======END====== SAVING BLOCK LOCATIONS FROM SCAN AFTER 5 SECONDS ======END====== ======END====== ###
    

### ==================== ==================== LOCOS FOR SAVING BLOCKS FOR CORRESPONDING STICK ==================== ==================== ###
# used to determine which block belongs to which stick
    foundYellow = [[0 for i in range(3)] for j in range(numSticks)]        # matched yellow with the stick
    foundGreen = [[0 for i in range(3)] for j in range(numSticks)]         # matched yellow with the stick
    xyStickLocation = [[0 for i in range(3)] for j in range(numSticks)]    # middle of stick [x][y][yaw]
    
    # print(foundYellow)
    # print(foundGreen)
    # print(xyStickLocation)
### ======END====== ======END====== LOCOS FOR SAVING BLOCKS FOR CORRESPONDING STICK ======END====== ======END====== ###


### ==================== ==================== STARTING TARGET PLACEMENTS ==================== ==================== ###
# distance between left and right
    # distance = 0.16     # for 6 high
    distance = 0.16
# left target location
    targetxl = 0.325
    targetyl = 0
    targetzl = blockSize
# right target location
    targetxr = targetxl
    targetyr = targetyl + distance
    targetzr = blockSize
### ======END====== ======END====== STARTING TARGET PLACEMENTS ======END====== ======END====== ###
    






### ==================== ====================    FINDING STICK    ==================== ==================== ###
# go through each center block and look for nearby blocks that are attached
    radius = blockSize + 0.001
    # start at purple look for yellow and purple next to it
    for a in range(0, numSticks):
        for b in range(0, numSticks):
            # check x and y for yellow
            checkUpperX = savePurple[a][0] + radius      # x
            checkLowerX = savePurple[a][0] - radius      # x
            checkUpperY = savePurple[a][1] + radius      # y
            checkLowerY = savePurple[a][1] - radius      # y

            if checkLowerX < saveYellow[b][0] and checkUpperX > saveYellow[b][0]:
                if checkLowerY < saveYellow[b][1] and checkUpperY > saveYellow[b][1]:
                    foundYellow[a][0] = saveYellow[b][0]
                    foundYellow[a][1] = saveYellow[b][1]
                    # print("a = ", a)
                    # print("b = ", b)
                    # print("foundYellow", foundYellow[a][0], foundYellow[a][1])
            # check x and y for green
            if checkLowerX < saveGreen[b][0] and checkUpperX > saveGreen[b][0]:
                if checkLowerY < saveGreen[b][1] and checkUpperY > saveGreen[b][1]:
                    foundGreen[a][0] = saveGreen[b][0]
                    foundGreen[a][1] = saveGreen[b][1]
            
                    
                    # print("a = ", a)
                    # print("b = ", b)
                    # print("foundGreen", foundGreen[a][0], foundGreen[a][1])
        ##### end for loop b #####
        # print("made it out of loop b")
    ##### end for loop a #####
    # print("made it out of loop a")
    # print("foundGreen", len(foundGreen))
    # print("foundYellow", len(foundYellow))
    print("foundgreeen", foundGreen)
    print("foundyellow", foundYellow)

### ======END====== ======END====== ======END======    FINDING STICK    ======END====== ======END====== ======END====== ###




### ==================== ====================    TIME TO GET YAW    ==================== ==================== ###
    for c in range(0, numSticks):
        # find slope and that is yaw
            # if slope = 0; yaw = 90 degree
            # if slope = divisionbyzero; yaw = 0 degree
            # if slope = 1; yaw = 45 degree
        # print("foundgreeen", foundGreen[c][1])
        # print("foundyellow", foundYellow[c][1])
        
        if 100*(foundGreen[c][1] - foundYellow[c][1]) == 0:   # yG - yY = 0
            yaw = 90                                            # if vertical
        else:
            slopeX = (foundGreen[c][0] - foundYellow[c][0])
            slopeY = (foundGreen[c][1] - foundYellow[c][1])
            theta = np.arctan2(slopeX, slopeY)
            yaw = degrees(theta)
            
            
            

            

        # if foundGreen[c][1] < foundYellow[c][1] and foundGreen[c][0] < foundYellow[c][0]:
        #     print("made it in")
        #     yaw += 360
        # else:
        #     print("nothing")
        #     yaw = yaw
        
        # Restrict the yaw angle to the range of -180 to 180 degrees
        if yaw > 180:
            yaw -= 180
        elif yaw < -180:
            yaw += 180
            
        xyStickLocation[c][0] = savePurple[c][0]
        xyStickLocation[c][1] = savePurple[c][1]
        xyStickLocation[c][2] = yaw
        print("yaw = ", yaw)
        # print('\n')
        
    ##### end for loop a #####

    print("stick X Y YAW", xyStickLocation)
### ======END====== ======END====== ======END======    TIME TO GET YAW    ======END====== ======END====== ======END====== ###








### ==================== ====================    MOVING BLOCK TIME    ==================== ==================== ###

    global rowCounter
    rowCounter = 0
# block moving
    for x in range(0, numSticks-1, 2):
    # move left stick
        print(x)
        xyStick = [xyStickLocation[x][0], xyStickLocation[x][1], blockSize]
        xyStickTarget = [targetxl, targetyl, targetzl]
        
        move_block(pub_command, loop_rate, xyStick, xyStickTarget, vel, accel, xyStickLocation[x][2])

    # move Right stick
        print(x+1)
        xyStick = [xyStickLocation[x+1][0], xyStickLocation[x+1][1], blockSize]
        xyStickTarget = [targetxr, targetyr, targetzr]
        
        move_block(pub_command, loop_rate, xyStick, xyStickTarget, vel, accel, xyStickLocation[x+1][2])


    # get next row locations
        if rowCounter < 2:
            offset = 3
        elif rowCounter >= 2 or rowCounter < 4:
            offset = 2
        elif rowCounter >= 4:
            offset = 1
    # left block loco for next row
        targetyl = targetyl + (blockSize/offset)
        targetzl = targetzl + blockSize
    # right block loco for next row
        targetyr = targetyr - (blockSize/offset)
        targetzr = targetzr + blockSize

    # increment rowCounter
        rowCounter = rowCounter + 1
    ##### end for loop x #####
    # move middle top
    xyStick = [xyStickLocation[numSticks-1][0], xyStickLocation[numSticks-1][1], blockSize]
    xyStickTarget = [targetxr, distance/2, targetzr]
    move_block(pub_command, loop_rate, xyStick, xyStickTarget, vel, accel, xyStickLocation[numSticks-1][2])

### ======END====== ======END====== ======END======    MOVING BLOCK END    ======END====== ======END====== ======END====== ###






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
