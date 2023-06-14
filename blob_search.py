#!/usr/bin/env python3

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.02746562
beta = np.sqrt((76)**2+(5)**2)/0.1   #length of pixels/length in meter

w = 640
h = 480
tx = h/2 - 48
ty = w/2 - 309

# Function that converts image coord to world coord
def IMG2W(row, col):

    global theta
    global beta
    global tx
    global ty

    x_cen = int(h/2)
    y_cen = int(w/2)

    x_c = ((col - x_cen) + tx)/beta
    y_c = ((row - y_cen) + ty)/beta

    xw = np.cos(theta)*x_c + np.sin(theta)*y_c
    yw = -np.sin(theta)*x_c + np.cos(theta)*y_c

    return xw, yw

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.4
    params.maxCircularity = 1.0

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    if color == 'green':
        lower = (45,80,100)     # green lower
        upper = (70,255,255)   # green upper
    elif color == 'orange':
        lower = (9,50,30)     # orange lower
        upper = (16,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
