#!/usr/bin/env python

import numpy as np
import cv2  
import glob
import os

"""
This script will take the images from /images and proceed camera calibration
The result will output to cam_calibration_data.npz 
If the "Mean reprojection error" is significantly > 0.5, 
your calibration result might be problematic.
"""
# TODO: 
# 1. Change the CHECKERBOARD dimensions to your checkerboardsetup
# 2. Change the square_size to your checkerboard setup
CHECKERBOARD = (6,8)    
square_size = 25    # in millimeter
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 
 
# Defining the world coordinates for 3D points
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2) * square_size

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Extracting path of individual image stored in a given directory
images = glob.glob('./images/*.jpg')
for fname in images:
    print("Reading "+str(fname))
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
    else:
        print("Cannot detect corners: "+fname)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
 
print("Camera Calibration Completed!")

total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

mean_error = total_error / len(objpoints)
print(f"Mean reprojection error: {mean_error} (ideally < 0.5)")


# Save the camera matrix and distortion coefficients using np.savez
np.savez('cam_calibration_data.npz', camera_matrix=mtx, dist_coeffs=dist)

# Print the camera matrix
# print("Camera Matrix:")
# print(mtx)

print("Result saved in cam_calibration_data.npz!")