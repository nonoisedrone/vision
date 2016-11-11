import numpy as np
import cv2
import glob

import os
import argparse

import rospy

"""
Uses opencv to calibrate a camera.

TODO:
- command line arguments to select camera source
- command line arguments to suppress image display
- command line arguments to calibrate stereo pairs
- command line arguments to give chessboard sizes
"""

minImages = 30

def calib(source='default', filename=''):
    cap = None
    if source == 'default':
        cap = cv2.VideoCapture()
        #uses built in drivers to pull from camera at index 0
        cap.open(0)
    elif source == 'file':
        images = glob.glob(os.path.join(filename, "*.jpg"))

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    while len(objpoints) < minImages:
        if source == 'default':
            ret, img = cap.read()
        elif source == 'file':
            #TODO will throw an error if there aren't enough images
            image = cv2.imread(images.pop())

        #convert to grayscale
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            print "new image found: ", len(objpoints)
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7,6), corners, ret)

            try:    
                cv2.imshow('img',img)
            except Exception:
                print "bad corners"

        cv2.waitKey(100)


    if source == 'default':
        cap.release()
    cv2.destroyAllWindows()

    #calculate matrices
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    #print out reprojection error
    mean_error = 0
    for i in xrange(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print "total error: ", mean_error/len(objpoints), "(0 is best)"

    return mtx, dist

def main():
    mtx, dist = calib()
    print mtx, dist

if __name__ == '__main__':
    main()