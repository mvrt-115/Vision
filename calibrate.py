#calibrate whatever camera we are using
#src: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

#!!! ->
#todo: make this a video so it can capture multiple photos? 
#I also printed out the official chess board so we can use that tomorrow
#!!!! <-

#get variables from cameramatrix like this:
#once we have reliable pictures
#+------------+
#| fx, 0,  cx |
#| 0,  fy, cy |
#| 0,  0,  1  |
#+------------+

'''
Read ->
For better results, the distance between the camera and calibration grid should 
be approximately equal to the working distance that you intend to maintain in your 
application. Additionally, the resolution and focus of the camera should be maintained 
constant while taking pictures.
'''

import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)

objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('images/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        print("photo works")
    cv.waitKey(100)

#calibrate
#camera matrix, distortion coefficients, rotation and translation vectors
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

#print(objpoints)
#print(imgpoints)

print (mtx)
#print(newcameramtx)

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)
error = 0
for i in range(len(objpoints)):
    projected_points = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)[0]
    temp = cv.norm(imgpoints[i], projected_points, cv.NORM_L2)/len(projected_points)
    error += temp 

error /= len(objpoints)
print("Error: ", error)
cv.waitKey(0)