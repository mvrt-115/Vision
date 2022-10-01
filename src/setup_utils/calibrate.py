import numpy as np
import cv2 as cv
import glob
from json_stuff import JsonTools

'''
src: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

Get variables from reliable cameramatrix like this:
+------------+
| fx, 0,  cx |
| 0,  fy, cy |
| 0,  0,  1  |
+------------+
'''

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Size of chessboard
size = (6,9)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((size[0]*size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:size[0],0:size[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('images/*.jpg')

print(len(images), 'images')

numWork = 0

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, size, None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        numWork += 1
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, size, corners2, ret)
        cv.imshow('img', img)

print(numWork, 'work')

# Camera matrix, distortion coefficients, rotation and translation vectors
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# Undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# Crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)
error = 0
for i in range(len(objpoints)):
    projected_points = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)[0]
    temp = cv.norm(imgpoints[i], projected_points, cv.NORM_L2)/len(projected_points)
    error += temp 

error /= len(objpoints)

#Write things to file
stuff = {
    'fx': mtx[0][0],
    'fy': mtx[1][1],
    'cx': mtx[0][2],
    'cy': mtx[1][2],
    'error' : error
}

tools = JsonTools()
tools.writeJson('calibration.txt', stuff)