from pupil_apriltags import Detector
import numpy as np
import cv2
import time
import math

import sys

sys.path.insert(0, 'utils/')
from jsontools import JsonTools

detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)


def main():
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("Unable to open camera")
        exit()

    print("Press 'q' to quit")

    while True:
        ok, frame = cap.read()

        if not ok:
            print("Unable to receive frame. Exiting...")
            break
        
        draw_detect(frame)
        cv2.imshow("April Tags", frame)

        if cv2.waitKey(int(1000/30)) & 0xFF == ord('q'):
            break
        
    cap.release()
    cap.destroyAllWindows()

def draw_detect(frame):
    grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(grayImage, estimate_tag_pose=True, camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909), tag_size=.038)

    #Read focal length and camera stuff from file
    tools = JsonTools()
    fx = tools.getJsonVal("files/matrix.txt", "fx")
    fy = tools.getJsonVal("files/matrix.txt", "fy")
    cx = tools.getJsonVal("files/matrix.txt", "cx")
    cy = tools.getJsonVal("files/matrix.txt", "cy")

    tags = detector.detect(grayImage, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=0.161)

    for t in tags :
        (ptA, ptB, ptC, ptD) = t.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 20)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 20)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 20)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 20)

        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(t.center[0]), int(t.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        printall(t)

#Print out coords and angle relative to field
def printall(t):
    #Coordinates relative to field
    cords = get_coords(t)
    print(f'Robot  Pos: x:{cords[0]*10} y:{cords[1]*10}')

    #Angle relative to field
    #angle = get_angle(t)
    #print(f'Robot  Angle:{angle}')

#Return coords relative to field
def get_coords(t):
    # Distance from april tag
    x = t.pose_t[0]
    y = t.pose_t[2]

    #TAG coordinates
    aprilX = 10 #arbitrary constant
    aprilY = 0 #arbitrary constant
    theta = 3.1415926/2
    dx = -math.sqrt(x**2+y**2)*math.cos(theta+math.atan(x/y))
    dy = math.sqrt(x**2+y**2)*math.sin(theta+math.atan(x/y))

    # Calculating ROBOT coordinates
    coordX = aprilX +dx
    coordY = aprilY + dy

    return (coordX, coordY)
    
#Convert angle from radiuns to degrees
def get_angle(t):
    return rot2eul(t.pose_R)*180/3.1415926

#Convert weird matrix to euls (x, y, z rotation)
#http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    a = np.array((alpha, beta, gamma))[1]
    return a

main()