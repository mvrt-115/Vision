from pupil_apriltags import Detector
import numpy
import cv2
import time
import sys

sys.path.insert(0, 'utils/')
from jsontools import JsonTools

'''
Prints distance between camera and april tag
'''

# Detector
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
    #Read focal length and camera stuff from file
    tools = JsonTools()
    fx = tools.getJsonVal("files/matrix.txt", "fx")
    fy = tools.getJsonVal("files/matrix.txt", "fy")
    cx = tools.getJsonVal("files/matrix.txt", "cx")
    cy = tools.getJsonVal("files/matrix.txt", "cy")
    tag_size = 0.1738 #square length in meters
    tags = detector.detect(grayImage, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=tag_size)

    for t in tags :
        (ptA, ptB, ptC, ptD) = t.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # Draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 20)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 20)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 20)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 20)

        # Draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(t.center[0]), int(t.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # Prints translation pose relative to the apriltag rather than camera
        print(f'Transformed Translation: x: {t.pose_t[0]}, y: {-t.pose_t[1]}, z: {t.pose_t[2]}') # interested in our x, y which is (x, z)
        pose = t.pose_t

main()