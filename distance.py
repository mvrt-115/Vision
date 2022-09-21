#from dt_apriltags import Detector
from pupil_apriltags import Detector
import numpy
import cv2
import time

detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

def main():
    cap = cv2.VideoCapture(5)

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

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        
    cap.release()
    cap.destroyAllWindows()

def draw_detect(frame):
    grayImage = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    pixel_size = 0.004
    resolution = 1280*720

    #focal length
    #fx = 3156.7
    #fy = 3129.5
    #fx = pixel_size * resolution
    #fy = fx

    fx = 511.12698364
    fy = 871.23065186

    #optiocal center
    cx = 968.75939971
    cy = 279.759014 
    
    tags = detector.detect(grayImage, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=0.06)

    for t in tags :
        print(t.pose_t) #fix these

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

main()