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
                    debug=0)ß

def main():
    
    cap = cv2.VideoCapture(0)

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

    pixel_size = 0.004
    resolution = 1280*720

    #focal length
    fx = 969.3881228971977
    fy = 972.1641034650454
  
    #optiocal center
    cx = 638.653318241334
    cy = 380.97132588633445

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

        #prints translation pose relative to the apriltag rather than camera
        #print(f'Transformed Translation: x: {t.pose_t[0]}, y: {-t.pose_t[1]}, z: {t.pose_t[2]}') # interested in our x, y which is (x, z)
        #print(f'Rotation: {t.pose_R}')
        pose = t.pose_t
        new_coors(t)


def new_coors(t):
    # Distance from april tag
    x = t.pose_t[0]
    y = t.pose_t[1]
    z = t.pose_t[2]

    #TAG coordinates
    aprilX = 10
    aprilY = 10
    aprilZ = 0

    # Calculating ROBOT coordinates
    coordX = aprilX - z
    coordY = aprilY - x
    coordZ = aprilZ - y

    print(f'Target Pos: x:{aprilX} y:{aprilY} z:{z}')
    print(f'Robot  Pos: x:{coordX} y:{coordY} z:{coordZ}')
    
main()