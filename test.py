from dt_apriltags import Detector
import numpy
import cv2

detector = Detector(searchpath=['apriltags'],
                    families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

imagepath = 'mosaic2.png'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
tags = detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=None)

cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

for t in tags :
    c = t.corners

    # print("BL", c[0])
    # print("BR", c[1])
    # print("TR", c[2])
    # print("TL", c[3])
    # print()

    (ptA, ptB, ptC, ptD) = t.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))

    # draw the bounding box of the AprilTag detection
    cv2.line(image, ptA, ptB, (0, 255, 0), 20)
    cv2.line(image, ptB, ptC, (0, 255, 0), 20)
    cv2.line(image, ptC, ptD, (0, 255, 0), 20)
    cv2.line(image, ptD, ptA, (0, 255, 0), 20)

    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(t.center[0]), int(t.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

    print((cX, cY))

cv2.circle(image, (0, 0), 100, (255, 0, 0), -1)

cv2.imshow("April Tags", image)
cv2.waitKey(0)

print(len(tags))