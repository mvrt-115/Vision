import cv2
import numpy as np
from apriltag import apriltag

imagepath = 'mosaic2.png'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tag36h11")

detections = detector.detect(image)

print(len(detections))
# print(detections)

for r in detections :
    (pt1, pt2, pt3, pt4) = r.get("lb-rb-rt-lt")
    print("hi", r.get("lb-rb-rt-lt"))

    pt1 = (int(pt1[0]), int(pt1[1]))
    pt2 = (int(pt2[0]), int(pt2[1]))
    pt3 = (int(pt3[0]), int(pt3[1]))
    pt4 = (int(pt4[0]), int(pt4[1]))

    print(pt1, pt2, pt3, pt4)

    cv2.line(image, pt1, pt2, (255, 0, 0), 2)
    cv2.line(image, pt2, pt3, (255, 0, 0), 2)
    cv2.line(image, pt3, pt4, (255, 0, 0), 2) 
    cv2.line(image, pt4, pt1, (255, 0, 0), 2)

print(image.shape)

cv2.imshow("April Tags", image)
cv2.waitKey(0)
