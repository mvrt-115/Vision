import numpy
import cv2
import time
import glob

'''
Takes bunch of pictures that we can use to calibrate
'''

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Unable to open camera")
        exit()

    print("Press 'q' to quit")
    fr = len(glob.glob('images/*.jpg'))
    while True:
        ok, frame = cap.read()

        if not ok:
            print("Unable to receive frame. Exiting...")
            break

        cv2.imshow("April Tags", frame)
        cv2.imwrite(f"images/{fr}.jpg", frame)
        fr+=1
        if cv2.waitKey(400) & 0xFF == ord('q'):
            break
    cap.release()

if __name__ == "__main__":
    main()