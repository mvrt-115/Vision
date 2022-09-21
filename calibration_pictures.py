import numpy
import cv2
import time

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Unable to open camera")
        exit()

    print("Press 'q' to quit")
    fr = 0
    while True:
        ok, frame = cap.read()

        if not ok:
            print("Unable to receive frame. Exiting...")
            break

        cv2.imshow("April Tags", frame)
        cv2.imwrite(f"images/{fr}.jpg", frame)
        fr+=1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cv2.waitKey(500)
    cap.release()

if __name__ == "__main__":
    main()