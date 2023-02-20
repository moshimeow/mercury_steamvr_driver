import cv2
import time


def stream_10_frames(cap):

    for i in range(50):
        ret, img = cap.read()
        if (not ret):
            print("oh no")
            continue
        cv2.imshow("h", img)
        cv2.waitKey(1)
        print(cap.get(cv2.CAP_PROP_POS_MSEC))

        print("50 frames")


cap = cv2.VideoCapture()

cap.open(1, cv2.CAP_DSHOW)

stream_10_frames(cap)

cap.release()

cap = cv2.VideoCapture()
cap.open(1, cv2.CAP_MSMF)

# print(cap.get)
blah = cap.get(cv2.CAP_PROP_FOURCC )
# print(blah)
# print(cv2.VideoWriter.fourcc(*"MJPG"))
# print(cv2.VideoWriter.fourcc(*"YUYV"))

# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"YUYV"))

# cap.set(cv2.CAP_PROP_MODE, cv2.CAP_MODE_YUYV);

# this seems to work ish? aaaa
#cap.set(cv2.CAP_PROP_MODE, cv2.VideoWriter.fourcc(*"YUYV"));

while True:
    ret, img = cap.read()
    if (not ret):
        print("oh no")
        continue
    cv2.imshow("h", img)
    cv2.waitKey(1)
    # print(cap.get(cv2.CAP_PROP_POS_MSEC))

    print("blah")
    # print(time.localtime(time.time()))