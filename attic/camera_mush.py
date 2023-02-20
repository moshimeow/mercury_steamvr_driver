import cv2
import time

if False:
    cap = cv2.VideoCapture()

    cap.open(1, cv2.CAP_DSHOW)
else:
    # :) Yeah this seems to work :D
    cap = cv2.VideoCapture(1, cv2.CAP_MSMF, (cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_NONE))
    
    # :(
    # cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    # print(cap.getBackendName())
# print(cap.get)
blah = cap.get(cv2.CAP_PROP_FOURCC )
# print(blah)
# print(cv2.VideoWriter.fourcc(*"MJPG"))
# print(cv2.VideoWriter.fourcc(*"YUYV"))

# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"YUYV"))

# cap.set(cv2.CAP_PROP_MODE, cv2.CAP_MODE_YUYV);

# this seems to work ish? aaaa
cap.set(cv2.CAP_PROP_MODE, cv2.VideoWriter.fourcc(*"YUYV"));

while True:
    ret, img = cap.read()
    cv2.imshow("h", img)
    cv2.waitKey(1)
    print(cap.get(cv2.CAP_PROP_POS_MSEC))

    print("blah")
    # print(time.localtime(time.time()))