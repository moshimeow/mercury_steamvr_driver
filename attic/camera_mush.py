import cv2
import time

if True:
    cap = cv2.VideoCapture()

    cap.open(1, cv2.CAP_DSHOW)
else:
    cap = cv2.VideoCapture(1)
    print(cap.getBackendName())
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