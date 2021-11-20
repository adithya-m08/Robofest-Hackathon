import cv2
import numpy as np
import serial

cap=cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

width=cap.get(3)

while True:
    ret, frame = cap.read()


    count=0
    i=200


    if i>0 and i<91:
        print("segment 1")
    elif i>91 and i<182:
        print("segment 2")
    elif i>182 and i<273:
        print("segment 3")
    elif i>273 and i<364:
        print("segment 4")
    elif i>364 and i<455:
        print("segment 5")
    elif i>455 and i<546:
        print("segment 6")
    elif i>546 and i<640:
        print("segment 7")
    else:
        print("point not in image")



    cv2.imshow("final",frame)
    if cv2.waitKey(1) & 0xff == ord('q'):

        break
cap.release()
cv2.destroyAllWindows()
