import numpy as np
import cv2
import serial
import time

cap = cv2.VideoCapture(0)
c1=0
linecolor = (100, 215, 255) 
lwr_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 30])

Ser = serial.Serial("COM8", baudrate=9600)
Ser.flush()
width=cap.get(4)
# print(width)

while True:
    ret, frame = cap.read()
    if not ret:
        _,frame=cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.inRange(hsv, lwr_black, upper_black)
    mask = cv2.dilate(mask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cnts,_=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    center = None
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 3:
            cv2.circle(frame, center, 5, linecolor, -1)
        cv2.circle(frame, (320, 465), 5, (0, 255, 0), thickness=2)
        # print(center)

        cv2.line(frame, center, (320, 465), (0, 255, 0), thickness=2, lineType=-1)

        if center[0] != 320:
            slope = (465 - center[1])/(320 - center[0])
            print(slope)
        
        if slope < 0.0 and slope > -3.0:
            frame = cv2.putText(frame, "Going right", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 255, 255), 2)
            Ser.write(b"R")


        elif slope > 0.0 and slope < 3.0:
            frame = cv2.putText(frame, "Going left", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 255, 255), 2)
            Ser.write(b"L")

        
        else:
            frame = cv2.putText(frame, "Going straight", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 255, 255), 2)
            Ser.write(b"F")


    else:
        print("Track Not Visible")
        c1+=1
        if(c1==5):
            Ser.write(b'S')
            c1=0
        
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        # Ser.close()
        cv2.destroyAllWindows()
        break
