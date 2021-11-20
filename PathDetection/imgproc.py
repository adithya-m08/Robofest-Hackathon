import numpy as np
import cv2
import serial
import time

cap = cv2.VideoCapture(0)
c1=0
linecolor = (100, 215, 255) 
lwr_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 60])

Ser = serial.Serial("COM8", baudrate=9600)
Ser.flush()
width=cap.get(3)

def aruco(img):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
        cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the image
        cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        markerID = (int(markerID))
        return markerID


while True:
    ret, frame = cap.read()
    if not ret:
        _,frame=cap.read()
    if aruco(frame)==0:
        print(aruco(frame))
        time.sleep(2)

        Ser.write(b"FF")
        Ser.write(b"FFFFFLLLLLLLLLLLLLLLLLL")

        time.sleep(2)

    elif aruco(frame)==1:
        print(aruco(frame))
        time.sleep(2)

        Ser.write(b"F")
        Ser.write(b"FFFFFRRRRRRRRRRRRRRRRRR")

        time.sleep(2)
    else:
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
                #cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)
                cv2.circle(frame, center, 5, linecolor, -1)
            
            if(x>0 and x<=0.25*width):
                print("L")
                Ser.write(b"L")
                time.sleep(0.01)
                
            elif(x >0.25*width and x<=0.75*width):
                print('F')
                Ser.write(b'F')
                time.sleep(0.01)
                
            elif(x >0.75*width and x<=width):
                print("R")
                Ser.write(b"R")
                time.sleep(0.01)
        else:
            print("Track Not Visible")
            c1+=1
            if(c1==3):
                Ser.write(b'RRR')
                c1=0
            
    cv2.imshow("Frame", frame)
    if cv2.waitKey(100) & 0xFF == ord('q'):
        cap.release()
        Ser.write(b'S')
        Ser.close()
        cv2.destroyAllWindows()
        break
