import numpy as np
import cv2



video = cv2.VideoCapture(0)

a = 0
while True:
    a = a+1
    check, frame = video.read()
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    lower_red = np.array([30,101,0])
    upper_red = np.array([255,255,255])

    mask = cv2.inRange(hsv,lower_red,upper_red)
    res = cv2.bitwise_and(frame,frame,mask = mask)
    
    cv2.imshow('Capturing',mask)
    cv2.imshow('Capturin',res)
    cv2.imshow('Capturing',frame)
    key = cv2.waitKey(5) & 0xFF
    if key == 27:
        break

video.release()
cv2.destroyAllWindows

