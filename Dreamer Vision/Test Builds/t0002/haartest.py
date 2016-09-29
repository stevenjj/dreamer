import serial
import numpy as np
import cv2

xgain = 0.01
ygain = 0.01
xres = 1300
yres = 700

gazeKey = 0xA7
gazeFocus = 128
gazePitch = 128
gazeYaw = 128
x0 = xres/2
y0 = yres/2

# ser = serial.Serial('COM12', 115200, parity='N')

# multiple cascades: https://github.com/Itseez/opencv/tree/master/data/haarcascades
#https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
#https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_eye.xml
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

cap = cv2.VideoCapture(0)

while 1:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x,y,w,h) in faces:
        x0=x+w/2
        y0=y+h/2

        cv2.rectangle(img,(x0-5,y0-5),(x0+5,y0+5),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

    gazePitch += (x0 - xres/2)*xgain
    gazeYaw += (y0 - yres/2)*ygain

    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
#    ser.write(newMessage)
    print gazePitch, gazeYaw
#    print newMessage

cap.release()
cv2.destroyAllWindows()
