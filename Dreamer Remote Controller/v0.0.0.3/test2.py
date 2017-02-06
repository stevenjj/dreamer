import serial
import time

gazeKey = 0xA7
gazeFocus = 128
gazePitch = 128
gazeYaw = 128

ser = serial.Serial('COM12', 115200, parity='N')

while 1:
    gazeYaw += 1
    gazePitch += 0

    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(int(newCommand)) + "\r"
    ser.write(newMessage)
    print gazeYaw, gazePitch, newMessage

    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
