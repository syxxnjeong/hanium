import cv2
import numpy as np
import datetime
from time import time
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)
video = cv2.VideoCapture(-1)
fire_status = True
previous = time()
delta = 0

def fire_detect():
    global fire_status, current, delta, previous

    # 감시는 무한 반복 하되 10초마다 사진을 찍음
    while True:

        (grabbed, frame) = video.read()
        frame = cv2.resize(frame, (800, 600))  # 680,480
        if not grabbed:
            break
        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower = [5, 44, 228]
        upper = [25, 255, 255]
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        mask = cv2.inRange(hsv, lower, upper)
        conts, h = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contoura in conts:
            area = cv2.contourArea(contoura)
            if (area > 400):

                x, y, w, h = cv2.boundingRect(contoura)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, 'Fire', (x + w, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
        output = cv2.bitwise_and(frame, hsv, mask=mask)
        no_red = cv2.countNonZero(mask)
        cv2.imshow("output", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # 여기에다 카메라 종료문을 선언하면 문제없음
            cv2.destroyAllWindows()
            video.release()
            break

        # 수치가 높을수록 정확도 떨어짐
        if int(no_red) > 1500 and fire_status == True:
            print("fire detected!!!")
            ser.write(b'1')

            now = datetime.datetime.now()

            # 탈출해서 시간 재야함
            fire_status = False
            previous = time()

        if fire_status == False:
            current = time()
            delta = delta + (current - previous)
            previous = current

            if delta > 10:
                delta = 0
                fire_status = True

        # Add the condition to send '2' to Arduino when nothing is detected
        if int(no_red) == 0:
            ser.write(b'0')

fire_detect()