import cv2
import serial
import time
import math

# assumption about servo performance
SERVO_RPM: int = 60

# signifance of distance difference between center and center of face
K_P: float = 60

servoPosition: float = 90
servoTarget: float = 90
lastPositionGiven: int = 90

def positionChange(current: float, target: float, dt: float) -> float:
    maxChangePossible: float = SERVO_RPM * 6 * dt
    changeRequested: float = target-current
    if abs(changeRequested) < maxChangePossible:
        return changeRequested
    return copysign(maxChangePossible, changeRequested)

def clipNumber(number: float, minimum: float, maximum: float) -> float:
    return minimum if number < minimum else \
           maximum if number > maximum else \
           number

# Replace COM5 with your Arduino's actual port
# arduino = serial.Serial('COM5', 9600)
time.sleep(2)  # Let Arduino initialize

cap = cv2.VideoCapture(0)

ret, frame1 = cap.read()
frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
frame1 = cv2.GaussianBlur(frame1, (21, 21), 0)


startTime: float = 0
dt: float = 0

while True:
    startTime = time.perf_counter()
    ret, frame2 = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    delta = cv2.absdiff(frame1, gray)
    thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) < 800:
            continue

        (x, y, w, h) = cv2.boundingRect(contour)
        center_x = x + w // 2
        frame_width = frame2.shape[1]

        #find the difference between center and face center [-1, 1]
        difference: float = 2*(center_x - (frame_width//2)) / frame_width
        servoTarget = servoPosition + difference * K_P;
        servoTarget = clipNumber(servoTarget, 0, 180);
        
        if lastPositionGiven != round(servoTarget):
            lastPositionGiven = round(servoTarget)
            #ardunio.write(bytes([lastPositionGiven]))
            print(f"Sent to Arduino: {lastPositionGiven}")

        #draws the bounding rect I believe?
        cv2.rectangle(frame2, (x, y), (x + w, y + h), (0, 255, 0), 2)
        break  # Track only first motion blob

    cv2.imshow("Motion Tracking", frame2)
    frame1 = gray

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    dt = time.perf_counter() - startTime;
    servoPosition += positionChange(servoPosition, servoTarget, dt)
    servoPosition = clipNumber(servoPosition, 0, 180);

cap.release()
cv2.destroyAllWindows()
arduino.close()
