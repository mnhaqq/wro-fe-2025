import cv2
from picamera2 import Picamera2
from ArduinoComms import ArduinoComms
from pid import PIDController
import numpy as np
from masks import rOrange, rBlack, rBlue
import time
from time import sleep
from utils import display_roi, display_variables, find_contours, max_contour

def main():
    time.sleep(3)

    picam2 = Picamera2()
    sensor_modes = picam2.sensor_modes
    selected_mode = None
    for mode in sensor_modes:
        if mode['size'] == (2304, 1296) or mode['size'] == (1920, 1080) or mode['size'] == (1280, 720):
            selected_mode = mode
            break
    if selected_mode is None:
        selected_mode = sensor_modes[0]

    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (1280, 720)},
        sensor={"output_size": selected_mode['size'], "bit_depth": selected_mode['bit_depth']}
    )

    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    # Regions of interest
    ROI1 = [10, 420, 200, 500]  # left
    ROI2 = [1080, 420, 1270, 500]  # right
    ROI3 = [540, 420, 740, 500]  # center

    arduino = ArduinoComms()
    arduino.connect("/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0", 9600, 1000)

    lTurn = False
    rTurn = False
    t = 0
    kp = 0.001#0.001
    kd = 0.0009
    pid = PIDController(kp, 0, kd)
    straightConst = 90
    turnThresh = 150
    exitThresh = 1500
    angle = 90
    prevAngle = angle
    tDeviation = 45
    sharpRight = straightConst - tDeviation
    sharpLeft = straightConst + tDeviation
    maxRight = straightConst - 60
    maxLeft = straightConst + 60
    speed = 210
    error = 0
    arduino.set_drive_motor_value(0)
    sleep(2)

    lDetected = False
    debug = True
    start = False
    turnDir = "none"
    last_turn_time = 0
    turn_cooldown = 1  # seconds cooldown after detecting a turn

    while True:
        rightArea, leftArea = 0, 0
        img = picam2.capture_array()
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)

        cListLeft = find_contours(img_lab, rBlack, ROI1)
        cListRight = find_contours(img_lab, rBlack, ROI2)
        cListOrange = find_contours(img_lab, rOrange, ROI3)
        cListBlue = find_contours(img_lab, rBlue, ROI3)

        leftArea = max_contour(cListLeft, ROI1)[0]
        rightArea = max_contour(cListRight, ROI2)[0]

        if max_contour(cListOrange, ROI3)[0] > 100:
            lDetected = True
            if turnDir == "none":
                turnDir = "right"
        elif max_contour(cListBlue, ROI3)[0] > 100:
            lDetected = True
            if turnDir == "none":
                turnDir = "left"

        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cListLeft, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cListRight, -1, (0, 255, 0), 2)

        error = rightArea - leftArea
        angle = int(max(straightConst + pid.compute(error), 0))

        if leftArea <= turnThresh and not rTurn:
            lTurn = True
        elif rightArea <= turnThresh and not lTurn:
            rTurn = True

        if not start:
            arduino.set_steering_motor_value(90)
            arduino.set_drive_motor_value(speed)
            start = True

        arduino.set_drive_motor_value(speed)

        if angle != prevAngle:
            if lTurn or rTurn:
                if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn):
                    lTurn = False
                    rTurn = False
                    pid.prev_error = 0
                    if lDetected:
                        current_time = time.time()
                        if current_time - last_turn_time > turn_cooldown:  # Check cooldown
                            t += 1
                            last_turn_time = current_time  # Update last turn time
                            lDetected = False

                elif lTurn:
                    angle = min(max(angle, sharpLeft), maxLeft)
                elif rTurn:
                    angle = max(min(angle, sharpRight), maxRight)

                arduino.set_steering_motor_value(90)
                time.sleep(0.05)
                arduino.set_steering_motor_value(angle)
                time.sleep(0.01)
            else:
                arduino.set_steering_motor_value(max(min(angle, sharpLeft), sharpRight))
                time.sleep(0.01)

        pid.prev_error = error
        prevAngle = angle

        if t == 12 and abs(angle - straightConst) <= 10:
            if turnDir == "left":
                sleep(1)
            else:
                sleep(1.5)

            arduino.set_drive_motor_value(0)
            break

        if debug:
            if cv2.waitKey(1) == ord('q'):
                arduino.set_drive_motor_value(0)
                break

            img = display_roi(img, [ROI1, ROI2, ROI3], (255, 204, 0))
            cv2.imshow("finalColor", img)

            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "left turn": lTurn,
                "right turn": rTurn,
                "# turns": t,
                "lDetected": lDetected
            }

            display_variables(variables)

    if debug:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()