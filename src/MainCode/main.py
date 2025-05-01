#!./env/bin/python3

import cv2
import numpy as np
from picamera2 import Picamera2
from ArduinoComms import ArduinoComms
from pid import PIDController
import time


def process_frame(frame):
        height, width = frame.shape[:2]
        roi = frame[int(height*0.6):, :]  
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=50)
        
        mid_x = width // 2
        lane_center = mid_x

        if lines is not None:
            x_coords = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_coords.extend([x1, x2])
            lane_center = int(np.mean(x_coords)) if x_coords else mid_x

        error = lane_center - mid_x

        cv2.line(roi, (lane_center, 0), (lane_center, roi.shape[0]), (0, 255, 0), 2)
        cv2.line(roi, (mid_x, 0), (mid_x, roi.shape[0]), (0, 0, 255), 2)    

        return error, frame


def main():
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(1)

    arduino = ArduinoComms()
    arduino.connect("/dev/ttyUSB0", 9600, 1000)

    pid = PIDController(0, 0, 0)

    speed = 100
    
    try:
        while True:
            frame = picam2.capture_array()
            error, annotated_frame = process_frame(frame)
            steer_adjust = pid.compute(error)
            steer = int(90 + np.clip(steer_adjust, -30, 30))

            arduino.set_steering_motor_value(steer)
            arduino.set_drive_motor_value(speed)

            cv2.imshow("Robot View", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        arduino.set_drive_motor_value(0)
        arduino.set_steering_motor_value(90)
        arduino.disconnect()
        cv2.destroyAllWindows()
    


if __name__ == '__main__':
    main()