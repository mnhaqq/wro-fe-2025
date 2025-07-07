#!./env/bin/python3

from ultrasonic_reader import UltrasonicSensor
from src.MainCode.ArduinoComms import ArduinoComms
import RPi.GPIO as GPIO
import time

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    arduino = ArduinoComms()
    arduino.connect('/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0', 9600, 1000)
    sensors = [
        UltrasonicSensor(trig_pin=17, echo_pin=27),  # Left
        UltrasonicSensor(trig_pin=22, echo_pin=23),  # Front
        UltrasonicSensor(trig_pin=24, echo_pin=25)   # Right
    ]
    # Wall following parameters
    TARGET_RIGHT_DIST = 20.0  # Target distance from right wall in cm
    KP = 1.0  
    OPEN_SPACE_THRESHOLD = 60.0  # Distance in cm to consider as open space
    MAX_TURNS = 12  
    open_space_count = 0
    in_open_space = False
    SPEED = 200  
    arduino.set_drive_motor_value(SPEED)

    # Capture initial front distance
    initial_front_distance = sensors[1].get_distance()  # Front sensor is index 1
    STOP_MARGIN = 5.0  # Margin of error in cm for stopping
    stopped = False

    try:
        while True:
            distances = [sensor.get_distance() for sensor in sensors]
            right_dist = distances[2]
            front_dist = distances[1]
            error = TARGET_RIGHT_DIST - right_dist
            correction = KP * error
            
            # Feedback system: send steering command to Arduino
            steering_angle = 90 + correction  # 90 is straight, adjust as needed
            steering_angle = max(min(int(steering_angle), 135), 45)  # Clamp to safe range
            arduino.set_steering_motor_value(steering_angle)
            arduino.set_drive_motor_value(SPEED)

            # Open space detection and turn counting
            if right_dist > OPEN_SPACE_THRESHOLD and not in_open_space:
                open_space_count += 1
                in_open_space = True
            elif right_dist <= OPEN_SPACE_THRESHOLD and in_open_space:
                in_open_space = False

            if open_space_count >= MAX_TURNS and not stopped:
                # After max turns, move forward until front distance matches initial distance
                if abs(front_dist - initial_front_distance) <= STOP_MARGIN:
                    arduino.set_drive_motor_value(0)
                    stopped = True
                    break
            elif open_space_count >= MAX_TURNS:
                arduino.set_drive_motor_value(0)
                break

            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        if arduino.connected():
            arduino.set_drive_motor_value(0)
            arduino.disconnect()
