#!./env/bin/python3

import keyboard
import time
from ArduinoComms import ArduinoComms


class KeyboardController:
    def __init__(self):
        self.arduino = ArduinoComms()
        self.speed = 0
        self.steer = 90
        self.last_action_time = time.time()

    def arduino_setup(self):
        try:
            self.arduino.connect('/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0', 9600, 1000)
            if self.arduino.connected():
                print("Connected to Arduino.")
            else:
                print("Failed to connect.")
        except Exception as e:
            print(f"Error: {e}")

    def set_speed(self, direction):
        if direction:
            self.speed += 10
        else:
            self.speed -= 10
        self.speed = max(min(self.speed, 500), -500)
        print(self.speed)
        if self.arduino.connected():
            self.arduino.set_drive_motor_value(self.speed)

    def set_steer(self, direction):
        if direction:
            self.steer += 5
        else:
            self.steer -= 5
        self.steer = max(min(self.steer, 135), 45)
        if self.arduino.connected():
            self.arduino.set_steering_motor_value(self.steer)

    def stop(self):
        self.speed = 0
        self.steer = 90
        if self.arduino.connected():
            self.arduino.set_steering_motor_value(self.steer)
            self.arduino.set_drive_motor_value(self.speed)

    def run(self):
        self.arduino_setup()
        try:
            print("Program Started")
            while True:
                now = time.time()
                if now - self.last_action_time > 0.1:
                    if keyboard.is_pressed('w'):
                        self.set_speed(True)
                    if keyboard.is_pressed('s'):
                        self.set_speed(False)
                    if keyboard.is_pressed('a'):
                        self.set_steer(True)
                    if keyboard.is_pressed('d'):
                        self.set_steer(False)
                    if keyboard.is_pressed('space'):
                        self.stop()
                    self.last_action_time = now

                if keyboard.is_pressed('q'):
                    print("Quitting...")
                    break

                time.sleep(0.01)
        finally:
            if self.arduino.connected():
                self.arduino.disconnect()
                print("Disconnected.")


if __name__ == '__main__':
    controller = KeyboardController()
    controller.run()
