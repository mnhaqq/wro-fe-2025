#!./env/bin/python3

from ArduinoComms import ArduinoComms

def main():
    arduino = ArduinoComms()

    try:
        arduino.connect('/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0', 9600, 1000)

        if arduino.connected():
            print("Connected to Arduino.")

            arduino.set_drive_motor_value(1700)
            arduino.set_steering_motor_value(60)

        else:
            print("Failed to connect.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        arduino.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()
