from ArduinoComms.ArduinoComms import ArduinoComms

def main():
    arduino = ArduinoComms()

    try:
        arduino.connect('/dev/ttyUSB0', 9600, 1000)

        if arduino.connected():
            print("Connected to Arduino.")

            arduino.set_drive_motor_value(1600)
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
