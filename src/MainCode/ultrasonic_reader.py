
import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    def get_distance(self):
        GPIO.output(self.trig, False)
        time.sleep(0.01)
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()
        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    # Example pins: adjust as needed
    sensors = [
        UltrasonicSensor(trig_pin=17, echo_pin=27),  # Left
        UltrasonicSensor(trig_pin=22, echo_pin=23),  # Front
        UltrasonicSensor(trig_pin=24, echo_pin=25)   # Right
    ]
    try:
        while True:
            distances = [sensor.get_distance() for sensor in sensors]
            print(f"Left: {distances[0]} cm, Front: {distances[1]} cm, Right: {distances[2]} cm")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        GPIO.cleanup()
