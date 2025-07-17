import RPi.GPIO as GPIO
import subprocess
import time
import os
import signal

BUTTON_PIN = 17

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

process = None  # Track the running process

def toggle_executable(channel):
    global process
    if process is None:
        print("Button pressed! Starting script...")
        process = subprocess.Popen(
            ['python3', 'open.py'],
            cwd='/home/pi/Documents/wro-fe-2025/src/MainCode/'
        )
    else:
        print("Button pressed! Stopping script...")
        try:
            os.kill(process.pid, signal.SIGTERM)
        except Exception as e:
            print(f"Error stopping script: {e}")
        process = None

GPIO.add_event_detect(BUTTON_PIN, GPIO.RISING, callback=toggle_executable, bouncetime=300)

try:
    print("Waiting for button press to toggle script...")
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    if process:
        os.kill(process.pid, signal.SIGTERM)
