import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)

try:
    while True:
        GPIO.output(20, GPIO.HIGH)
        time.sleep(0.05)  # 0.05초 동안 대기

except KeyboardInterrupt:
    GPIO.cleanup()