#!/usr/bin/env python
import time
import RPi.GPIO as GPIO

if __name__ == "__main__":
    trigger_pin = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)

    GPIO.output(trigger_pin, GPIO.LOW)

    while True:
        # Write output high
        GPIO.output(trigger_pin, GPIO.HIGH)
        # Sleep
        time.sleep(10)
        # Write low
        #GPIO.output(trigger_pin, GPIO.LOW)
