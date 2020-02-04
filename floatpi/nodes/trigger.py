#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Bool,Time
import RPi.GPIO as GPIO

if __name__ == "__main__":
    rospy.init_node('camera_trigger_node')
    trigger_pub = rospy.Publisher('trigger', Time, queue_size=1)
    trigger_pin = rospy.get_param('~trigger_pin', 26)
    trigger_rate = rospy.get_param('~trigger_rate', 2)  # in Hz. Defaults at 2Hz
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)

    r = rospy.Rate(trigger_rate) # hz
    pulse_width = 0.05

    while not rospy.is_shutdown():
        # Write output high
        GPIO.output(trigger_pin, GPIO.HIGH)
        # Sleep
        time.sleep(pulse_width)
        # Write low
        GPIO.output(trigger_pin, GPIO.LOW)

        tmsg = Time()
        tmsg.data = rospy.get_rostime()
        trigger_pub.publish(tmsg)

        r.sleep()
    GPIO.cleanup(trigger_pin)
