#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

if __name__ == "__main__":
    rospy.init_node('leak_sensor_node')
    leak_pub = rospy.Publisher('leak', Bool, queue_size=1)
    leak_pin = rospy.get_param('~leak_pin', 4)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(leak_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    r = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        leak_detected = GPIO.input(leak_pin)
        # Leak == high, dry==low
        leak_msg = Bool()
        leak_msg.data = leak_detected
        leak_pub.publish(leak_msg)

        r.sleep()
