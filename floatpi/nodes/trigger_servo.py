#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Bool
import os
import sys

class ServoGPO:
    SYS_EXPORT_PATH = "/sys/class/gpio/export"
    SYS_UNEXPORT_PATH = "/sys/class/gpio/unexport"
    SYS_GPIO_BASE_PATH = "/sys/class/gpio/"

    def __init__(self, servo_pin):
        self.pin = 500 + servo_pin - 1
        self.pin_path = self.SYS_GPIO_BASE_PATH + "gpio{}/".format(self.pin)

    def initialise(self):
        if os.path.exists(self.pin_path):
            self.deinitialise()
            time.sleep(0.5)
        with open(self.SYS_EXPORT_PATH, "w") as gpio_enable:
            gpio_enable.write(str(self.pin))
            self.is_enabled = True
        time.sleep(0.5)
        with open(self.pin_path + "direction", "w") as gpio_direction:
            gpio_direction.write("out")

    def deinitialise(self):
        with open(self.SYS_UNEXPORT_PATH, "w") as gpio_disable:
            gpio_disable.write(str(self.pin))
            self.is_enabled = False

    def output(self, value):
        with open(self.pin_path + "value", "w") as gpio_write:
            gpio_write.write(str(value))

if __name__ == "__main__":
    rospy.init_node('camera_trigger_node')
    trigger_pub = rospy.Publisher('trigger', Bool, queue_size=1)
    trigger_pin = rospy.get_param('~trigger_pin', 1)  # Corresponds to the Servo rails
    trigger_rate = rospy.get_param('~trigger_rate', 8.0)  # in Hz. Defaults at 2Hz

    sgpo = ServoGPO(trigger_pin)
    sgpo.initialise()

    r = rospy.Rate(trigger_rate) # hz
    pulse_width = 0.002

    while not rospy.is_shutdown():
        # Write output high
        sgpo.output(1)
        # Sleep
        time.sleep(pulse_width)
        # Write low
        sgpo.output(0)

        bmsg = Bool()
        bmsg.data = True
        trigger_pub.publish(bmsg)

        r.sleep()
    sgpo.deinitialise()
