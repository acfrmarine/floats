#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Bool
import os


class ServoGPI:
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
            gpio_direction.write("in")

    def deinitialise(self):
        with open(self.SYS_UNEXPORT_PATH, "w") as gpio_disable:
            gpio_disable.write(str(self.pin))
            self.is_enabled = False

    def read(self):
        with open(self.pin_path + "value", "r") as gpio_read:
            value = gpio_read.read()
        return value.strip()

if __name__ == "__main__":
    rospy.init_node('switch_node', anonymous=True)
    topic = rospy.get_param("topic", "switch")
    switch_pub = rospy.Publisher(topic, Bool, queue_size=1)
    switch_pin = rospy.get_param('~switch_pin', 4)  # Corresponds to the Servo rails
    sample_rate = rospy.get_param('~sample_rate', 2)  # in Hz. Defaults at 2Hz

    sgpi = ServoGPI(switch_pin)
    sgpi.initialise()

    r = rospy.Rate(sample_rate) # hz

    while not rospy.is_shutdown():
        # Write output high
        value = sgpi.read()
        bmsg = Bool()
        print("Value", value)
        bmsg.data = bool(int(value))
        switch_pub.publish(bmsg)

        r.sleep()
    sgpi.deinitialise()
