#!/usr/bin/env python
import time
import sys
import os


class ServoGPO:
    SYS_EXPORT_PATH="/sys/class/gpio/export"
    SYS_UNEXPORT_PATH="/sys/class/gpio/unexport"
    SYS_GPIO_BASE_PATH="/sys/class/gpio/"

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
    sgpo = ServoGPO(2)
    sgpo.initialise()

    try:
        while True:
            sgpo.output(1)
            time.sleep(100)
    except KeyboardInterrupt:
        sgpo.deinitialise()
