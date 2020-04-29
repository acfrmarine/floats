#!/usr/bin/env python
import time
import signal
import os
import sys
import warnings
import navio2.pwm


class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self,signum, frame):
    self.kill_now = True

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
        try:
            with open(self.SYS_UNEXPORT_PATH, "w") as gpio_disable:
                gpio_disable.write(str(self.pin))
            self.is_enabled = False
        except IOError:
            warnings.warn("Couldn't deinitialise")

    def output(self, value):
        with open(self.pin_path + "value", "w") as gpio_write:
            gpio_write.write(str(value))



if __name__ == "__main__":
    trigger_pin = 10
    pwm_ = navio2.pwm.PWM(trigger_pin - 1)
    pwm_.initialize()
    freq = 100.
    pwm_.set_period(freq);
    pwm_.enable()
    period = 1/freq
    startup_time = 20 
    killer = GracefulKiller()
    duty = 0.1  # 0-1
    duty_ms = period * duty * 1000
    time0 = time.time()
    while time.time() - time0 < startup_time and not killer.kill_now:
        pwm_.set_duty_cycle(duty_ms)
        # Sleep
        time.sleep(1.)
    pwm_.deinitialize()
    time.sleep(5) 
    print("staying high")
    sgpo = ServoGPO(trigger_pin)
    sgpo.initialise()
    sgpo.output(1)
    while not killer.kill_now:
        time.sleep(100)
    print("Done")
