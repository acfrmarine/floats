#!/usr/bin/env python
import os
import sys
import subprocess
import rospy
from sensor_msgs.msg import BatteryState

if __name__ == "__main__":
    rospy.init_node('navio_ups_node')
    pub = rospy.Publisher('navio/ups', BatteryState, queue_size=1)

    r = rospy.Rate(1) # 1hz
    count = 0
    while not rospy.is_shutdown():
        out = subprocess.check_output(['upsc', 'openups@localhost'])
        outd = out.decode("utf-8")
        lines = outd.split('\n')

        bsmsg = BatteryState()

        bsmsg.header.stamp = rospy.Time.now()
        bsmsg.header.frame_id = 'base_link'
        bsmsg.header.seq = count

        count += 1

        # voltage, current, charge capacity, design_capacity, percentage

        for li in lines:
            if "battery.capacity" in li:
                bsmsg.capacity = float(li.split(' ')[-1])
            elif "battery.charge" in li:
                bsmsg.charge = float(li.split(' ')[-1])
            elif "battery.current" in li:
                bsmsg.current = float(li.split(' ')[-1])
            elif "battery.voltage" in li:
                bsmsg.voltage = float(li.split(' ')[-1])
            elif "device.serial" in li:
                serial_number = str(li.split(' ')[-1])
            elif "device.input_current" in li:
                device_input_current = float(li.split(' ')[-1])
            elif "device.output_current" in li:
                device_output_current = float(li.split(' ')[-1])
            elif "device.input_voltage" in li:
                device_input_voltage = float(li.split(' ')[-1])
            elif "device.output_voltage" in li:
                device_output_voltage = float(li.split(' ')[-1])

        try:
            bsmsg.percentage = bmsg.charge/bmsg.capacity
        except:
            pass
        try:
            if device_input_current - device_output_current > 0.0:
                bsmsg.power_supply_status = bsmsg.POWER_SUPPLY_STATUS_CHARGING
                # TODO check for full
            else:
                bsmsg.power_supply_status = bsmsg.POWER_SUPPLY_STATUS_DISCHARGING
        except:
            pass

        pub.publish(bsmsg)


        r.sleep()
