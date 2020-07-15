#!/usr/bin/env python

import time
import argparse
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import MagneticField

class ProcessCompass:
    """
    Converts the raw compass value to a calibrated one.
    """
    def __init__(self):
        self.magPub = rospy.Publisher("/navio/magnetic_field_cal", MagneticField, queue_size=1)

        self.offset_x = rospy.get_param("~mag_offset_x", 0.0)
        self.offset_y = rospy.get_param("~mag_offset_y", 0.0)
        self.offset_z = rospy.get_param("~mag_offset_z", 0.0)

        self.radius = rospy.get_param("~mag_radius", 1.0)

        rospy.Subscriber("/navio/magnetic_field", MagneticField, self.rawCallback)

    def rawCallback(self, msg):
        pmsg = msg
        pmsg.magnetic_field.x = (pmsg.magnetic_field.x - self.offset_x)/self.radius
        pmsg.magnetic_field.y = (pmsg.magnetic_field.y - self.offset_y)/self.radius
        pmsg.magnetic_field.z = (pmsg.magnetic_field.z - self.offset_z)/self.radius
        self.magPub.publish(pmsg)

if __name__ == "__main__":
    rospy.init_node('compass_process', anonymous=True)
    processor = ProcessCompass()

    rospy.spin()
