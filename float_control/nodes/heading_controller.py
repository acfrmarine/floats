#!/usr/bin/env python

import time
import argparse
import sys
import numpy as np
import rospy
import math
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import Range, MagneticField
from float_control.srv import SetAltitudeTarget,SetAltitudeTargetResponse
from float_control.srv import SetDepthTarget,SetDepthTargetResponse
from float_control.srv import SetThrusterCommand,SetThrusterCommandResponse
from float_msgs.srv import SetHeadingTarget,SetHeadingTargetResponse
from std_srvs.srv import SetBool


def angdiff(a, b):
    diff = a - b
    while diff > np.pi:
        diff -= 2 * np.pi
    while diff < -np.pi:
        diff += 2 * np.pi
    return diff


class HeadingController:
    """
    Runs an differential controller
    """
    def __init__(self):
        self.enabled = True
        self.pid_enable = False

        # self.thruster_cmd_max = rospy.get_param('~control_freq', 0.2)
        self.thruster_offset_max = rospy.get_param('~max_offset', 0.025)
        self.offset_kp = rospy.get_param('~offset_kp', 0.05)

        self.heading = None
        self.heading_target = None

        self.last_heading_time = rospy.Time()
        self.no_sensor_count = 0


        self.enable_heading_control = rospy.Service('enable_heading_control', SetBool, self.altitudeCmdServiceCallback)
        self.heading_set_service = rospy.Service('set_heading_target', SetHeadingTarget, self.setHeadingTargetServiceCallback)

        self.leftThrusterPub = rospy.Publisher("thruster_cmd_left", Float64, queue_size=1)
        self.rightThrusterPub = rospy.Publisher("thruster_cmd_right", Float64, queue_size=1)

        rospy.Subscriber("/navio/magnetic_field_cal", MagneticField, self.magCallback)
        rospy.Subscriber("/pid_enable", Bool, self.pidEnableCallback)
        rospy.Subscriber("/thruster_cmd", Float64, self.thrusterCmdCallback)


    def pidEnableCallback(self, msg):
        self.pid_enable = msg.data


    def thrusterCmdCallback(self, msg):
        if self.enabled and self.pid_enable and self.heading is not None and self.heading_target is not None:  # Heading controller has to be enabled and in depth control mode (which is pid enabled)
            diff = angdiff(self.heading_target, self.heading)
            offset = np.clip(diff * self.offset_kp, -self.thruster_offset_max, self.thruster_offset_max)
            if msg.data >= 0: # thrusting down, pushing water up
                right_cmd = msg.data - offset
                left_cmd = msg.data - offset
            else:
                right_cmd = msg.data + offset
                left_cmd = msg.data - offset

            right_msg = Float64()
            right_msg.data = right_cmd
            left_msg = Float64()
            left_msg.data = left_cmd
            self.leftThrusterPub.publish(left_msg)
            self.rightThrusterPub.publish(right_msg)
        else:  # In thruster control mode - republish the thruster cmd values
            self.leftThrusterPub.publish(msg)
            self.rightThrusterPub.publish(msg)



    def setHeadingTargetServiceCallback(self, req):
        self.heading_target = req.heading_target
        return SetHeadingTargetResponse(True)

    def magCallback(self, msg):
        self.heading = math.atan2(msg.magnetic_field.x, msg.magnetic_field.z)
        self.last_heading_time = rospy.Time.now()

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    controller = HeadingController()

    rospy.spin()
