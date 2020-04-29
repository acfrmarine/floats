#!/usr/bin/env python

import time
import argparse
import sys
import rospy
from std_msgs.msg import Float32, Float64



class StaticCommand:
    """
    """
    def __init__(self):
        self.static_command_ = 0.0

        self.thruster_pub = rospy.Publisher('/thruster_cmd', Float64)

        rospy.Subscriber("/set_thruster_command", Float64, self.setCallback)

        rospy.Timer(rospy.Duration(0.2), self.timerCallback)

    def setCallback(self, msg):
        self.static_command_ = msg.data

    def timerCallback(self, event):
        msg = Float64()
        msg.data = self.static_command_
        self.thruster_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('static_command')

    sc = StaticCommand()

    rospy.spin()
