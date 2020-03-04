#!/usr/bin/env python

import time
import argparse
import sys
import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node('static_command')
    thruster_pub = rospy.Publisher('/thruster_cmd', Float32)

    r = rospy.Rate(5)  # 5 Hz

    count = 0

    while not rospy.is_shutdown():
        set_command = rospy.get_param("set_thruster_command", 0.0)
        msg = Float32()
        msg.data = set_command
        thruster_pub.publish(msg)

        count += 1

        r.sleep()
