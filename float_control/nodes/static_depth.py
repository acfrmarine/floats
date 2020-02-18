#!/usr/bin/env python

import time
import argparse
import sys
import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node('static_depth')
    depth_pub = rospy.Publisher('/depth_target', Float32)

    r = rospy.Rate(5)  # 5 Hz

    count = 0

    while not rospy.is_shutdown():
        set_depth = rospy.get_param("set_target_depth", 0.0)
        msg = Float32()
        msg.data = set_depth
        depth_pub.publish(msg)

        count += 1

        r.sleep()