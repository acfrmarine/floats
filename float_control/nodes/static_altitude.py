#!/usr/bin/env python

import time
import argparse
import sys
import rospy
from std_msgs.msg import Float32, Float64


if __name__ == "__main__":
    rospy.init_node('static_altitude')
    alt_pub = rospy.Publisher('altitude_target', Float64)

    r = rospy.Rate(5)  # 5 Hz

    count = 0

    while not rospy.is_shutdown():
        set_altitude = rospy.get_param("set_target_altitude", 0.8)
        msg = Float64()
        msg.data = set_altitude
        alt_pub.publish(msg)

        count += 1

        r.sleep()
