#!/usr/bin/env python

import time
import argparse
import sys
import rospy
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Range

class AltitudeController:
    """
    Runs an altitude controller
    """
    def __init__(self):
        self.target_altitude = None
        self.depth = None
        self.altitude = None

        self.tgtDepthPub = rospy.Publisher("depth_target", Float64, queue_size=1)

        rospy.Subscriber("altitude_target", Float64, self.targetCallback)
        rospy.Subscriber("depth", Float64, self.depthCallback)
        rospy.Subscriber("ping", Range, self.pingCallback)


    def targetCallback(self, msg):
        self.target_altitude = msg.data

    def depthCallback(self, msg):
        self.depth = msg.data

    def pingCallback(self, msg):
        self.altitude = msg.range
        self.publishTargetDepth()

    def publishTargetDepth(self):
        # current_depth + current_altitude = target_depth + target_altitude
        # target_depth = current_depth + current_altitude - target_altitude
        if self.depth is None:
            rospy.logwarn("Altitude controller isn't receiving depth")
            return
        elif self.target_altitude is None:
            rospy.logwarn("Target altitude hasn't been received - keeping the current depth")
            tgt_depth = self.depth
        elif self.altitude is None:
            rospy.logwarn("Not receiving altitude, keeping current depth")
            tgt_depth = self.depth
        else:
            tgt_depth =  self.depth + self.altitude - self.target_altitude
        depth_msg = Float64()
        depth_msg.data = tgt_depth
        self.tgtDepthPub.publish(depth_msg)

if __name__ == "__main__":
    rospy.init_node('altitude_controller', anonymous=True)
    altcontrol = AltitudeController()

    rospy.spin()
