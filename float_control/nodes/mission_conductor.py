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
from std_srvs.srv import Empty

from float_msgs.srv import SendMission, SendMissionResponse

class MissionConductor:
    """
    """
    def __init__(self):
        self.on_mission = False
        self.at_max_depth = False
        # self.use_bottom_detection = rospy.get_param("~use_bottom_detection", False)
        self.mission_timeout = None
        self.target_altitude = None
        self.max_depth = None
        self.use_bottom_detection = False
        self.use_bottom_time = False
        self.bottom_time = None
        self.vary_altitude = False

        self.time0 = rospy.Time.now()

        self.vary_altitude_sign = 1
        self.vary_altitude_count = 0

        self.mission_service = rospy.Service('send_mission', SendMission, self.missionServiceCallback)
        self.abort_service = rospy.Service('abort_mission', Empty, self.abortServiceCallback)

        rospy.Subscriber("depth", Float64, self.depthCallback)
        rospy.Subscriber("ping", Range, self.pingCallback)
        rospy.Subscriber("depth_target", Float64, self.depthTargetCallback)

        rospy.Timer(rospy.Duration(1.0), self.timerCallback)

    def send_zero(self):
        rospy.wait_for_service('set_thruster_command')
        try:
            set_thruster_cmd_proxy = rospy.ServiceProxy('set_thruster_command', SetThrusterCommand)
            success = set_thruster_cmd_proxy(0.0,2.0) #  Set a zero command
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def set_target_altitude(self, alt, timeout):
        rospy.wait_for_service('set_altitude_target')
        try:
            set_altitude_proxy = rospy.ServiceProxy('set_altitude_target', SetAltitudeTarget)
            success = set_altitude_proxy(alt,timeout) #  Sets the controller to altitude target mode
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def set_target_depth(self, depth, timeout):
        rospy.wait_for_service('set_depth_target')
        try:
            set_depth_proxy = rospy.ServiceProxy('set_depth_target', SetDepthTarget)
            success = set_depth_proxy(depth,timeout) #  Sets the controller to depth target mode
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def timerCallback(self, event):
        time_diff = rospy.Time.now() - self.time0
        time_left = rospy.Duration(self.mission_timeout) - time_diff
        if time_left.to_sec() < 0.0:
            self.on_mission = False
            self.send_zero()
            # TODO drive to surface?
            return
        else:
            if self.depth >= self.max_depth:  # Check if at the max depth
                self.set_target_depth(self.max_depth, time_left)
                self.at_max_depth = False
            else:
                if self.vary_altitude:

                    if self.vary_altitude_count % 10 == 0:
                        if self.vary_altitude_sign == 1:
                            self.set_target_altitude(self.target_altitude+0.5, time_left)
                            self.vary_altitude_sign = 0
                        else:
                            self.set_target_altitude(self.target_altitude, time_left)
                            self.vary_altitude_sign = 0
                    self.vary_altitude_count += 1
                else:
                    self.set_target_altitude(self.target_altitude, time_left)


    def missionServiceCallback(self, req):
        self.mission_timeout = req.timeout
        self.target_altitude = req.target_altitude
        self.max_depth = req.max_depth
        self.use_bottom_detection = req.use_buttom_detection
        self.use_bottom_time = req.use_bottom_time
        self.bottom_time = req.bottom_time
        self.vary_altitude = req.vary_altitude
        self.time0 = rospy.Time.now()
        self.on_mission = True
        return SendMissionResponse(True)

    def abortServiceCallback(self, req):
        self.on_mission = False
        self.send_zero()


    def depthCallback(self, msg):
        self.depth = msg.data

    def pingCallback(self, msg):
        self.altitude = msg.range

if __name__ == "__main__":
    rospy.init_node('conductor', anonymous=True)
    conductor = MissionConductor()

    rospy.spin()
