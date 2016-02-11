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

class UnifiedController:
    """
    Runs an thruster/depth/altitude controller
    """
    def __init__(self):
        self.mode = 'thruster'  # Start off in thruster control mode
        self.timeout = rospy.Duration(0.0)

        self.control_freq = rospy.get_param('~control_freq', 5.0)
        self.use_lag_control = rospy.get_param('~use_lag_control', False)
        self.ping_lag = rospy.get_param('~ping_lag', 2.0)  # Gets the ping lag, in seconds.

        self.thruster_set = 0.0
        self.depth_set = None
        self.altitude_set = None

        self.depth = None
        self.altitude = None

        # These are for checking new sensor messages are being received
        self.new_depth = False
        self.new_altitude = False
        self.last_altitude_time = rospy.Time()
        self.last_depth_time = rospy.Time()
        self.last_heading_time = rospy.Time()
        self.no_sensor_count = 0

        # Calculating the 2s lag value
        self.times_list = []
        self.depths_list = []

        self.time0 = rospy.Time.now()

        self.thruster_cmd_service = rospy.Service('set_thruster_command', SetThrusterCommand, self.thrusterCmdServiceCallback)
        self.depth_cmd_service = rospy.Service('set_depth_target', SetDepthTarget, self.depthCmdServiceCallback)
        self.altitude_cmd_service = rospy.Service('set_altitude_target', SetAltitudeTarget, self.altitudeCmdServiceCallback)

        self.tgtDepthPub = rospy.Publisher("depth_target", Float64, queue_size=1)
        self.thrusterPub = rospy.Publisher("thruster_cmd", Float64, queue_size=1)

        self.pidEnablePub = rospy.Publisher("pid_enable", Bool, queue_size=1)

        rospy.Subscriber("depth", Float64, self.depthCallback)
        rospy.Subscriber("ping", Range, self.pingCallback)

        rospy.Timer(rospy.Duration(1/self.control_freq), self.timerCallback)


    def timerCallback(self, event):
        """
        A regular timer to publish the desired control commands.

        Args:
            event: (rospy.TimerEvent) Information about timing. Not used.
        """
        if rospy.Time.now() - self.time0 >= self.timeout:
            rospy.logwarn("Current command timed out. Changing to 'thruster' mode with zero command")
            self.mode = 'thruster'
            self.thruster_set = 0.0

        if self.mode == "thruster":
            # Disable the PID controller - this is used to control on depth
            disable_msg = Bool()
            disable_msg.data = False
            self.pidEnablePub.publish(disable_msg)
            cmd_msg = Float64()
            cmd_msg.data = self.thruster_set
            self.thrusterPub.publish(cmd_msg)
        elif self.mode == "depth":
            if self.new_depth:
                # Enable the PID controller
                enable_msg = Bool()
                enable_msg.data = True
                self.pidEnablePub.publish(enable_msg)
                tgt_msg = Float64()
                tgt_msg.data = self.depth_set
                self.tgtDepthPub.publish(tgt_msg) # This goes to PID controller
                self.no_sensor_count = 0
            else:
                self.no_sensor_count += 1
                if self.no_sensor_count > 5:
                    rospy.logwarn("Sensor readings not coming in - last_depth=%f, last_altitude=%f" %((rospy.Time.now() - self.last_depth_time).to_sec(), (rospy.Time.now() - self.last_altitude_time).to_sec()))
                    rospy.logwarn("Setting to 'thruster' mode with zero command")
                    self.mode = 'thruster'
                    self.thruster_set = 0.0
        elif self.mode == "altitude":
            if self.new_depth and self.new_altitude:
                # Enable the PID controller
                enable_msg = Bool()
                enable_msg.data = True
                self.pidEnablePub.publish(enable_msg)
                tgt_depth = self.calc_target_depth()
                if tgt_depth is None:
                    rospy.logwarn("tgt_depth is 'None'. Setting mode 'thruster' mode with zero command")
                    self.mode = 'thruster'
                    self.thruster_set = 0.0
                tgt_msg = Float64()
                tgt_msg.data = tgt_depth
                self.tgtDepthPub.publish(tgt_msg) # This goes to PID controller
                self.no_sensor_count = 0
            else:
                self.no_sensor_count += 1
                if self.no_sensor_count > 5:
                    rospy.logwarn("Sensor readings not coming in - last_depth=%f, last_altitude=%f" %((rospy.Time.now() - self.last_depth_time).to_sec(), (rospy.Time.now() - self.last_altitude_time).to_sec()))
                    rospy.logwarn("Setting to 'thruster' mode with zero command")
                    self.mode = 'thruster'
                    self.thruster_set = 0.0
        else:
            rospy.logwarn("Mode %s not supported" % self.mode)
            rospy.logwarn("Setting to 'thruster' mode with zero command")
            self.mode = 'thruster'
            self.thruster_set = 0.0

    def thrusterCmdServiceCallback(self, req):
        self.timeout = rospy.Duration(req.timeout)
        self.time0 = rospy.Time.now()
        self.mode = 'thruster'
        self.thruster_set = req.thruster_command
        return SetThrusterCommandResponse(True)

    def depthCmdServiceCallback(self, req):
        self.timeout = rospy.Duration(req.timeout)
        self.time0 = rospy.Time.now()
        self.mode = 'depth'
        self.depth_set = req.depth_target
        return SetDepthTargetResponse(True)

    def altitudeCmdServiceCallback(self, req):
        self.timeout = rospy.Duration(req.timeout)
        self.time0 = rospy.Time.now()
        self.mode = 'altitude'
        self.altitude_set = req.altitude_target
        return SetAltitudeTargetResponse(True)


    def depthCallback(self, msg):
        self.depth = msg.data
        self.new_depth = True
        self.last_depth_time = rospy.Time.now()
        if self.use_lag_control:
            self.depths_list.append(msg.data)
            self.times_list.append(rospy.Time.now().to_sec())

    def pingCallback(self, msg):
        self.altitude = msg.range
        self.new_altitude = True
        self.last_altitude_time = rospy.Time.now()

    def calc_target_depth(self):
        # current_depth + current_altitude = target_depth + target_altitude
        # target_depth = current_depth + current_altitude - target_altitude
        if self.depth is None:
            rospy.logwarn("Altitude controller isn't receiving depth")
            return None
        elif self.altitude_set is None:
            # rospy.logwarn("Target altitude hasn't been received - keeping the current depth")
            # tgt_depth = self.depth
            rospy.logwarn("Target altitude hasn't been received, error")
            return None
        elif self.altitude is None:
            # rospy.logwarn("Not receiving altitude, keeping current depth")
            # tgt_depth = self.depth
            rospy.logwarn("Altitude hasn't been received, error")
            return None
        else:
            if self.use_lag_control:  # Tries to compensate for ping lag
                # Finds the index of the lag depth measurement
                idx_lag = np.abs(np.array(self.times_list) - rospy.Time.now().to_sec() + self.ping_lag).argmin()
                # Gets the depth value at ping lag
                depth_lag = self.depths_list[idx_lag]
                # Calculates the target depth using this lag value
                tgt_depth = depth_lag + self.altitude - self.altitude_set
            else:
                tgt_depth = self.depth + self.altitude - self.altitude_set
        return tgt_depth

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    controller = UnifiedController()

    rospy.spin()
