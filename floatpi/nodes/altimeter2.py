#!/usr/bin/env python3

from brping import Ping1D
import rospy
from sensor_msgs.msg import Range
import numpy as np
import floatpi.srv
import std_srvs.srv

class Altimeter:
    def __init__(self):
        device = rospy.get_param("~device", "/dev/ttyAMA0")
        baudrate = rospy.get_param("~baudrate", 115200)
        sample_rate = rospy.get_param("~sample_rate", 5.0)
        speed_of_sound = rospy.get_param("~speed_of_sound", 1500.0)  # 1500m/s is the speed of sound in salt water, 1435 for fresh, 343 for air

        self.auto_mode = rospy.get_param("~auto_mode", True)
        self.range_min = rospy.get_param("~range_min", 0.1)
        self.range_max = rospy.get_param("~range_max", 30.0)

        self.range_pub = rospy.Publisher('ping', Range, queue_size=1)

        self.field_of_view = 30 * np.pi / 180.0

        # Old initialisation...
        self.pinger = Ping1D(device, baudrate)

        # New initialisation
        #self.pinger = Ping1D()
        #self.pinger.connect_serial(device, baudrate)

        if self.pinger.initialize() is False:
            rospy.logerr("Failed to initialize Ping")
            exit(1)

        self.pinger.set_speed_of_sound(int(speed_of_sound))

        if not self.auto_mode:
            self.pinger.set_mode_auto(0)
            scan_length = (self.range_max - self.range_min) *1000.0
            self.pinger.set_range(self.range_min*1000., scan_length)
            rospy.loginfo("Setting scan range to [%f, %f] (m)" %self.range_min, self.range_max)

        self.count = 0

        self.set_range_service = rospy.Service('set_pinger_range', floatpi.srv.SetPingerRange, self.setRangeCallback)
        self.set_mode_service = rospy.Service('set_pinger_mode', std_srvs.srv.SetBool, self.setModeCallback)
        print("Started services")
        rospy.Timer(rospy.Duration(1 / sample_rate), self.timerCallback)

    def setRangeCallback(self, req):
        self.range_min = req.range_min
        self.range_max = req.range_max
        self.pinger.set_mode_auto(0)
        scan_length = (self.range_max - self.range_min) * 1000.0
        self.pinger.set_range(int(self.range_min * 1000.), int(scan_length))
        rospy.loginfo("Setting pinger to auto mode - necessary for custom scan range")
        rospy.loginfo("Setting scan range to [%f, %f] (m)" % (self.range_min, self.range_max))
        res = floatpi.srv.SetPingerRangeResponse()
        res.success = True
        return res

    def setModeCallback(self, req):
        self.pinger.set_mode_auto(req.data)
        if req.data is True:
            mode = "Auto"
        else:
            mode = "Manual"
        res = std_srvs.srv.SetBoolResponse()
        res.success = True
        res.message = "Set mode to %s" % mode
        rospy.loginfo("Set pinger mode to %s" % mode)
        return res



    def timerCallback(self, event):
        data = self.pinger.get_distance()

        if data:
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = 'pinger'
            range_msg.header.seq = self.count
            range_msg.field_of_view = self.field_of_view
            range_msg.min_range = 0.5
            range_msg.max_range = 30.0
            range_msg.range = float(data["distance"])/1000.0
            self.range_pub.publish(range_msg)
        else:
            rospy.loginfo("Failed to get distance data")
        self.count += 1


if __name__ == "__main__":
    rospy.init_node('pinger')
    alt = Altimeter()
    rospy.spin()
