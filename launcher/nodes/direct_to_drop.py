#!/usr/bin/env python

import numpy as np
import pymap3d
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

class Orientor:
    def __init__(self):
        self.tgt_lat = rospy.get_param("~latitude")
        self.tgt_lon = rospy.get_param("~longitude")
        self.got_gps = False

        self.headingPub = rospy.Publisher("heading_to_drop", Float64, queue_size=1)
        self.distancePub = rospy.Publisher("distance_to_drop", Float64, queue_size=1)

        rospy.Subscriber("/navio/gps", NavSatFix, self.gpsCallback)

        rospy.Timer(rospy.Duration(10.0), self.gpsCallback)

    def gpsCallback(self, msg):
        self.got_gps = True
        north,east,down = pymap3d.geodetic2ned(msg.latitude, msg.longitude, 0.0, lat0=self.tgt_lat, lon0=self.tgt_lon, h0=0.0)
        theta = np.atan2(0.0 - north, 0.0 - east)
        heading = theta * np.pi / 180.0
        dist = np.sqrt(north**2 + east**2)
        dist_msg = Float64()
        dist_msg.data = dist
        heading_msg = Float64()
        heading_msg.data = heading
        self.headingPub.publish(heading_msg)
        self.distancePub.publish(dist_msg)
        print("Heading: %f, Distance: %f")

        # east,north,up = pymap3d.geodetic2enu(msg.latitude, msg.longitude, 0.0, lat0=self.tgt_lat, lon0=self.tgt_lon, h0=0.0)
        # theta = np.atan2(0.0 - east, 0.0 - north)




if __name__ == "__main__":
    rospy.init_node('orientor')
    orientor = Orientor()
    rospy.spin()


