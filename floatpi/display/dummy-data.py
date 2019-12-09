#!/usr/bin/python

import time
import os
import numpy
import rospy
import sys
import csv
import cv2
import numpy as np
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Vector3, Vector3Stamped
from cv_bridge import CvBridge, CvBridgeError
import PIL.Image
from cv_bridge import CvBridge

if __name__  == "__main__":
    rospy.init_node('dummy_data', anonymous=True)

    r = rospy.Rate(10)  # 10 Hz

    navpub = rospy.Publisher('/navio/gps', NavSatFix, queue_size=1)
    battery_pub = rospy.Publisher('/navio/ups', BatteryState, queue_size=1)
    depth_pub = rospy.Publisher('/depth', Vector3Stamped, queue_size=1)
    img_pub = rospy.Publisher('/stereo/right/image_raw', Image, queue_size=1)
    rec_pub = rospy.Publisher('/recording', Bool, queue_size=1)
    log_pub = rospy.Publisher('/logs', String, queue_size=1)

    log_options = ['Starting logs to /media/data/logs.bag', 'Running all the logs', 'Finishing logs /media/data/logs.bag', 'Shutting down']
    # log_options = ['Running all the logs', 'Shutting down', 'Starting up', 'Shutting down', 'Starting up']
    count = 0
    log_idx = 0

    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Publish lat,lon
        navmsg = NavSatFix()
        navmsg.latitude = np.random.uniform(-33.0,-32.0)
        navmsg.longitude = np.random.uniform(151.0,152.0)
        navmsg.altitude = np.random.uniform(-5.0,0.0)
        navpub.publish(navmsg)

        # Publish battery state
        batmsg = BatteryState()
        batmsg.voltage = np.random.uniform(14.5,15.5)
        batmsg.current = np.random.uniform(0.0, 2.0)
        batmsg.capacity = np.random.uniform(0.0, 1.0)
        battery_pub.publish(batmsg)

        # Depth
        vmsg = Vector3Stamped()
        vmsg.header.frame_id = 'header'
        vmsg.header.seq = count
        vmsg.header.stamp = rospy.Time.now()
        vmsg.vector.z = np.random.uniform(0, 5.0)
        depth_pub.publish(vmsg)

        # Image
        path="/home/auv/Pictures/sirius.png"
        img = cv2.imread(path, 1)
        img = img.astype(np.uint8)
        imgmsg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        img_pub.publish(imgmsg)

        # Logs
        if count % 10 == 0:
            log_msg = String()
            log_msg.data = log_options[log_idx]
            log_pub.publish(log_msg)
            log_idx += 1
            if log_idx >= len(log_options) - 1:
                log_idx = 0
            print("loggings")


        # Recording
        rmsg = Bool()
        rmsg.data = bool(np.round(np.random.uniform()))
        rec_pub.publish(rmsg)

        count += 1

        r.sleep()

    rospy.spin()



