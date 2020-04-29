#!/usr/bin/env python3

from brping import Ping1D
import rospy
from sensor_msgs.msg import Range
import numpy as np

if __name__ == "__main__":
    rospy.init_node('pinger')
    range_pub = rospy.Publisher('ping', Range, queue_size=1)
    device = rospy.get_param("~device", "/dev/ttyAMA0")
    baudrate = rospy.get_param("~baudrate", 115200)
    sample_rate = rospy.get_param("~sample_rate", 10)

    # Old initialisation...
    #myPing = Ping1D(device, baudrate)

    # New initialisation
    myPing = Ping1D()
    myPing.connect_serial(device, baudrate)
    if myPing.initialize() is False:
        rospy.logerr("Failed to initialize Ping")
        exit(1)
    count = 0
    r = rospy.Rate(sample_rate)  # 100 Hz
    field_of_view = 30 * np.pi / 180.0
    # Read and print distance measurements with confidence
    while True:
        data = myPing.get_distance()

        if data:
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = 'pinger'
            range_msg.header.seq = count
            range_msg.field_of_view = field_of_view
            range_msg.min_range = 0.5
            range_msg.max_range = 30.0
            range_msg.range = float(data["distance"])/1000.0
            range_pub.publish(range_msg)
        else:
            rospy.loginfo("Failed to get distance data")
        count += 1
        r.sleep()
