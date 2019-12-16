#!/usr/bin/env python

import spidev
import time
import argparse
import sys
import navio2.mpu9250
import navio2.util

import rospy
from sensor_msgs.msg import Imu, MagneticField

if __name__ == "__main__":
    rospy.init_node('pressure_sensor_node')
    imu_pub_ned = rospy.Publisher('/navio/imu_ned', Imu, queue_size=1)
    imu_pub = rospy.Publisher('/navio/imu', Imu, queue_size=1)
    mag_pub_ned = rospy.Publisher('/navio/magnetic_field_ned', MagneticField, queue_size=1)
    mag_pub = rospy.Publisher('/navio/magnetic_field', MagneticField, queue_size=1)


    navio2.util.check_apm()

    imu_sensor = rospy.get_param('imu_sensor', 'mpu')

    if imu_sensor == 'mpu':
        print("Selected: MPU9250")
        imu = navio2.mpu9250.MPU9250()
    elif imu_sensor == 'lsm':
        print("Selected: LSM9DS1")
        imu = navio2.lsm9ds1.LSM9DS1()
    else:
        print("Wrong sensor name. Select: mpu or lsm")
        sys.exit(1)



    if imu.testConnection():
        print("Connection established: True")
    else:
        sys.exit("Connection established: False")

    imu.initialize()

    r = rospy.Rate(100) # 100 Hz

    count = 0

    while not rospy.is_shutdown():
        m9a, m9g, m9m = imu.getMotion9()

        imu_msg_ned = Imu()
        imu_msg_ned.header.stamp = rospy.Time.now()
        imu_msg_ned.header.frame_id = 'imu'
        imu_msg_ned.header.seq = count
        imu_msg_ned.orientation_covariance = [-1,0,0,0,0,0,0,0,0]  # Set invalid
        imu_msg_ned.angular_velocity.x = m9g[0]
        imu_msg_ned.angular_velocity.y = m9g[1]
        imu_msg_ned.angular_velocity.z = m9g[2]
        imu_msg_ned.linear_acceleration.x = m9a[0]
        imu_msg_ned.linear_acceleration.y = m9a[1]
        imu_msg_ned.linear_acceleration.z = m9a[2]
        imu_pub_ned.publish(imu_msg_ned)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu'
        imu_msg.header.seq = count
        imu_msg.orientation_covariance = [-1,0,0,0,0,0,0,0,0]  # Set invalid
        imu_msg.angular_velocity.x = m9g[1]
        imu_msg.angular_velocity.y = -m9g[0]
        imu_msg.angular_velocity.z = -m9g[2]
        imu_msg.linear_acceleration.x = m9a[1]
        imu_msg.linear_acceleration.y = -m9a[0]
        imu_msg.linear_acceleration.z = -m9a[2]
        imu_pub.publish(imu_msg)

        
        
        mag_msg_ned = MagneticField()
        mag_msg_ned.header.stamp = rospy.Time.now()
        mag_msg_ned.header.frame_id = 'imu'
        mag_msg_ned.header.seq = count
        mag_msg_ned.magnetic_field.x = m9m[0]  # TODO Check NED, ENU
        mag_msg_ned.magnetic_field.y = m9m[1]
        mag_msg_ned.magnetic_field.z = m9m[2]
        mag_pub_ned.publish(mag_msg_ned)

        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = 'imu'
        mag_msg.header.seq = count
        mag_msg.magnetic_field.x = m9m[1]  # TODO Check NED, ENU
        mag_msg.magnetic_field.y = -m9m[0]
        mag_msg.magnetic_field.z = -m9m[2]
        mag_pub.publish(mag_msg)


        
        count += 1

        r.sleep()
