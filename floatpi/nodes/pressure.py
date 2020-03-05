#!/usr/bin/env python
import ms5837
import time
import rospy
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Vector3Stamped



if __name__ == "__main__":
    rospy.init_node('pressure_sensor_node')
    fluid_pub = rospy.Publisher('fluid', FluidPressure, queue_size=1)
    depth_stamped_pub = rospy.Publisher('depth_stamped', Vector3Stamped, queue_size=1)
    depth_pub = rospy.Publisher('depth', Float64, queue_size=1)
    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=1)

    sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

    # We must initialize the sensor before reading it
    if not sensor.init():
            print("Sensor could not be initialized")
            exit(1)
    # We have to read values from sensor to update pressure and temperature
    if not sensor.read():
        print("Sensor read failed!")
        exit(1)

    fluid_density = rospy.get_param('fluid_density', 1028)
    fluid_density = rospy.get_param('fluid_density', 1000)

    r = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        if sensor.read():
            pressure = sensor.pressure()
            temp = sensor.temperature()
            depth = sensor.depth()

            pressure_msg = FluidPressure()
            pressure_msg.header.frame_id = 'pressure_sensor'
            pressure_msg.header.seq = count
            pressure_msg.header.stamp = rospy.Time.now()
            pressure_msg.fluid_pressure = pressure
            fluid_pub.publish(pressure_msg)

            depth_stamped_msg = Vector3Stamped()
            depth_stamped_msg.header.frame_id = 'pressure_sensor'
            depth_stamped_msg.header.seq = count
            depth_stamped_msg.header.stamp = rospy.Time.now() #TODO fix this
            depth_stamped_msg.vector.z = depth  # TODO this
            depth_stamped_pub.publish(depth_stamped_msg)

            depth_msg = Float64()
            depth_msg.data = depth
            depth_pub.publish(depth_msg)

            temp_msg = Temperature()
            temp_msg.header.frame_id = 'pressure_sensor'
            temp_msg.header.seq = count
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.temperature = temp
            temp_pub.publish(temp_msg)

            count += 1

        r.sleep()
