#!/usr/bin/env python
import ms5837
import time
import rospy
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Vector3Stamped
import std_srvs.srv
import floatpi.srv


class PressureSensor:
    def __init__(self):
        self.fluid_pub = rospy.Publisher('fluid', FluidPressure, queue_size=1)
        self.depth_stamped_pub = rospy.Publisher('depth_stamped', Vector3Stamped, queue_size=1)
        self.depth_pub = rospy.Publisher('depth', Float64, queue_size=1)
        self.temp_pub = rospy.Publisher('temperature', Temperature, queue_size=1)

        self.sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

        self.depth = None
        self.depth_raw = None

        self.count = 0

        # We must initialize the sensor before reading it
        if not self.sensor.init():
                print("Sensor could not be initialized")
                exit(1)
        # We have to read values from sensor to update pressure and temperature
        if not self.sensor.read():
            print("Sensor read failed!")
            exit(1)

        fluid_density = rospy.get_param('~fluid_density', 1028)
        self.depth_offset = rospy.get_param('~depth_offset', 0.0)
        sample_rate = float(rospy.get_param('~sample_rate', 10.0))
        self.sensor.setFluidDensity(fluid_density)

        self.tare_service = rospy.Service('tare_depth_sensor', std_srvs.srv.SetBool, self.tareCallback)
        self.depth_offset_service = rospy.Service('set_depth_sensor_offset', floatpi.srv.SetDepthOffset, self.setOffsetCallback)

        rospy.Timer(rospy.Duration(1/sample_rate), self.timerCallback)

    def timerCallback(self, event):
        if self.sensor.read():
            pressure = self.sensor.pressure()
            temp = self.sensor.temperature()
            self.depth_raw = self.sensor.depth()
            depth = self.depth_raw - self.depth_offset
            self.depth = depth

            pressure_msg = FluidPressure()
            pressure_msg.header.frame_id = 'pressure_sensor'
            pressure_msg.header.seq = self.count
            pressure_msg.header.stamp = rospy.Time.now()
            pressure_msg.fluid_pressure = pressure
            self.fluid_pub.publish(pressure_msg)

            depth_stamped_msg = Vector3Stamped()
            depth_stamped_msg.header.frame_id = 'pressure_sensor'
            depth_stamped_msg.header.seq = self.count
            depth_stamped_msg.header.stamp = rospy.Time.now() #TODO fix this
            depth_stamped_msg.vector.z = depth  # TODO this
            self.depth_stamped_pub.publish(depth_stamped_msg)

            depth_msg = Float64()
            depth_msg.data = depth
            self.depth_pub.publish(depth_msg)

            temp_msg = Temperature()
            temp_msg.header.frame_id = 'pressure_sensor'
            temp_msg.header.seq = self.count
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.temperature = temp
            self.temp_pub.publish(temp_msg)

            self.count += 1

    def tareCallback(self, req):
        if self.depth is not None:
            self.depth_offset = self.depth
            resp = std_srvs.srv.SetBoolResponse()
            resp.success = True
            resp.message = "Depth offset set to %f" % float(self.depth_raw)
        else:
            resp = std_srvs.srv.SetBoolResponse()
            resp.success = False
            resp.message = "No depth value yet, couldn't tare"
        return resp

    def setOffsetCallback(self, req):
        self.depth_offset = req.depth_offset
        res = floatpi.srv.SetDepthOffsetResponse()
        res.success = True
        return res



if __name__ == "__main__":
    rospy.init_node('pressure_sensor_node')
    pressure = PressureSensor()
    rospy.spin()
