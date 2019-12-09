#!/usr/bin/python

import thread
import time
import os
import subprocess
import serial
import math
import pygame
import numpy
from glyph import Glyph, Macros
import rospy
import sys
import csv
import cv2
from operator import itemgetter
from math import radians, cos, sin, asin, sqrt, atan2, pi
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Vector3, Vector3Stamped
from cv_bridge import CvBridge, CvBridgeError


class D3Display:
    def __init__(self):
        # Get origin
        origin_path = rospy.get_param("~origin_path", "")  # TODO add loading of origin
        self.recording_state = False  # TODO make a recording button

        # Display
        size = ( 800, 480 )  # Size of WaveRacer 4inch screen
        pygame.init()
        self.display = pygame.display.set_mode( size, 0)
        self.box_font = pygame.font.SysFont('monospaced', 30)
        self.side_rect = pygame.Rect(400, 0, 800, 480)
        self.bottom_rect = pygame.Rect(0, 400, 800, 480)
        # Side box displays depth, heading, location etc.
        self.side_box = Glyph(self.side_rect, font=self.box_font)
        # Bottom box holds logs
        self.bottom_box = Glyph(self.bottom_rect, font=self.box_font)
        Macros['L'] = ('font', pygame.font.SysFont('monospaced', 50))
        Macros['M'] = ('font', pygame.font.SysFont('monospaced', 40))
        Macros['S'] = ('font', pygame.font.SysFont('monospaced', 30))
        Macros['red'] = ('color', (255, 0, 0))
        Macros['green'] = ('color', (0, 255, 0))
        Macros['blue'] = ('color', (0, 0, 255))

        # Variables
        self.H = 0.  # ???
        self.A = 0.  # ???
        self.depth = 0.  # ???
        self.gps_msg = NavSatFix()
        self.battery_state = BatteryState()
        self.R = 0.0  # ???
        self.P = 0.0  # ???

        self.cpu = 0.0  # ???
        self.hdd = 0.0  # ???
        self.upt = 0.0  # ???

        self.recording_state = False

        self.img = None
        self.log_list = []

        self.bridge = CvBridge()

        # Setup timer to update display
        screen_hz = rospy.get_param("~screen_hz", 2.0)
        rospy.Timer(rospy.Duration(screen_hz), self.screenUpdateCallback)

        # Subscribers
        compass_topic = "/navio/compass"
        gps_topic = "/navio/gps"
        log_topic = "/logs"
        img_topic = "/image_small"
        battery_topic = "/navio/ups"
        depth_topic = "/depth"
        rec_topic = '/recording'
        rospy.Subscriber(img_topic, Image, self.imgCallback)
        rospy.Subscriber(gps_topic, NavSatFix, self.gpsCallback)
        rospy.Subscriber(compass_topic, Float32, self.compassCallback)
        rospy.Subscriber(log_topic, String, self.logCallback)
        rospy.Subscriber(battery_topic, BatteryState, self.batteryCallback)
        rospy.Subscriber(depth_topic, Vector3Stamped, self.depthCallback)
        rospy.Subscriber(rec_topic, Bool, self.recCallback)

    def recCallback(self, msg):
       self.recording_state = msg.data

    def logCallback(self, msg):
        self.log_list.append(str(msg.data))


    def screenUpdateCallback(self, event):
        # Side display - location, heading, depth etc.
        dispstr = "{{L; H:\t{}\n}}".format(self.R)
        dispstr += "{{L; A:\t{}\n}}".format(self.A)
        dispstr += "{{L; D:\t{}\n}}".format(self.depth)
        dispstr += "{{S; LAT:\t{}\n}}".format(self.gps_msg.latitude)
        dispstr += "{{S; LON:\t{}\n}}".format(self.gps_msg.longitude)
        dispstr += "{{S; R:\t{}\n}}".format(self.R)
        dispstr += "{{S; P:\t{}\n}}".format(self.P)
        dispstr += "{} \n".format(1)
        dispstr += "{{S; BAT: {}\n}}".format(self.battery_state.charge)
        dispstr += "{{S; CPU: {}\n}}".format(self.cpu)
        dispstr += "{{S; HDD: {}\n}}".format(self.hdd)
        dispstr += "{{S; Uptime: {}\n}}".format(self.upt)

        dispstr += "{{L; Rec:\t{}\n}}".format(self.recording_state)
        
        self.side_box.clear()
        self.side_box.input(dispstr)
        self.side_box.update()
        self.display.blit(self.side_box.image, self.side_rect)

        # Bottom box - logs
        if len(self.log_list) > 0:
            if len(self.log_list) > 3:
                self.log_list = self.log_list[-3:]
            for n, log in enumerate(self.log_list):
                if n == 0:
                    # disp_logs = "Log:\t{}".format(log)
                    disp_logs = "{{S; {}\n}}".format(log.replace('/', '|'))
                else:
                    # disp_logs += "Log:\t{}".format(log)
                    disp_logs += "{{S; {}\n}}".format(log.replace('/', '|'))
            self.bottom_box.clear()
            self.bottom_box.input(disp_logs)
            self.bottom_box.update()
            self.display.blit(self.bottom_box.image, self.bottom_rect)

        # Image display
        if self.img is not None:
            surface = pygame.surfarray.make_surface(self.img)
            surface = pygame.transform.rotate(surface, -90)
            self.display.blit(surface, (0,0))

        # update the display
        pygame.display.flip()

    def imgCallback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.img = cv2.resize(self.img, (400, 400))

    def compassCallback(self, msg):
        self.compass = msg.data

    def depthCallback(self, msg):
        self.depth = msg.vector.z

    def gpsCallback(self, msg):
        self.gps_msg = msg

    def batteryCallback(self, msg):
        self.battery_state = msg

if __name__  == "__main__":
    rospy.init_node('d3_display', anonymous=True)

    disp = D3Display()

    rospy.spin()



