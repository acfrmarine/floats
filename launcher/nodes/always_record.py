#!/usr/bin/env python2

import numpy as np
import rospy
from std_msgs.msg import Bool,String
import roslaunch
import os
import glob

class RecordSwitch:
    """RecordSwitch:
    Listens to the "record_switch" topic and when activates starts/stops recording.
    """
    def __init__(self):
        self.dive_number = rospy.get_param("~initial_dive", 0)
        self.data_dir = rospy.get_param("~data_dir", "/media/data/")
        self.trip = rospy.get_param("~trip", "202001_Local")
        self.recording = False
        self.switch_activated = False
	self.activation_count = 0
        self.launch = None
        self.change_mode_flag = False
	self.log_pub = rospy.Publisher('logs', String, queue_size=1)
	self.rec_pub = rospy.Publisher('recording', Bool, queue_size=1)
        #rospy.Subscriber('switch', Bool, self.switchCallback)
	self.start_recording()

    def start_recording(self):
        """Starts recording. Creates a roslaunch xml string and launches it.
        """
	self.get_next_dive()
        # if True:
        try:
            # Generates UUID and starts logging
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            # Roslaunch process initialise
            self.launch = roslaunch.scriptapi.ROSLaunch()
            launch_begin = '''<launch>\n'''

            launch_end = '''\n</launch>'''

            record_part1 = '''<node name="recorder" pkg="rosbag" type="record" args="--split --duration=60 -o '''
            save_path = os.path.join(self.data_dir, self.trip, "dive_"+str(self.dive_number),  "logs.bag")
            if not os.path.exists(os.path.dirname(save_path)):
                os.makedirs(os.path.dirname(save_path))
            topic_list = ["/stereo/right/image_raw",
                          "/stereo/right/camera_info",
                          "/stereo/left/image_raw",
                          "/stereo/left/camera_info",
			  "/fisheye/image_raw",
			  "/fisheye/camera_info",
                          "/science/image_rect_color",
                          "/science/camera_info",
                          "/trigger",
                          "/navio/gps",
                          "/navio/magnetic_field",
                          "/imu/data",
                          "/navio/imu",
                          "/depth",
                          "/temperature",
                          "/navio/ups",
                          "/leak"]
            topic_str = " ".join(topic_list)
            # Create the launch string
            record_str = record_part1 + save_path + " " + topic_str + "\"" + " />"
            launch_str = launch_begin + record_str + launch_end
            # Set the launch string
            self.launch.parent.roslaunch_strs = {launch_str}
            # Start the process
            self.launch.start()

            rospy.loginfo("Recording to %s" % save_path)
            #log_msg = String()
	    #log_msg.data = "Starting logs on dive %d" % self.dive_number
	    #self.log_pub.publish(log_msg)
	    
            # Set the recording state
            self.recording = True

        except BaseException as e:
            rospy.logerr("Failed to start log - %s" % e)

    def stop_recording(self):
        """ Stops the recording, that was started with the start_recording function.
        """
        self.recording = False
        rospy.loginfo("Stopping logs")
        try:
            self.launch.stop()
            #log_msg = String()
	    #log_msg.data = "Stopping Logs on dive %d" % self.dive_number
	    #self.log_pub.publish(log_msg)
            self.launch = None
        except BaseException as e:
            rospy.logerr("Failed to stop log, nothing to stop - %s" % e)
            rospy.loginfo("Stopping. Nothing to stop.")

    def switchCallback(self, msg):
        """
        Checks if switch has been activated and if so starts/stops recording
        """
        # switch activated if msg.data == False. Also checks if switch has already been activated
        if not msg.data and not self.switch_activated:
	    self.activation_count += 1
	    if self.activation_count > 5:
            	self.change_mode_flag = True
            	self.switch_activated = True  # Set the switch to activated state
            # TODO test if timer is needed
        else:
            self.switch_activated = False  # Set the switch to not activated state, so it can be activated again.
	    self.activation_count = 0

    def change_mode(self):
        """
        Changes the mode from recording to not recording and vice versa. Needs to be called from the main thread (i.e. not from a callback).
        """
        if self.change_mode_flag:
            if self.recording:
                self.stop_recording()
            else:
                self.start_recording()
        self.change_mode_flag = False

    def publish_state(self):
	bmsg = Bool()
	bmsg.data = self.recording
	self.rec_pub.publish(bmsg)

    def get_next_dive(self):
	all_files = glob.glob(os.path.join(self.data_dir, self.trip, '*'))
	all_dive_dirs = []
	for f in all_files:
	    if os.path.isdir(f) and 'dive_' in os.path.basename(f):
		all_dive_dirs.append(f)
	if len(all_dive_dirs) == 0:
	    next_dive = 0
	else:
	    dive_nums = list(set([int(d.split('_')[-1]) for d in all_dive_dirs]))
	    next_dive = dive_nums[-1] + 1
	if not os.path.isdir(os.path.join(self.data_dir, self.trip, 'dive_' + str(next_dive))):
	    os.makedirs(os.path.join(self.data_dir, self.trip, 'dive_' + str(next_dive)))
	self.dive_number  = next_dive


if __name__ == '__main__':
    rospy.init_node('record_switch', anonymous=True)
    r = rospy.Rate(10)
    # Initialise the class
    rs = RecordSwitch()
    # Need to run a while loop here to ensure rs.change_mode() is called from the main thread
    while not rospy.is_shutdown():
        #rs.change_mode()
	rs.publish_state()
        r.sleep()
