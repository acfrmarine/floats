#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64
import navio2.pwm
import numpy as np

class Thruster:
    """
    Runs a blue robotics thruster
    """
    def __init__(self):
        """
        Gets params
        Initialises PWM
        Primes ESC
        Sets up timers + subscribers
        """
        self.primed = False
        self.go = False
        self.pwm_max_ = rospy.get_param("~val_max",1.0)
        self.pwm_min_ = rospy.get_param("~val_min",-1.0)
        self.time_max_ = rospy.get_param("~time_max",2.00)# in ms
        self.time_min_ = rospy.get_param("~time_min",1.00)
        self.prime_required_ = rospy.get_param("~prime",True)

        self.max_change = rospy.get_param("~max_change", 0.04)

        self.timeout_ = rospy.get_param("~timeout", 2.0)
        control_cmd_topic0 = rospy.get_param("~control_cmd_topic0", "thruster0_cmd")  # Sometimes easier doing this than remapping
        control_cmd_topic1 = rospy.get_param("~control_cmd_topic1", "thruster1_cmd")  # Sometimes easier doing this than remapping

        pwm_id0_ = rospy.get_param("~pwm_id0", 0)
        pwm_id1_ = rospy.get_param("~pwm_id1", 0)
        rospy.loginfo("Setting PWM ID %d" %pwm_id0_)
        rospy.loginfo("Setting PWM ID %d" %pwm_id1_)
        self.pwm0_ = navio2.pwm.PWM(pwm_id0_)
        self.pwm1_ = navio2.pwm.PWM(pwm_id1_)

        self.direction0_ = np.sign(rospy.get_param("~thruster_direction0", 1))
        self.direction1_ = np.sign(rospy.get_param("~thruster_direction1", 1))

        rospy.loginfo("Thruster 0 direction set to %d" %self.direction0_)
        rospy.loginfo("Thruster 1 direction set to %d" %self.direction1_)

        self.pwm0_.initialize()
        self.pwm0_.set_period(50);
        self.pwm0_.enable()

        self.pwm1_.initialize()
        self.pwm1_.set_period(50);
        self.pwm1_.enable()

        if self.prime_required_:
            self.prime_esc0()
            self.prime_esc1()

        self.cmd0_ = 0.0
        self.cmd1_ = 0.0
        self.prev_cmd0_ = 0.0
        self.prev_cmd1_ = 0.0
        self.last_received_cmd_ = rospy.Time.now()

        # Create safety timer - if new cmd isn't received every timeout seconds, turn thruster off
        rospy.Timer(rospy.Duration(self.timeout_), self.timerCallback)
        rospy.Subscriber(control_cmd_topic0, Float64, self.cmd0Callback)
        rospy.Subscriber(control_cmd_topic1, Float64, self.cmd1Callback)

    def cmd0Callback(self, msg):
        """
        Receives the commanded thruster value, scales the value to ms, then sets the duty cycle
        Args:
            msg: (std_msgs.Float32) the commanded thruster value.
        """
        self.go = True
        self.cmd0_ = msg.data
        self.last_received_cmd_ = rospy.Time.now()
        if self.go and self.primed: 
            self.cmd0_, self.prev_cmd0_ = self.limit_change(self.cmd0_, self.prev_cmd0_)
            self.setPWM0(self.scale_input(float(self.direction0_) * self.cmd0_))
        rospy.loginfo("Setting command %f" % float(self.direction0_ * self.cmd0_))

    def cmd1Callback(self, msg):
        """
        Receives the commanded thruster value, scales the value to ms, then sets the duty cycle
        Args:
            msg: (std_msgs.Float32) the commanded thruster value.
        """
        self.go = True
        self.cmd1_ = msg.data
        self.last_received_cmd_ = rospy.Time.now()
        if self.go and self.primed:
            self.cmd1_, self.prev_cmd1_ = self.limit_change(self.cmd1_, self.prev_cmd1_)
            self.setPWM1(self.scale_input(float(self.direction1_) * self.cmd1_))
        rospy.loginfo("Setting command %f" % float(self.direction1_ * self.cmd1_))

    
    def limit_change(self, cmd, prev_cmd):
        if cmd - prev_cmd > self.max_change:
            cmd = prev_cmd + self.max_change
        elif cmd - prev_cmd < -self.max_change:
            cmd = prev_cmd - self.max_change
            #rospy.loginfo("cmd: %f, prev_cmd: %f" %(cmd, prev_cmd))
        prev_cmd = cmd
        return cmd, prev_cmd
    

    def timerCallback(self, event):
        """
        A regular timer to turn off thruster if set.

        Args:
            event: (rospy.TimerEvent) Information about timing. Not used.
        """
        if rospy.Time.now() - self.last_received_cmd_ > rospy.Duration(self.timeout_):
            self.setPWM0(self.scale_input(0.0)) # Stop motor
            self.setPWM1(self.scale_input(0.0)) # Stop motor
            rospy.logwarn("Timeout hit - turning thruster off")
        self.go = False

    def scale_input(self, value):
        """
        Scales the input to ms so the duty cycle can be set
        Args:
            value: the value to be scaled

        Returns:
            float: The scaled value

        """
        scaled = value
        # clip the range
        if (scaled>self.pwm_max_):
            scaled=self.pwm_max_
        if (scaled<self.pwm_min_):
            scaled=self.pwm_min_
        # get the corresponding value
        scaled=(1.0*scaled-1.0*self.pwm_min_)/(1.0*self.pwm_max_-1.0*self.pwm_min_)*(self.time_max_-self.time_min_)+self.time_min_
        return scaled

    def prime_esc0(self):
        """
        Primes the ESC
        """
        # perform a priming sequence, by setting to zero, 0.5, 0 for some time.
        rospy.loginfo("Performing priming sequence")
        rate=rospy.Rate(50)
        #for i in range(100):
        self.setPWM0((self.time_max_+self.time_min_)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM0((self.time_max_*1.5+self.time_min_*0.5)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM0((self.time_max_+self.time_min_)/2.0)
        rate.sleep()
        #priming done!
        rospy.loginfo("Priming Done!")
        self.primed = True

    def prime_esc1(self):
        """
        Primes the ESC
        """
        # perform a priming sequence, by setting to zero, 0.5, 0 for some time.
        rospy.loginfo("Performing priming sequence")
        rate=rospy.Rate(50)
        #for i in range(100):
        self.setPWM1((self.time_max_+self.time_min_)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM1((self.time_max_*1.5+self.time_min_*0.5)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM1((self.time_max_+self.time_min_)/2.0)
        rate.sleep()
        #priming done!
        rospy.loginfo("Priming Done!")
        self.primed = True

    def setPWM0(self, value):
        """
        Sets the PWM
        Args:
            value: (float) The duty cycle in miliseconds
        """
        self.pwm0_.set_duty_cycle(value)

    def setPWM1(self, value):
        """
        Sets the PWM
        Args:
            value: (float) The duty cycle in miliseconds
        """
        self.pwm1_.set_duty_cycle(value)

    def shutdown(self):
        """
        Deinitializes the PWM during shutdown
        """
        rospy.loginfo("Deitinialising PWM")
        self.pwm0_.deinitialize()
        self.pwm1_.deinitialize()


if __name__ == "__main__":
    rospy.init_node('thruster_node', anonymous=True)
    thruster = Thruster()
    def shutdown_hook():
        thruster.shutdown()
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()

