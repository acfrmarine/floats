#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import navio2.pwm


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
        # Get params
        self.pwm_output_ = rospy.get_param("~srv_num",0)
        self.pwm_max_ = rospy.get_param("~val_max",1.0)
        self.pwm_min_ = rospy.get_param("~val_min",-1.0)
        self.time_max_ = rospy.get_param("~time_max",2.00)# in ms
        self.time_min_ = rospy.get_param("~time_min",1.00)
        self.prime_required_ = rospy.get_param("~prime",False)

        self.timeout_ = rospy.get_param("~timeout", 2.0)

        pwm_id_ = rospy.get_param("~pwm_id", 0)

        self.pwm_ = navio2.pwm.PWM(pwm_id_)

        self.pwm_.initialize()
        self.pwm_.set_period(50);
        self.pwm_.enable()
        self.prime_esc()

        self.cmd_ = 0.0
        self.last_received_cmd_ = rospy.Time.now()

        # Create safety timer - if new cmd isn't received every timeout seconds, turn thruster off
        rospy.Timer(rospy.Duration(self.timeout_), self.timerCallback)
        rospy.Subscriber("thruster_cmd", Float32, self.cmdCallback)

    def cmdCallback(self, msg):
        """
        Receives the commanded thruster value, scales the value to ms, then sets the duty cycle
        Args:
            msg: (std_msgs.Float32) the commanded thruster value.
        """
        self.go = True
        self.cmd_ = msg.data
        self.last_received_cmd_ = rospy.Time.now()
        if self.go and self.primed:
            self.setPWM(self.scale_input(self.cmd_))


    def timerCallback(self, event):
        """
        A regular timer to turn off thruster if set.

        Args:
            event: (rospy.TimerEvent) Information about timing. Not used.
        """
        if rospy.Time.now() - self.last_received_cmd_ > rospy.Duration(self.timeout_):
            self.setPWM(self.scale_input(0.0)) # Stop motor
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

    def prime_esc(self):
        """
        Primes the ESC
        """
        # perform a priming sequence, by setting to zero, 0.5, 0 for some time.
        rospy.loginfo("Performing priming sequence")
        rate=rospy.Rate(50)
        #for i in range(100):
        self.setPWM((self.time_max_+self.time_min_)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM((self.time_max_*1.5+self.time_min_*0.5)/2.0)
        rate.sleep()

        #for i in range(100):
        self.setPWM((self.time_max_+self.time_min_)/2.0)
        rate.sleep()
        #priming done!
        rospy.loginfo("Priming Done!")
        self.primed = True

    def setPWM(self, value):
        """
        Sets the PWM
        Args:
            value: (float) The duty cycle in miliseconds
        """
        self.pwm_.set_duty_cycle(value)

    def shutdown(self):
        """
        Deinitializes the PWM during shutdown
        """
        rospy.loginfo("Deitinialising PWM")
        self.pwm_.deinitialize()


if __name__ == "__main__":
    rospy.init_node('thruster_node', anonymous=True)
    thruster = Thruster()
    def shutdown_hook():
        thruster.shutdown()
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()

