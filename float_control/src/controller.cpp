
/*
 * Controller
 * Purpose: Implements a thruster, depth and altitude controller.
 * Subscribes:
 *  -
 * Publishes:
 * -
 * Services:
 * -
 * Outputs: Motor Commands
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


enum mode_select{THRUSTER=0, DEPTH=1, ALTITUDE=2}

class Controller
{
    public:
        Controller(ros::NodeHandle *nh)
        {
            mode_ = THRUSTER;
            ros::Duration timeout_(0.0);

            thruster_set_ = 0.0
            depth_set_ = 0.0
            altitude_set_ = 0.0

            depth_ = 0.0
            altitude_ = 0.0

            new_depth_ = False;
            new_altitude_ = False;
            ros::Time last_altitude_time_()




            stbdPub_ = n_.advertise<std_msgs::Float32>("/right_thrust_cmd", 1);

        }
    private:
        // The nodehandle
        ros::NodeHandle nh_;

        // Mode selection
        enum mode_select mode_;

        // Timeout
        ros::Duration timeout_; // Maximum time that can elapse before switching thrusters off
        ros::Time time0_; // Time of the last command

        // Variables to hold the set commands
        double thruster_set_;
        double depth_set_;
        double altitude_set_;

        // Variables to hold values of altitude and depth
        double depth_;
        double altitude_;

        // Flags to indicate when we have information
        bool depth_received_;
        bool altitude_received_;

        // Flags to determine when we have new information
        bool new_depth_;
        bool new_altitude_;

        // Variables holding the time information was last received
        ros::Time last_depth_time_;
        ros::Time last_altitude_time_;

        // Counter for warning the user when sensors stop receiving
        int no_sensor_count_;

        // Services
        ros::ServiceServer thruster_cmd_service_;
        ros::ServiceServer depth_cmd_service_;
        ros::ServiceServer altitude_cmd_service_;

        // Publishers
        ros::Publisher tgtDepthPub_;
        ros::Publisher thrusterPub_;
        ros::Publisher pidEnablePub_;

        // Subscribers
        ros::Subscriber altitudeSub_;
        ros::Subscriber depthSub_;


};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "controller");

  Controller controller(&nh);

  ros::spin();

  return 0;
}