#include <ros/ros.h>
#include "pose.h"

namespace
{
    const std::string NODE_NAME = "planner";
    const double RATE = 40.0;

    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::Rate loop_rate(RATE);

    while (ros::ok())
    {
    	loop_rate.sleep();
    	ros::spinOnce();
    }

    return 0;
}
