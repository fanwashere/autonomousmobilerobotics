#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
    	loop_rate.sleep();
    	ros::spinOnce();
    }

    return 0;
}
