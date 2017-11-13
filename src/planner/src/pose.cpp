#include "pose.h"

#include <tf/transform_datatypes.h>

Pose PoseHandler::getPose() const
{
    return pose;
}

void PoseHandlerLive::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf::getYaw(msg->pose.pose.orientation);
}

void PoseHandlerSim::callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++) if(msg->name[i] == "mobile_base") break;

    pose.x = msg->pose[i].position.x;
    pose.y = msg->pose[i].position.y;
    pose.yaw = tf::getYaw(msg->pose[i].orientation);
}