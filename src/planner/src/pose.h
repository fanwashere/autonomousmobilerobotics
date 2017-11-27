#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Feel free to redefine what is needed in the Pose container
struct Pose {
    double x;
    double y;
    double yaw;
};

class PoseHandler {
public:
    virtual ~PoseHandler() = default;
    Pose getPose() const;

protected:
    PoseHandler() = default;
    Pose pose;
};

class PoseHandlerLive : public PoseHandler {
public:
    PoseHandlerLive() = default;
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};

class PoseHandlerSim : public PoseHandler {
public:
    PoseHandlerSim() = default;
    void callback(const gazebo_msgs::ModelStates::ConstPtr &msg);
};