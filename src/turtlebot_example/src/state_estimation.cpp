#include "state_estimation.h"

#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

namespace
{
    const double RATE = 1.0;

    constexpr double squared(double x)
    {
        return x*x;
    }

    constexpr double normpdf(double x, double u, double s) 
    { 
        return (1.0/(s*sqrt(2.0*M_PI)))*exp(-0.5*squared(x-u)/squared(s));
    }

    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
    using OdometryCallback = boost::function<void(const nav_msgs::Odometry::ConstPtr&)>;
}

Pose PoseHandler::getPose() const
{
    return pose;
}

void PoseHandler::callbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++) if(msg->name[i] == "mobile_base") break;

    pose.x = msg->pose[i].position.x ;
    pose.y = msg->pose[i].position.y ;
    pose.yaw = tf::getYaw(msg->pose[i].orientation);
}

void PoseHandler::callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf::getYaw(msg->pose.pose.orientation);
}

Odometry OdometryHandler::getOdometry() const
{
    return odometry;
}

void OdometryHandler::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Pose contains all the position information
    odometry.pose.x = msg->pose.pose.position.x;
    odometry.pose.y = msg->pose.pose.position.y;
    odometry.pose.yaw = tf::getYaw(msg->pose.pose.orientation);

    // Twist contains all the velocity information
    odometry.twist = msg->twist;
}

ParticleFilter::ParticleFilter(uint32_t numParticles)
    : numParticles(numParticles)
    , weights(numParticles)
    , particles(numParticles)
    , predictions(numParticles)
    , prevTime(ros::Time::now())
{}

void ParticleFilter::run(const Pose& ips, const Odometry& wheel)
{
    ros::Time currentTime = ros::Time::now();

    double dt = (currentTime - prevTime).toSec();

    // Define setup
    const double mean = 0.0;
    const double Q = 0.1;

    double cumsum_weight = 0.0;
    double seed = 0.0;

    // Prediction update
    for (int i = 0; i < numParticles; i++)
    {
        // Use the motion model to calculate predictions
        predictions[i].x = particles[i].x + wheel.twist.twist.linear.x * dt; // Check for trig
        predictions[i].y = particles[i].x + wheel.twist.twist.linear.y * dt;
        predictions[i].yaw = particles[i].yaw + wheel.twist.twist.angular.z * dt;

        // Update weights
        weights[i] = normpdf(ips.x, predictions[i].x, Q) + normpdf(ips.y, predictions[i].y, Q) + normpdf(ips.yaw, predictions[i].y, Q);
        cumsum_weight += weights[i];
        weights[i] = cumsum_weight;
    }

    // Measurement update

    for (int i = 0 ; i < numParticles; i++) {
        seed = cumsum_weight*((double) rand() / (RAND_MAX));
        
        for (int j = 0; j < numParticles; j++) {
            if(weights[j] > seed) {
                particles[i].x = predictions[j].x;
                particles[i].y = predictions[j].y;
                particles[i].yaw = predictions[j].yaw;
                
                break;
            }
        }
    }

    prevTime = currentTime;
}

void publishParticles(const ros::Publisher& publisher, const Pose& pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/particle_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "particle_filter";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.g = 1.0f;

    geometry_msgs::Point point;
    point.x = pose.x;
    point.y = pose.y;

    marker.points.push_back(point);

    publisher.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimation");
    ros::NodeHandle n;
    ros::Rate rate(RATE);

    PoseHandler poseHandler;
#ifdef LIVE
    PoseLiveCallback poseLiveCallback = boost::bind(&PoseHandler::callbackLive, &poseHandler, _1);
    auto poseLiveSubscriber = n.subscribe("/indoor_pos", 1, poseLiveCallback);
    ROS_INFO("Subscribed to /indoor_pos topic");
#else
    PoseSimCallback poseSimCallback = boost::bind(&PoseHandler::callbackSim, &poseHandler, _1);
    auto poseSimSubscriber = n.subscribe("/gazebo/model_states", 1, poseSimCallback);
    ROS_INFO("Subscribed to /gazebo/model_states topic");
#endif

    OdometryHandler odomHandler;
    OdometryCallback odomCallback = boost::bind(&OdometryHandler::callback, &odomHandler, _1);
    auto odomSubscriber = n.subscribe("/odom", 1, odomCallback);
    ROS_INFO("Subscribed to /odom topic");

    ros::Publisher particlePublisher = n.advertise<visualization_msgs::Marker>("/particle_filter", 1);

    while(ros::ok())
    {
        const auto odom = odomHandler.getOdometry();
        const auto pose = poseHandler.getPose();

        publishParticles(particlePublisher, pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}