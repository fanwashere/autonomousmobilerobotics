#include "state_estimation.h"
#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <math.h>

namespace
{
    const double RATE = 1.0;
    const uint32_t NUMPARTICLES = 300;

    constexpr double squared(double x)
    {
        return x*x;
    }
    
    double multivariateGaussianCalculation(Vector3d X, Vector3d U, Matrix3d E) 
    { 
        double determinant = pow((2.0*M_PI*E).determinant(), -0.5);
        Matrix3d E_inverse = E.inverse();
        RowVector3d transpose = (X-U).transpose();

        double exp_eval = exp(-0.5*transpose*E_inverse*(X-U));

        return determinant*exp_eval;    
    }

    std::random_device rngDevice;  // Will be used to obtain a seed for the random number engine
    std::mt19937 rngGenerator(rngDevice()); //Standard mersenne_twister_engine seeded with random_device()
    std::uniform_real_distribution<> realDist(-10.0, 10.0);
    std::normal_distribution<> noise(-0.1, 0.1);

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

    pose.x = msg->pose[i].position.x + noise(rngGenerator);
    pose.y = msg->pose[i].position.y + noise(rngGenerator);
    pose.yaw = tf::getYaw(msg->pose[i].orientation) + noise(rngGenerator);
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

    // Get Variance with respect to X and Y
    (odometry.pose.covariance)(0,0) = (msg->pose.covariance)[0];
    (odometry.pose.covariance)(0,1) = (msg->pose.covariance)[1];
    (odometry.pose.covariance)(0,2) = (msg->pose.covariance)[5];
    (odometry.pose.covariance)(1,0) = (msg->pose.covariance)[6];
    (odometry.pose.covariance)(1,1) = (msg->pose.covariance)[7];
    (odometry.pose.covariance)(1,2) = (msg->pose.covariance)[11];
    (odometry.pose.covariance)(2,0) = (msg->pose.covariance)[30];
    (odometry.pose.covariance)(2,1) = (msg->pose.covariance)[31];
    (odometry.pose.covariance)(2,2) = (msg->pose.covariance)[35];
}

ParticleFilter::ParticleFilter(uint32_t numParticles)
    : numParticles(numParticles)
    , weights(numParticles)
    , particles(numParticles)
    , predictions(numParticles)
    , prevTime(ros::Time::now())
{
    for (Pose& pose : particles)
    {
        pose.x = realDist(rngGenerator);
        pose.y = realDist(rngGenerator);
        pose.yaw = realDist(rngGenerator);
    }
}

void ParticleFilter::run(const Pose& ips, const Odometry& wheel)
{
    ros::Time currentTime = ros::Time::now();

    double dt = (currentTime - prevTime).toSec();

    // Define setup
    double cumsum_weight = 0.0;
    double seed = 0.0;

    // Prediction update
    for (int i = 0; i < numParticles; i++)
    {
        // Use the motion model to calculate predictions
        predictions[i].x = particles[i].x + cos(wheel.pose.yaw) * wheel.twist.twist.linear.x * dt + noise(rngGenerator);
        predictions[i].y = particles[i].y + sin(wheel.pose.yaw) * wheel.twist.twist.linear.x * dt + noise(rngGenerator);
        predictions[i].yaw = particles[i].yaw + wheel.twist.twist.angular.z * dt + noise(rngGenerator);

        // Update weights
        Vector3d measurement; Vector3d particle;
        particle(0) = predictions[i].x; particle(1) = predictions[i].y; particle(2) = predictions[i].yaw;
        measurement(0) = ips.x; measurement(1) = ips.y; measurement(2) = ips.yaw;
        
        weights[i] = multivariateGaussianCalculation(particle, measurement, wheel.pose.covariance);
        cumsum_weight += weights[i];
        weights[i] = cumsum_weight;
    }

    // Measurement update

    for (int i = 0 ; i < numParticles; i++)
    {
        seed = cumsum_weight*((double) rand() / (RAND_MAX));
        
        for (int j = 0; j < numParticles; j++)
        {

            if(weights[j] > seed)
            {
                particles[i].x = predictions[j].x;
                particles[i].y = predictions[j].y;
                particles[i].yaw = predictions[j].yaw;

                break;
            }
        }
    }

    prevTime = currentTime;
}

void ParticleFilter::publish(const ros::Publisher& publisher)
{
    for (int i = 0 ; i < numParticles; i++) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.color.g = 1.0f;
        marker.color.a = 1.0f;

        geometry_msgs::Point point;
        point.x = particles[i].x;
        point.y = particles[i].y;

        marker.points.push_back(point);
        publisher.publish(marker);
    }
}

void publishPath(const ros::Publisher& publisher, const Pose& pose)
{
    geometry_msgs::PoseStamped stampedPose;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.id = NUMPARTICLES + 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.r = 1.0f;
    marker.color.a = 1.0f;

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
    ros::Publisher pathPublisher = n.advertise<visualization_msgs::Marker>("/path", 1);

    ParticleFilter particleFilter(NUMPARTICLES);

    while(ros::ok())
    {
        const auto odom = odomHandler.getOdometry();
        const auto pose = poseHandler.getPose();

        publishPath(pathPublisher, pose);

        particleFilter.run(pose, odom);
        particleFilter.publish(particlePublisher);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
