#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include <nav_msgs/OccupancyGrid.h>

namespace
{
    const std::string NODE_NAME = "planner";
    const double RATE = 1.0;
    const int NUM_NODES = 100;
    const double MAX_NODE_DISTANCE = 10.0;

    using MapCallback = boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    MapHandler mapHandler;

#ifdef LIVE
    PoseHandlerLive poseHandler;
    PoseLiveCallback poseLiveCallback = boost::bind(&PoseHandlerLive::callback, &poseHandler, _1);
    auto poseLiveSubscriber = n.subscribe("/indoor_pos", 1, poseLiveCallback);
    ROS_INFO("Subscribed to /indoor_pos topic");
#else
    PoseHandlerSim poseHandler;
    PoseSimCallback poseSimCallback = boost::bind(&PoseHandlerSim::callback, &poseHandler, _1);
    auto poseSimSubscriber = n.subscribe("/gazebo/model_states", 1, poseSimCallback);
    ROS_INFO("Subscribed to /gazebo/model_states topic");
#endif

    MapCallback mapCallback = boost::bind(&MapHandler::callback, &mapHandler, _1);
    auto mapSubscriber = n.subscribe("/map", 1, mapCallback);
    ROS_INFO("Subscribed to /map topic");

    /*
    ros::ServiceServer service = n.advertiseService("plan", plan);
    ROS_INFO("Planning server online");
    ros::spin();
    */

    Grid grid = mapHandler.getGrid();
    Graph graph(grid);

    int i;
    for (i = 0; i < NUM_NODES; i++) {
        Coordinate coord = grid.getRandomCoordinate();
        Node node(coord);
        graph.addNode(node);
    }


    Coordinate start(0, 0);
    Coordinate end(10, 10);
    std::vector<Coordinate> path = graph.findShortestPath(start, end);

    /*
    Pose pose = poseHandler.getPose();
    Coordinate initCoord(pose.x, pose.y);
    Node initNode(initCoord);
    */

    ros::Rate rate(RATE);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
