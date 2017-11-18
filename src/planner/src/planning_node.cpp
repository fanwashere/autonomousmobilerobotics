#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include "nav_msgs/OccupancyGrid.h"

namespace
{
    const std::string NODE_NAME = "planner";
    const double RATE = 1.0;
    const int NUM_NODES = 100;
    const double MAX_NODE_DISTANCE = 10.0;

    using MapCallback = boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)>
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    PoseHandler poseHandler;
    MapHandler mapHandler;

#ifdef LIVE
    PoseLiveCallback poseLiveCallback = boost::bind(&PoseHandler::callbackLive, &poseHandler, _1);
    auto poseLiveSubscriber = n.subscribe("/indoor_pos", 1, poseLiveCallback);
    ROS_INFO("Subscribed to /indoor_pos topic");
#else
    PoseSimCallback poseSimCallback = boost::bind(&PoseHandler::callbackSim, &poseHandler, _1);
    auto poseSimSubscriber = n.subscribe("/gazebo/model_states", 1, poseSimCallback);
    ROS_INFO("Subscribed to /gazebo/model_states topic");
#endif

    MapCallback mapCallback = boost::bind(&MapHandler::callback, &MapHandler, _1);
    auto mapSubscriber = n.subscribe("/map", 1, mapCallback);
    ROS_INFO("Subscribed to /map topic");

    /*
    ros::ServiceServer service = n.advertiseService("plan", plan);
    ROS_INFO("Planning server online");
    ros::spin();
    */

    Grid grid = MapHandler.getGrid();
    Graph graph(grid);

    int i;
    for (i = 0; i < NUM_NODES; i++) {
        Coordinate coord = grid.getRandomCoordinate();
        Node node(coord);

        int j;
        for (j = 0; j < nodes.size(); j++) {
            Coordinate targetCoord = nodes[j].getCoordinate();
            double distance = coord.distanceTo(targetCoord);

            if (distance > MAX_NODE_DISTANCE || grid.checkCollision(coord, targetCoord))
            {
                continue;
            }

            node.addNeighbor(nodes[j], distance);
            nodes[j].addNeighbor(node, distance);
        }

        graph.addNode(node);
    }

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
