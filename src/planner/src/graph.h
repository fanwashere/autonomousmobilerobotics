#pragma once

#include "map.h"
#include "math.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Node
{
    using Neighbor = std::pair<std::shared_ptr<Node>, double>;

public:
    Coordinate getCoordinate();
    void assignId(int id);
    void addNeighbor(Node neighbor, double distance);
    std::vector<Neighbor> getNeighbors();
    Node(Coordinate coord);

private:
    std::vector<Neighbor> neighbors;
    Coordinate coord;
    int id;
};

class Graph
{
public:
    void addNode(Node node);
    Graph(Grid grid);

private:
    void addVertex(Node n1, Node n2, double weight);
    std::vector<Node> nodes;
    Grid grid;
    int iota = 0;
};
