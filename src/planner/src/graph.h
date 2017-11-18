#pragma once

#include <map.h>
#include <math.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Node
{
public:
    Coordinate getCoordinate();
    void assignId(int id);
    void addNeighbor(Node neighbor, double distance);
    std::vector<Node> getNeighbors();
    Node(Coordinate setCoord) {
        coord = setCoord;
    };

  private:
    std::vector<std::pair<Node, double>> neighbors;
    Coordinate coord;
    int id;
};

class Graph
{
public:
    void addNode(Node node);
    Graph(Grid grid);

private:
    void addVertex(Node n1, Node n2);
    std::vector<Node> nodes;
    Grid grid;
    int iota = 0;
}