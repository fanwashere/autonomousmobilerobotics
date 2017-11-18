#pragma once

#include "map.h"

class Node
{
    using Neighbor = std::pair<std::shared_ptr<Node>, double>;

public:
    Node(const Coordinate &coord);

    void addNeighbor(Node neighbor, double distance);
    std::vector<Neighbor> getNeighbors();
    Coordinate getCoordinate();
    void assignId(int id);

private:
    std::vector<Neighbor> neighbors;
    Coordinate coord;
    int id;
};

class Graph
{
public:
    Graph(const Grid &grid);

    void addNode(const Node &node);

private:
    std::vector<std::shared_ptr<Node>> nodes;
    Grid grid;
    int iota = 0;

    void addVertex(Node n1, Node n2, double weight);
};
