#pragma once

#include "map.h"

struct Edge;

class Node
{
public:
    Node(const Coordinate &coord);

    void addEdge(std::shared_ptr<Node> to, double weight);
    std::vector<std::shared_ptr<Edge>> getEdges() const;
    Coordinate getCoordinate() const;
    void assignId(int id);
    int getId() const;

private:
    std::vector<std::shared_ptr<Edge>> edges;
    Coordinate coord;
    int id;
};

struct Edge
{
    std::shared_ptr<Node> destination;
    double weight;

    Edge(std::shared_ptr<Node> destination, double weight);
};

class Graph
{
public:
    Graph(const Grid &grid);

    void addNode(Node &node);
    std::vector<Coordinate> findShortestPath(const Coordinate &start, const Coordinate &end);
    std::vector<Coordinate> findShortestPath(const Coordinate &start, const std::vector<Coordinate> &waypoints);

private:
    std::vector<std::shared_ptr<Node>> nodes;
    Grid grid;
    int iota = 0;

    void addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight);
    std::vector<Coordinate> nodesToCoordinates(std::vector<std::shared_ptr<Node>> nodeVector) const;
};
