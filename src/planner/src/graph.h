#pragma once

#include "map.h"

struct Edge;

class Node
{
public:
    Node(const Coordinate &coord);

    void addEdge(std::shared_ptr<Edge> edge);
    std::vector<std::shared_ptr<Edge>> getEdges() const;
    Coordinate getCoordinate() const;
    void assignId(int id);

private:
    std::vector<std::shared_ptr<Edge>> edges;
    Coordinate coord;
    int id;
};

struct Edge
{
    std::shared_ptr<Node> n1;
    std::shared_ptr<Node> n2;
    double weight;

    Edge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight);
};

class Graph
{
public:
    Graph(const Grid &grid);

    void addNode(Node &node);

private:
    std::vector<std::shared_ptr<Node>> nodes;
    std::vector<std::shared_ptr<Edge>> edges;
    Grid grid;
    int iota = 0;

    void addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight);
};
