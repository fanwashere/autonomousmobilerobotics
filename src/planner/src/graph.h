#pragma once

class Edge;
class Coordinate;
class Grid;
class MapHandler;

class Node {
public:
    explicit Node(const Coordinate &c);

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

class Edge {
public:
    Edge(std::shared_ptr<Node> destination, double weight);

    std::shared_ptr<Node> getDestination() const;
    double getWeight() const;

private:
    std::shared_ptr<Node> destination;
    double weight;
};

class Graph
{
public:
    Graph(std::shared_ptr<Grid> g);

    void addNode(std::shared_ptr<Node> node);
    std::pair<std::vector<Coordinate>, double> findShortestPath(const Coordinate &start, const Coordinate &end);
    std::vector<Coordinate> findShortestPath(const Coordinate &start, std::vector<Coordinate> waypoints);

private:
    std::vector<std::shared_ptr<Node>> nodes;
    std::shared_ptr<Grid> grid;
    int iota;

    void addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight);
    std::vector<Coordinate> nodesToCoordinates(std::vector<std::shared_ptr<Node>> nodeVector) const;
};
