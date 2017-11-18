#include "graph.h"
#include "map.h"

namespace
{
    #define NEIGHBOR_MAX_DISTANCE 30.0
    #define Neighbor std::pair<std::shared_ptr<Node>, double>
}

Node::Node(const Coordinate &c) {
    coord = c;
}

void Node::addEdge(std::shared_ptr<Edge> edge)
{
    edges.push_back(edge);
}

std::vector<std::shared_ptr<Edge>> Node::getEdges() const
{
    return edges;
}

Coordinate Node::getCoordinate() const
{
    return coord;
}

void Node::assignId(int setId)
{
    id = setId;
}

Edge::Edge(std::shared_ptr<Node> setN1, std::shared_ptr<Node> setN2, double setWeight)
{
    n1 = setN1;
    n2 = setN2;
    weight = setWeight;
}

Graph::Graph(const Grid &g)
{
    grid = g;
}

void Graph::addNode(Node &node)
{
    Coordinate coord = node.getCoordinate();
    node.assignId(iota++);
    std::shared_ptr<Node> nodePtr = std::make_shared<Node>(node);

    int i;
    for (i = 0; i < nodes.size(); i++) {
        const Coordinate targetCoord = nodes[i]->getCoordinate();
	const double distance = coord.distanceTo(targetCoord, grid.getResolution());
	if (distance < NEIGHBOR_MAX_DISTANCE && grid.checkCollision(coord, targetCoord))
        {
            addVertex(nodePtr, nodes[i], distance);
        }
    }

    nodes.push_back(nodePtr);
}

void Graph::addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight)
{
    Edge edge(n1, n2, weight);
    std::shared_ptr<Edge> edgePtr = std::make_shared<Edge>(edge);
    n1->addEdge(edgePtr);
    n2->addEdge(edgePtr);
}
