#pragma once

#include <math.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

struct Coordinate
{
    int x = 0;
    int y = 0;

    Coordinate(int setX, int setY) {
        x = setX;
        y = setY;
    }
}

class Grid
{
public:
    Grid(int w, int h, float r, std::vector<signed char> g);
    Coordinate getRandomCoordinate();
    bool checkOccupancy(Coordinate coord);
    bool checkCollision(Coordinate from, Coordinate to);
    double getDistance(Coordinate from, Coordinate to);

private:
    std::vector<signed char> grid;
    int width;
    int height;
    float resolution;
    std::vector<Coordinate> bresenham(Coordinate from, Coordinate to);
};

class MapHandler
{
public:
    MapHandler() = default;
    virtual ~MapHandler() = default;

    Grid getGrid() const;
    void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:
    Grid grid;
};
