#pragma once

#include <nav_msgs/OccupancyGrid.h>

struct Coordinate
{
    int x;
    int y;

    Coordinate(int setX, int setY);

    double distanceTo(Coordinate target);
    double distanceTo(Coordinate target, double resolution);
};

class Grid
{
public:
    Grid() = default;
    virtual ~Grid() = default;
    Grid(int width, int height, float resolution, std::vector<int8_t> grid);

    float getResolution() const;
    Coordinate getRandomCoordinate() const;
    bool checkOccupancy(const Coordinate &coord);
    bool checkCollision(const Coordinate &from, const Coordinate &to);

private:
    std::vector<int8_t> grid;
    int width;
    int height;
    float resolution;

    std::vector<Coordinate> bresenham(const Coordinate &from, const Coordinate &to);
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
