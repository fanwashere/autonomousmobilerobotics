#pragma once

#include <nav_msgs/OccupancyGrid.h>

struct Coordinate
{
    int x = 0;
    int y = 0;

    Coordinate(int setX, int setY) {
        x = setX;
        y = setY;
    };

    double distanceTo(Coordinate target);
    double distanceTo(Coordinate target, double resolution);
};

class Grid
{
public:
    Grid() = default;
    virtual ~Grid() = default;
    Grid(int width, int height, float resolution, std::vector<signed char, std::allocator<signed char>> grid);
    Coordinate getRandomCoordinate();
    bool checkOccupancy(Coordinate coord);
    bool checkCollision(Coordinate from, Coordinate to);
    float getResolution();

private:
    std::vector<signed char, std::allocator<signed char>> grid;
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
