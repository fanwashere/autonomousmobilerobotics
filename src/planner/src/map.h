#pragma once

#include <nav_msgs/OccupancyGrid.h>

class Coordinate {
public:
    Coordinate() = default;
    Coordinate(int setX, int setY);

    int getX() const;
    int getY() const;
    double distanceTo(const Coordinate &target);
    double distanceTo(const Coordinate &target, double resolution);

private:
    int x;
    int y;
};

class Grid {
public:
    Grid() = default;
    Grid(int width, int height, float resolution, std::vector<int8_t> grid);
    virtual ~Grid() = default;

    float getResolution() const;
    Coordinate getRandomCoordinate() const;
    bool checkOccupancy(const Coordinate &coord);
    bool checkCollision(const Coordinate &from, const Coordinate &to);

private:
    std::vector<Coordinate> bresenham(const Coordinate &from, const Coordinate &to);

    int width;
    int height;
    float resolution;
    std::vector<int8_t> grid;
};

class MapHandler {
public:
    MapHandler() = default;
    virtual ~MapHandler() = default;

    std::shared_ptr<Grid> getGrid() const;
    void callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

private:
    std::shared_ptr<Grid> grid;
};
