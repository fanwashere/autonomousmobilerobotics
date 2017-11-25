#include <ros/ros.h>
#include "map.h"

namespace {
    const int OBSTACLE_PROBABILITY = 100;
    const int OBSTACLE_PADDING_PROBABILITY = 80;
    constexpr int sign(int x) { return ((x > 0) ? 1 : ((x < 0) ? -1 : 0)); }
}

Coordinate::Coordinate(int setX, int setY)
: x(setX)
, y(setY)
{}

int Coordinate::getX() const {
    return x;
}

int Coordinate::getY() const {
    return y;
}

double Coordinate::distanceTo(const Coordinate &target) {
    return distanceTo(target, 1.0);
}

double Coordinate::distanceTo(const Coordinate &target, double resolution) {
    int dx = abs(x - target.getX()) * resolution;
    int dy = abs(y - target.getY()) * resolution;

    return sqrt(pow(dx, 2) + pow(dy, 2));
}

Grid::Grid(int w, int h, float r, std::vector<int8_t> g)
: width(w)
, height(h)
, resolution(r)
, grid(g)
{}

float Grid::getResolution() const {
    return resolution;
}

Coordinate Grid::getRandomCoordinate() const {
    const int x = rand() % width;
    const int y = rand() % height;

    return Coordinate(x, y);
}

bool Grid::checkOccupancy(const Coordinate &coord) {
    const int index = coord.getY() * width + coord.getX();

    return grid[index] >= OBSTACLE_PADDING_PROBABILITY;
}

bool Grid::checkCollision(const Coordinate &from, const Coordinate &to) {
    // Check validity and occupancy of from coordinate
    if (from.getX() < 0 || 
        from.getX() >= width || 
        from.getY() < 0 ||
        from.getY() >= height ||
        checkOccupancy(from)) {
        return true;
    }

    // Check validity and occupancy of to coordinate
    if (to.getX() < 0 ||
        to.getX() >= width ||
        to.getY() < 0 ||
        to.getY() >= height ||
        checkOccupancy(to)) {
        return true;
    }

    std::vector<Coordinate> lineCoordinates = bresenham(from, to);

    for (int i = 0; i < lineCoordinates.size(); i++) {
        Coordinate coord = lineCoordinates[i];

        if (checkOccupancy(coord)) {
            return true;
        }
    }

    return false;
}

void Grid::padObstacles(const int radius) {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Coordinate coord(x, y);

            if (getCell(coord) < OBSTACLE_PROBABILITY) {
                continue;
            }

            for (int kx = std::max(x - radius, 0); kx <= std::min(x + radius, width - 1); kx++) {
                for (int ky = std::max(y - radius, 0); ky <= std::min(y + radius, height - 1); ky++) {
                    Coordinate k(kx, ky);
                    if (getCell(k) < OBSTACLE_PROBABILITY) {
                        updateCell(k, OBSTACLE_PADDING_PROBABILITY);
                    }
                }
            }
        }
    }
}

std::vector<Coordinate> Grid::bresenham(const Coordinate &from, const Coordinate &to) {
    const int x0 = from.getX();
    const int y0 = from.getY();
    const int x1 = to.getX();
    const int y1 = to.getY();

    std::vector<Coordinate> lineCoordinates;
    int x = x0;
    int y = y0;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    const int s1 = sign(x1 - x0);
    const int s2 = sign(y1 - y0);

    int swap = 0;
    if (dy > dx) {
        int temp = dx;
        dx = dy;
        dy = temp;
        swap = 1;
    }

    int D = 2 * dy - dx;
    for (int i = 0; i < dx; i++) {
        lineCoordinates.emplace_back(x, y);

        while (D >= 0) {
            D = D - 2 * dx;
            if (swap) {
                x += s1;
            } else {
                y += s2;
            }
        }

        D = D + 2 * dy;
        if (swap) {
            y += s2;
        } else {
            x += s1;
        }
    }

    return lineCoordinates;
}

int Grid::getCell(const Coordinate &coord) {
    const int index = coord.getY() * width + coord.getX();

    return grid[index];
}

void Grid::updateCell(const Coordinate &coord, const int p) {
    const int index = coord.getY() * width + coord.getX();

    grid[index] = p;
}

bool MapHandler::hasData() const {
    return receivedData;
}

std::shared_ptr<Grid> MapHandler::getGrid() const {
    return grid;
}

void MapHandler::callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    ROS_INFO("Map recieved [w: %d, h: %d, r: %f]", msg->info.width, msg->info.height, msg->info.resolution);
    receivedData = true;
    grid = std::make_shared<Grid>(msg->info.width, msg->info.height, msg->info.resolution, msg->data);
}
