#include "map.h"
#include "nav_msgs/OccupancyGrid.h"

double Coordinate::distanceTo(Coordinate target)
{
    int dx = abs(x - target.x);
    int dy = abs(y - target.y);

    return sqrt(pow(dx, 2) + pow(dy, 2));
}

double Coordinate::distanceTo(Coordinate target, double resolution)
{
    int dx = abs(x - target.x) * resolution;
    int dy = abs(y - target.y) * resolution;

    return sqrt(pow(dx, 2) + pow(dy, 2));
}

Grid::Grid(int width, int height, float r, std::vector<signed char> g)
{
    width = w;
    height = h;
    resolution = r;
    grid = g;
}

Coordinate Grid::getRandomCoordinate()
{
    int x = rand() % width;
    int y = rand() % height;
    return Coordinate(x, y);
}

bool Grid::checkOccupancy(Coordinate coord)
{
    int index = coord.y * width + coord.x;
    return grid[index]; // TODO Differentiate occupied or not
}

bool Grid::checkCollision(Coordinate from, Coordinate to)
{
    // Check validity and occupancy of from coordinate
    if (from.x < 0 || from.x >= width || checkOccupancy(from)) {
        return true;
    }

    // Check validity and occupancy of to coordinate
    if (to.x < 0 || to.y >= height || checkOccupancy(to)) {
        return true;
    }

    std::vector<Coordinate> lineCoordinates = bresenham(from, to);

    int i;
    for (i = 0; i < lineCoordinates.size(); i++) {
        Coordinate coord = lineCoordinates[i];

        if (checkOccupancy(coord)) {
            return true;
        }
    }

    return false;
}

std::vector<Coordinate> Grid::bresenham(Coordinate from, Coordinate to) {
    int x0 = from.x;
    int y0 = from.y;
    int x1 = to.x;
    int y1 = to.y;

    std::vector<Coordinate> lineCoordinates;
    int x = x0;
    int y = y0;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int s1 = sign(x1 - x0);
    int s2 = sign(y1 - y0);

    int swap = 0;
    if (dy > dx) {
        int temp = dx;
        dx = dy;
        dy = temp;
        swap = 1;
    }

    int D = 2 * dy - dx;
    int i;
    for (i = 0; i < dx; i++) {
        Coordinate coord(x, y);
        lineCoordinates.push_back(coord);

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

Grid MapHandler::getGrid()
{
    return grid;
}

void MapHandler::callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    grid = Grid(msg->info.width, msg->info.height, msg->info.resolution, msg->data);
}