#include "CollisionChecking.h"
#include <cmath>
#include <vector>

// Axis aligned bounding box
bool AABB::pointInsideAABB(double x, double y, double rad) const {
    return x >= (minX - rad) && x <= (maxX + rad) && y >= (minY - rad) && y <= (maxY + rad);
}
bool AABB::pointInsideAABB(double x, double y) const {
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
}

bool lineLineIntersection(
    const Point2D &p1, const Point2D &p2, const Point2D &q1, const Point2D &q2) {
    // re-encode the lines as rays
    // p1 + t1*v1
    std::vector<double> v1(2);
    v1[0] = (p2.first - p1.first);
    v1[1] = (p2.second - p1.second);

    // q1 + t2*v2
    std::vector<double> v2(2);
    v2[0] = (q2.first - q1.first);
    v2[1] = (q2.second - q1.second);

    double det = (v1[0] * -v2[1]) - (-v2[0] * v1[1]);
    if (fabs(det) < 1e-6) { // rays are parallel.
        return false;
    }

    double dx = q1.first - p1.first;
    double dy = q1.second - p1.second;

    double t1 = ((v2[0] * dy) - (v2[1] * dx)) / det;
    if (fabs(t1) < 1e-6) {
        t1 = 0.0;
    }

    double t2 = ((v1[0] * dy) - (v1[1] * dx)) / det;
    if (fabs(t2) < 1e-6) {
        t2 = 0.0;
    }

    if (t1 < 0 || t2 < 0) { // Both scalars must be non-negative for an intersection
        return false;
    }

    if (t1 > 1 || t2 > 1) { // Both scalars must be <= 1 for an intersection
        return false;
    }

    return true;
}

AABB rectangleToAABB(const Rectangle &obstacle) {
    AABB rect { obstacle.x, obstacle.y, obstacle.x + obstacle.width, obstacle.y + obstacle.height };
    return rect;
}

// Robot is a point
/*bool isValidPoint(const ompl::base::State* state, const std::vector<AABB>& obstacles)
{
    const ompl::base::RealVectorStateSpace::StateType* R2State =
state->as<ompl::base::RealVectorStateSpace::StateType>(); double x = R2State->values[0]; double y =
R2State->values[1];

    for(size_t i = 0; i < obstacles.size(); ++i)
        if (obstacles[i].pointInsideAABB(x, y))
            return false;

    return true;
}
*/

bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles) {
    const ompl::base::RealVectorStateSpace::StateType *R2State
        = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = R2State->values[0];
    double y = R2State->values[1];

    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (rectangleToAABB(obstacles[i]).pointInsideAABB(x, y)) {
            return false;
        }
    }

    return true;
}

// Robot is a box
/*
bool isValidSquare(const ompl::base::State* state, const std::vector<AABB>& obstacles, double
sideLen, double bounds)
{
    const ompl::base::SE2StateSpace::StateType* se2State =
state->as<ompl::base::SE2StateSpace::StateType>(); double x = se2State->getX(); double y =
se2State->getY(); double theta = se2State->getYaw();

    // Local coordinates of a box
    std::vector<Point2D> pts;
    pts.push_back(std::make_pair(-sideLen/2.0, -sideLen/2.0));
    pts.push_back(std::make_pair( sideLen/2.0, -sideLen/2.0));
    pts.push_back(std::make_pair( sideLen/2.0,  sideLen/2.0));
    pts.push_back(std::make_pair(-sideLen/2.0,  sideLen/2.0));

    // Rigid transformation of the box
    for(size_t i = 0; i < pts.size(); ++i)
    {
        double newX = pts[i].first * cos(theta) - pts[i].second * sin(theta) + x;
        double newY = pts[i].first * sin(theta) + pts[i].second * cos(theta) + y;
        pts[i] = std::make_pair(newX, newY);

        // Make sure the new coordinate is in bounds
        if (newX < -bounds || newX > bounds)
            return false;
        if (newY < -bounds || newX > bounds)
            return false;
    }

    // Intersect each line segment of the box robot with each line segment of the obstacles
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        std::vector<Point2D> vertices;
        vertices.push_back(MAKE_POINT(obstacles[i].minX, obstacles[i].minY));
        vertices.push_back(MAKE_POINT(obstacles[i].maxX, obstacles[i].minY));
        vertices.push_back(MAKE_POINT(obstacles[i].maxX, obstacles[i].maxY));
        vertices.push_back(MAKE_POINT(obstacles[i].minX, obstacles[i].maxY));

        for(size_t j = 0; j < pts.size(); ++j)
        {
            // Line segment of the box
            Point2D b1 = pts[j];
            Point2D b2 = pts[(j+1) % pts.size()];

            for(size_t k = 0; k < vertices.size(); ++k)
            {
                // Line segment of the obstacle
                Point2D v1 = vertices[k];
                Point2D v2 = vertices[(k+1) % vertices.size()];

                // intersection?
                if (lineLineIntersection(b1, b2, v1, v2))
                    return false;
            }
        }
    }

    return true;
}
*/

bool isValidStateSquare(
    const ompl::base::State *state, double sideLen, const std::vector<Rectangle> &obstacles) {
    const ompl::base::SE2StateSpace::StateType *se2State
        = state->as<ompl::base::SE2StateSpace::StateType>();
    double x = se2State->getX();
    double y = se2State->getY();
    double theta = se2State->getYaw();

    // Local coordinates of a box
    std::vector<Point2D> pts;
    pts.push_back(std::make_pair(-sideLen / 2.0, -sideLen / 2.0));
    pts.push_back(std::make_pair(sideLen / 2.0, -sideLen / 2.0));
    pts.push_back(std::make_pair(sideLen / 2.0, sideLen / 2.0));
    pts.push_back(std::make_pair(-sideLen / 2.0, sideLen / 2.0));

    // Rigid transformation of the box
    for (size_t i = 0; i < pts.size(); ++i) {
        double newX = pts[i].first * cos(theta) - pts[i].second * sin(theta) + x;
        double newY = pts[i].first * sin(theta) + pts[i].second * cos(theta) + y;
        pts[i] = std::make_pair(newX, newY);
        /*
                // Make sure the new coordinate is in bounds
                if (newX < -bounds || newX > bounds)
                    return false;
                if (newY < -bounds || newX > bounds)
                    return false;
        */
    }

    // Intersect each line segment of the box robot with each line segment of the obstacles
    for (size_t i = 0; i < obstacles.size(); ++i) {
        std::vector<Point2D> vertices;
        vertices.push_back(
            MAKE_POINT(rectangleToAABB(obstacles[i]).minX, rectangleToAABB(obstacles[i]).minY));
        vertices.push_back(
            MAKE_POINT(rectangleToAABB(obstacles[i]).maxX, rectangleToAABB(obstacles[i]).minY));
        vertices.push_back(
            MAKE_POINT(rectangleToAABB(obstacles[i]).maxX, rectangleToAABB(obstacles[i]).maxY));
        vertices.push_back(
            MAKE_POINT(rectangleToAABB(obstacles[i]).minX, rectangleToAABB(obstacles[i]).maxY));

        for (size_t j = 0; j < pts.size(); ++j) {
            // Line segment of the box
            Point2D b1 = pts[j];
            Point2D b2 = pts[(j + 1) % pts.size()];

            for (size_t k = 0; k < vertices.size(); ++k) {
                // Line segment of the obstacle
                Point2D v1 = vertices[k];
                Point2D v2 = vertices[(k + 1) % vertices.size()];

                // intersection?
                if (lineLineIntersection(b1, b2, v1, v2)) {
                    return false;
                }
            }
        }
    }

    return true;
}
