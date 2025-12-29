///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: vc68 Vincent Chang
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for(Rectangle o : obstacles){
        if(o.x <= x && x <= o.x + o.width){
            if(o.y <= y && y <= o.y + o.height){
                return false;
            }
        }
    } 

    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    // Take care of extended weight and height separately
    for(Rectangle o : obstacles){
        if(o.x - radius <= x && x <= o.x + o.width + radius){
            if(o.y <= y && y <= o.y + o.height){
                return false;
            }
        }

        if(o.x <= x && x <= o.x + o.width){
            if(o.y <= y + radius && y <= o.y + o.height + radius){
                return false;
            }
        }

        //corner case for 4 vertices (sqrt2R > R so use euclidean distance to tell whether it's valid) 
        if(sqrt( pow((x - o.x), 2) + pow((y - o.y - o.height), 2)) <= radius ||
            sqrt( pow((x - o.x - o.width), 2) + pow((y - o.y - o.height), 2)) <= radius ||
            sqrt( pow((x - o.x - o.width), 2) + pow((y - o.y), 2)) <= radius ||
            sqrt( pow((x - o.x), 2) + pow((y - o.y), 2)) <= radius
            )
        {
            return false;
        }
    }
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    // Put the reference point of robot to the origin and get 4 vertices' coordinates
    double half_side_length = sideLength / 2; 
    std::vector<std::pair<double, double>> robot_vertices = {   
        {-half_side_length, -half_side_length},
        {half_side_length, -half_side_length},
        {half_side_length, half_side_length},
        {-half_side_length, half_side_length},
    };

    double cos_t = cos(theta);
    double sin_t = sin(theta);

    // Perform rotation and then tranlate back to x,y
    for(auto& pair : robot_vertices){
        double rotatedx = cos_t * pair.first - sin_t * pair.second + x;
        double rotatedy = sin_t * pair.first + cos_t * pair.second + y;
        pair.first = rotatedx;
        pair.second = rotatedy;
    }

    // Calculate 4 normals of robot's 4 edges
    std::vector<std::pair<double, double>> robot_normal = {};
    for(int i = 0; i < 4; i++){
        double x1 = robot_vertices[i].first;
        double y1 = robot_vertices[i].second;
        double x2 = robot_vertices[(i + 1) % 4].first;
        double y2 = robot_vertices[(i + 1) % 4].second;
        double nx = -(y2-y1);
        double ny = (x2-x1);
        robot_normal.push_back(std::make_pair(nx, ny));
    }

    std::vector<std::vector<std::pair<double, double>>> all_obstacles_vertices = {};
    std::vector<std::vector<std::pair<double, double>>> all_obstacles_normals = {};

    // Calculate 4 normals of obstacle 4 edges for every obstacle
    for(const auto& o : obstacles){
        std::vector<std::pair<double, double>> cur_obstacle_vetices = {};
        std::vector<std::pair<double, double>> cur_obstacle_normals = {};
        cur_obstacle_vetices.push_back(std::make_pair(o.x, o.y));
        cur_obstacle_vetices.push_back(std::make_pair(o.x + o.width, o.y));
        cur_obstacle_vetices.push_back(std::make_pair(o.x + o.width, o.y + o.height));
        cur_obstacle_vetices.push_back(std::make_pair(o.x, o.y + o.height));

        all_obstacles_vertices.push_back(cur_obstacle_vetices);

        for(int i = 0; i < 4; i++){
            double x1 = cur_obstacle_vetices[i].first;
            double y1 = cur_obstacle_vetices[i].second;
            double x2 = cur_obstacle_vetices[(i + 1) % 4].first;
            double y2 = cur_obstacle_vetices[(i + 1) % 4].second;
            double nx = -(y2-y1);
            double ny = (x2-x1);
            cur_obstacle_normals.push_back(std::make_pair(nx, ny));
        }

        all_obstacles_normals.push_back(cur_obstacle_normals);
    }

    /*
    Find the projection of 4 robot's vertices and 4 obstacle's vertices on every normals 
    to find one to separate them.
    */

    for(int i = 0; i < all_obstacles_vertices.size(); i++){ //every obstacle
        bool intersect = true;
        double max_value = 1e9;
        for(const auto& cur_obs_nor : all_obstacles_normals[i]){    //4 obstacle normal
            double minObstacle = max_value;
            double maxObstacle = -max_value;
            double minRobot = max_value;
            double maxRobot = -max_value;
            double nx = cur_obs_nor.first;
            double ny = cur_obs_nor.second;

            for(const auto& cur_obs_ver : all_obstacles_vertices[i]){ //4 obstacle vertices
                double projection = cur_obs_ver.first * nx + cur_obs_ver.second * ny;
                minObstacle = std::min(minObstacle, projection);
                maxObstacle = std::max(maxObstacle, projection);
            }

            for(const auto& rob_ver : robot_vertices){  //4 robot vertices
                double projection = rob_ver.first * nx + rob_ver.second * ny;
                minRobot = std::min(minRobot, projection);
                maxRobot = std::max(maxRobot, projection);
            }  

            if(maxObstacle < minRobot || maxRobot < minObstacle){
                intersect = false;
                break;
            }
        }

        if(intersect){
            for(const auto& rob_nor : robot_normal){    //4 robot normal
                double minObstacle = max_value;
                double maxObstacle = -max_value;
                double minRobot = max_value;
                double maxRobot = -max_value;
                double nx = rob_nor.first;
                double ny = rob_nor.second;

                for(const auto& cur_obs_ver : all_obstacles_vertices[i]){ //4 obstacle vertices
                double projection = cur_obs_ver.first * nx + cur_obs_ver.second * ny;
                minObstacle = std::min(minObstacle, projection);
                maxObstacle = std::max(maxObstacle, projection);
                }

                for(const auto& rob_ver : robot_vertices){  //4 robot vertices
                    double projection = rob_ver.first * nx + rob_ver.second * ny;
                    minRobot = std::min(minRobot, projection);
                    maxRobot = std::max(maxRobot, projection);
                }  

                if(maxObstacle < minRobot || maxRobot < minObstacle){
                    intersect = false;
                    break;
                }
            }
        }

        if(intersect){
            return false;
        }
    }
    return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> &robots, const std::vector<Rectangle> &obstacles,
               const std::vector<bool> &valid)
{
}
