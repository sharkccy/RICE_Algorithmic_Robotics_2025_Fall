///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Chen-En Lin, Vincent Chang
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cstdio> // for std::remove

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a point robot.
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(100);
    space->setBounds(bounds);

    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    si->setStateValidityChecker([&obstacles](const ompl::base::State *s) -> bool {
        return isValidStatePoint(s, obstacles);
    });

    si->setup();

    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    // start->values[0] = 10;
    // start->values[1] = 10;
    // goal->values[0] = 90;
    // goal->values[1] = 90;

    // start->values[0] = 90;
    // start->values[1] = 10;
    // goal->values[0] = 10;
    // goal->values[1] = 80;

    start->values[0] = 50;
    start->values[1] = 10;
    goal->values[0] = 10;
    goal->values[1] = 90;
    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<ompl::geometric::RTP>(si));
    planner->setProblemDefinition(pdef);
    // set goal bias here (probability of sampling the goal instead of a random state)
    // default in RTP is 0.05; change the value below to increase/decrease goal sampling
    planner->setGoalBias(0.05);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->solve(ompl::base::timedPlannerTerminationCondition(60.0));

    if (solved)
    {
        std::cout << "Point: solution found\n";
        if (pdef->hasSolution())
        {
            auto path = pdef->getSolutionPath();
            path->print(std::cout);
            // export obstacles and path to CSV so an external visualizer can read them
            std::ofstream obsout("obstacles.csv");
            if (obsout)
            {
                for (const auto &o : obstacles)
                    obsout << o.x << ',' << o.y << ',' << o.width << ',' << o.height << '\n';
            }

            auto gpath = std::static_pointer_cast<ompl::geometric::PathGeometric>(path);
            std::ofstream pout("path_point.csv");
            if (pout)
            {
                for (std::size_t i = 0; i < gpath->getStateCount(); ++i)
                {
                    const ompl::base::State *s = gpath->getState(i);
                    const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
                    pout << rv->values[0] << ',' << rv->values[1] << '\n';
                }
            }
        }
    }
    else
    {
        std::cout << "Point: no solution\n";
    }

}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    auto space(std::make_shared<ompl::base::SE2StateSpace>());
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(100);
    space->setBounds(bounds);
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    const double sideLen = 5.0; 
    si->setStateValidityChecker([&obstacles, sideLen](const ompl::base::State *s) -> bool {
        return isValidStateSquare(s, sideLen, obstacles);
    });

    si->setup();

    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    // start->setXY(10, 10);
    // start->setYaw(0);
    // goal->setXY(90, 90);
    // goal->setYaw(0);

    // start->setXY(90, 10);
    // start->setYaw(0);
    // goal->setXY(10, 80);
    // goal->setYaw(0);

    start->setXY(50, 10);
    start->setYaw(0);
    goal->setXY(10, 90);
    goal->setYaw(0);
    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<ompl::geometric::RTP>(si));
    planner->setProblemDefinition(pdef);
    // set goal bias here (probability of sampling the goal instead of a random state)
    // default in RTP is 0.05; change the value below to increase/decrease goal sampling
    planner->setGoalBias(0.05);
    planner->setup();
    ompl::base::PlannerStatus solved = planner->solve(ompl::base::timedPlannerTerminationCondition(60.0));
    if (solved)
    {
        std::cout << "Box: solution found\n";
        if (pdef->hasSolution())
        {
            auto path = pdef->getSolutionPath();
            path->print(std::cout);
            // export obstacles and path to CSV so an external visualizer can read them
            std::ofstream obsout("obstacles.csv");
            if (obsout)
            {
                for (const auto &o : obstacles)
                    obsout << o.x << ',' << o.y << ',' << o.width << ',' << o.height << '\n';
            }

            auto gpath = std::static_pointer_cast<ompl::geometric::PathGeometric>(path);
            std::ofstream bout("path_box.csv");
            if (bout)
            {
                for (std::size_t i = 0; i < gpath->getStateCount(); ++i)
                {
                    const ompl::base::State *s = gpath->getState(i);
                    const auto *se2 = s->as<ompl::base::SE2StateSpace::StateType>();
                    double x = se2->getX();
                    double y = se2->getY();
                    double yaw = se2->getYaw();
                    bout << x << ',' << y << ',' << yaw << '\n';
                }
            }
        }
    }
    else
    {
        std::cout << "Box: no solution\n";
    }
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    obstacles.clear();
    obstacles.push_back({30, 30, 8, 8});   // small block near center
    obstacles.push_back({60, 20, 10, 10}); // small block lower-right
    obstacles.push_back({20, 70, 12, 6});  // small block upper-left
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    obstacles.clear();

    // Left vertical wall with a small gap between y=35..45 (gap height = 10)
    obstacles.push_back({30, 0, 10, 35});   // lower part of left wall
    obstacles.push_back({30, 45, 10, 55});  // upper part of left wall

    // Middle vertical wall with small gap between y=60..70 (gap height = 10)
    obstacles.push_back({55, 0, 10, 60});   // lower part of middle wall
    obstacles.push_back({55, 70, 10, 30});  // upper part of middle wall (to top)

}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
