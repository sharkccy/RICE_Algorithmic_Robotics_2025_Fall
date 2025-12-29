/*
 * Project 1 - C++ Version
 * 
 * Rigid Body Planning Example
 * 
 * This file demonstrates two ways to set up and solve a motion planning problem
 * using OMPL in C++.
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief State validity checker function
 * @param state State to check
 * @return true if state is valid, false otherwise
 */
bool isStateValid(const ob::State* state) {
    // Some arbitrary condition on the state
    const auto* se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    return pos->values[0] < 0.6;
}

/**
 * @brief Plan using SimpleSetup (easier method)
 */
void planWithSimpleSetup() {
    std::cout << "=== Planning with SimpleSetup ===" << std::endl;
    
    // Create an SE2 state space
    auto space = std::make_shared<ob::SE2StateSpace>();
    
    // Set lower and upper bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    
    // Create a simple setup object
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);
    
    // Create start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start.random();  // Pick a random start state
    start->setX(0.5);  // Set specific values
    
    // Create goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal.random();  // Pick a random goal state
    goal->setX(-0.5);  // Set specific values
    
    ss.setStartAndGoalStates(start, goal);
    
    // This will automatically choose a default planner with default parameters
    ob::PlannerStatus solved = ss.solve(1.0);
    
    if (solved) {
        // Try to shorten the path
        ss.simplifySolution();
        // Print the simplified path
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "No solution found" << std::endl;
    }
}

/**
 * @brief Plan the hard way (more control over the process)
 */
void planTheHardWay() {
    std::cout << "\n=== Planning the Hard Way ===" << std::endl;
    
    // Create an SE2 state space
    auto space = std::make_shared<ob::SE2StateSpace>();
    
    // Set lower and upper bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    
    // Construct an instance of space information from this state space
    auto si = std::make_shared<ob::SpaceInformation>(space);
    
    // Set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    
    // Create a random start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start.random();
    // Ensure start state is valid (X < 0.6)
    start->setX(0.3);  // Set to a valid X value
    
    // Create a random goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal.random();
    // Ensure goal state is valid (X < 0.6)
    goal->setX(-0.3);  // Set to a valid X value
    
    // Create a problem instance
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    
    // Create a planner for the defined space
    auto planner = std::make_shared<og::RRTConnect>(si);
    
    // Set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    
    // Perform setup steps for the planner
    planner->setup();
    
    // Print the settings for this space
    std::cout << "Space settings:" << std::endl;
    si->printSettings(std::cout);
    
    // Print the problem settings
    std::cout << "Problem definition:" << std::endl;
    pdef->print(std::cout);
    
    // Attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->solve(ob::PlannerTerminationCondition(ob::timedPlannerTerminationCondition(1.0)));
    
    if (solved) {
        // Get the goal representation from the problem definition and inquire about the found path
        auto path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        path->print(std::cout);
    } else {
        std::cout << "No solution found" << std::endl;
    }
}

int main() {
    std::cout << "OMPL Rigid Body Planning Demo" << std::endl;
    std::cout << "=============================" << std::endl;
    
    planWithSimpleSetup();
    planTheHardWay();
    
    return 0;
} 