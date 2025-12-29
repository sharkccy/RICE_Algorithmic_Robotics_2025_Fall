/*
 * Project 1 - C++ Version
 * 
 * Rigid Body Planning with Controls Example
 * 
 * This file demonstrates motion planning for a car-like system using
 * first-order controls (steering velocity and forward velocity).
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <iostream>
#include <cmath>

namespace ob = ompl::base;
namespace oc = ompl::control;

/**
 * @brief Custom decomposition for SyclopEST planner
 */
class MyDecomposition : public oc::GridDecomposition {
public:
    MyDecomposition(int length, const ob::RealVectorBounds& bounds)
        : GridDecomposition(length, 2, bounds) {}
    
    void project(const ob::State* s, std::vector<double>& coord) const override {
        const auto* se2state = s->as<ob::SE2StateSpace::StateType>();
        const auto* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        coord[0] = pos->values[0];
        coord[1] = pos->values[1];
    }
    
    void sampleFullState(const ob::StateSamplerPtr& sampler, 
                        const std::vector<double>& coord, 
                        ob::State* s) const override {
        sampler->sampleUniform(s);
        auto* se2state = s->as<ob::SE2StateSpace::StateType>();
        auto* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        pos->values[0] = coord[0];
        pos->values[1] = coord[1];
    }
};

/**
 * @brief State validity checker
 * @param spaceInformation Space information object
 * @param state State to check
 * @return true if state is valid, false otherwise
 */
bool isStateValid(const oc::SpaceInformation* spaceInformation, const ob::State* state) {
    // Perform collision checking or check if other constraints are satisfied
    return spaceInformation->satisfiesBounds(state);
}

/**
 * @brief State propagation function
 * @param start Starting state
 * @param control Control to apply
 * @param duration Duration to apply control
 * @param result Resulting state
 */
void propagate(const ob::State* start, const oc::Control* control, 
               const double duration, ob::State* result) {
    const auto* se2start = start->as<ob::SE2StateSpace::StateType>();
    const auto* pos = se2start->as<ob::RealVectorStateSpace::StateType>(0);
    double yaw = se2start->as<ob::SO2StateSpace::StateType>(1)->value;
    
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
    auto* se2result = result->as<ob::SE2StateSpace::StateType>();
    auto* result_pos = se2result->as<ob::RealVectorStateSpace::StateType>(0);
    
    result_pos->values[0] = pos->values[0] + ctrl[0] * duration * std::cos(yaw);
    result_pos->values[1] = pos->values[1] + ctrl[0] * duration * std::sin(yaw);
    se2result->as<ob::SO2StateSpace::StateType>(1)->value = yaw + ctrl[1] * duration;
}

/**
 * @brief Main planning function
 */
void plan() {
    std::cout << "=== Planning with Controls ===" << std::endl;
    
    // Construct the state space we are planning in
    auto space = std::make_shared<ob::SE2StateSpace>();
    
    // Set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    
    // Create a control space
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    
    // Set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    cspace->setBounds(cbounds);
    
    // Define a simple setup class
    oc::SimpleSetup ss(cspace);
    
    // Set state validity checker
    ss.setStateValidityChecker([&ss](const ob::State* state) {
        return isStateValid(ss.getSpaceInformation().get(), state);
    });
    
    // Set state propagator
    ss.setStatePropagator(propagate);
    
    // Create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);
    
    // Create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.0);
    goal->setY(0.5);
    goal->setYaw(0.0);
    
    // Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    // (optionally) set planner
    auto si = ss.getSpaceInformation();
    
    // SyclopEST requires a decomposition to guide the search
    auto decomp = std::make_shared<MyDecomposition>(32, bounds);
    auto planner = std::make_shared<oc::SyclopEST>(si, decomp);
    ss.setPlanner(planner);
    
    // (optionally) set propagation step size
    si->setPropagationStepSize(0.1);
    
    // Attempt to solve the problem
    ob::PlannerStatus solved = ss.solve(20.0);
    
    if (solved) {
        // Print the path to screen
        std::cout << "Found solution:" << std::endl;
        auto path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
    } else {
        std::cout << "No solution found" << std::endl;
    }
}

int main() {
    std::cout << "OMPL Rigid Body Planning with Controls Demo" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    plan();
    
    return 0;
} 