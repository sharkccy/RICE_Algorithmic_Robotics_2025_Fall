#include "ManipulatorPlanner.h"
#include "RTP.h"
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <iostream>

ManipulatorPlanner::ManipulatorPlanner(std::shared_ptr<PyBulletInterface> simulator,
                                     const std::vector<double>& start,
                                     const std::vector<std::vector<double>>& goals,
                                     double planning_time)
    : simulator_(simulator), planning_time_(planning_time) {
    
    // Get joint limits from simulator
    auto limits = simulator_->getJointLimits();
    n_dims_ = limits.first.size();
    
    // Create the state space
    auto realVectorSpace = std::make_shared<ob::RealVectorStateSpace>(n_dims_);
    auto bounds = std::make_shared<ob::RealVectorBounds>(n_dims_);
    
    for (int i = 0; i < n_dims_; ++i) {
        bounds->setLow(i, limits.first[i]);
        bounds->setHigh(i, limits.second[i]);
    }
    
    realVectorSpace->setBounds(*bounds);
    space_ = realVectorSpace;
    
    // Create space information
    si_ = std::make_shared<ob::SpaceInformation>(space_);
    si_->setStateValidityChecker([this](const ob::State* state) {
        return this->isStateValid(state);
    });
    si_->setStateValidityCheckingResolution(0.0005);
    si_->setup();
    
    // Create simple setup
    ss_ = std::make_shared<og::SimpleSetup>(si_);
    
    // Set start state
    ob::ScopedState<> start_state(space_);
    for (int i = 0; i < n_dims_; ++i) {
        start_state[i] = start[i];
    }
    ss_->setStartState(start_state);
    
    // Set goal states
    auto goal_states = std::make_shared<ob::GoalStates>(si_);
    for (const auto& goal : goals) {
        ob::ScopedState<> goal_state(space_);
        for (int i = 0; i < n_dims_; ++i) {
            goal_state[i] = goal[i];
        }
        goal_states->addState(goal_state);
    }
    ss_->setGoal(goal_states);
}

void ManipulatorPlanner::setPlanner(ob::PlannerPtr planner) {
    current_planner_ = planner;
    ss_->setPlanner(planner);
}

std::vector<std::vector<double>> ManipulatorPlanner::solve() {
    // Solve the planning problem
    ob::PlannerStatus solved = ss_->solve(planning_time_);
    
    if (solved && ss_->haveSolutionPath()) {
        // Get raw solution path without simplification/interpolation
            og::PathGeometric path = ss_->getSolutionPath();
            
            // Convert to vector of configurations
            std::vector<std::vector<double>> result;
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                result.push_back(stateToConfig(path.getState(i)));
            }
            
            return result;
    }
    
    // Return empty vector if no solution found
    return {};
}

std::vector<std::vector<double>> ManipulatorPlanner::process() {
    
    if (ss_->haveSolutionPath()) {
        // Simplify and interpolate
        ss_->simplifySolution();
        og::PathGeometric path = ss_->getSolutionPath();
        path.interpolate(500);
        
        // Validate interpolated path
        for (size_t i = 1; i < path.getStateCount(); ++i) {
            if (!si_->checkMotion(path.getState(i-1), path.getState(i))) {
                std::cerr << "Warning: Interpolated path contains collision at waypoint " << i << std::endl;
            }
        }
        
        // Convert to vector of configurations
        std::vector<std::vector<double>> result;
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            result.push_back(stateToConfig(path.getState(i)));
        }
        
        return result;
    }
    
    return {};
}

bool ManipulatorPlanner::isStateValid(const ob::State* state) {
    // Convert state to configuration and check collision
    std::vector<double> config = stateToConfig(state);
    simulator_->setJointPositions(config);
    return !simulator_->inCollision();
}

std::vector<double> ManipulatorPlanner::stateToConfig(const ob::State* state) {
    const auto* real_state = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> config(n_dims_);
    for (int i = 0; i < n_dims_; ++i) {
        config[i] = real_state->values[i];
    }
    return config;
}

void ManipulatorPlanner::configToState(ob::State* state, const std::vector<double>& config) {
    auto* real_state = state->as<ob::RealVectorStateSpace::StateType>();
    for (int i = 0; i < n_dims_; ++i) {
        real_state->values[i] = config[i];
    }
}

void ManipulatorPlanner::setRTPParameters(double goal_bias) {
    // STUDENT TODO START
    // (void)goal_bias;
    if (current_planner_) {
        if (auto rtp = std::dynamic_pointer_cast<ompl::geometric::RTP>(current_planner_)) {
            rtp->setGoalBias(goal_bias);
        }
    }
    // STUDENT TODO END
}

void ManipulatorPlanner::setRRTParameters(double goal_bias, double range) {
    if (current_planner_) {
        // Try to cast to RRT planner and set parameters
        if (auto rrt = std::dynamic_pointer_cast<ompl::geometric::RRT>(current_planner_)) {
            rrt->setGoalBias(goal_bias);
            rrt->setRange(range);
        }
    }
}

void ManipulatorPlanner::setPRMParameters(int max_neighbors) {
    if (current_planner_) {
        // Try to cast to PRM planner and set parameters
        if (auto prm = std::dynamic_pointer_cast<ompl::geometric::PRM>(current_planner_)) {
            prm->setMaxNearestNeighbors(max_neighbors);
        }
    }
} 