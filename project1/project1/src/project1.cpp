/*
 * Project 1
 * 
 * Main manipulator planning program
 * 
 * This program runs motion planning for robotic manipulators using OMPL.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

#include "ManipulatorPlanner.h"
#include "PyBulletInterface.h"

namespace po = boost::program_options;
namespace og = ompl::geometric;

/**
 * @brief Get joint names for the robot
 */
std::vector<std::string> getJointNames(const std::string& robot) {
    if (robot == "ur5") {
        return {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    } else if (robot == "fetch") {
        return {"shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint",
                "wrist_roll_joint", "gripper_axis"};
    } else {
        throw std::runtime_error("Unknown robot type: " + robot);
    }
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("robot", po::value<std::string>()->default_value("ur5"), "robot type (ur5 or fetch)")
        ("planner", po::value<std::string>()->default_value("RRTConnect"), "planner type")
        ("problem", po::value<std::string>()->default_value("bookshelf_small"), "problem name")
        ("index", po::value<int>()->default_value(1), "problem index")
        ("planner_range", po::value<double>()->default_value(0.5), "planner range parameter")
        ("goal_bias", po::value<double>()->default_value(0.01), "goal bias parameter")
        ("max_nn", po::value<int>()->default_value(20), "max nearest neighbors for PRM")
        ("planning_time", po::value<double>()->default_value(30.0), "planning time limit")
        ("gui", po::value<bool>()->default_value(false), "enable GUI visualization");
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    // Extract parameters
    std::string robot = vm["robot"].as<std::string>();
    std::string planner_name = vm["planner"].as<std::string>();
    std::string problem = vm["problem"].as<std::string>();
    int index = vm["index"].as<int>();
    double planner_range = vm["planner_range"].as<double>();
    double goal_bias = vm["goal_bias"].as<double>();
    int max_nn = vm["max_nn"].as<int>();
    double planning_time = vm["planning_time"].as<double>();
    bool gui = vm["gui"].as<bool>();
    
    std::cout << "OMPL Manipulator Planning - C++ Version" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "Robot: " << robot << std::endl;
    std::cout << "Planner: " << planner_name << std::endl;
    std::cout << "Problem: " << problem << " (index " << index << ")" << std::endl;
    std::cout << "Planning time: " << planning_time << " seconds" << std::endl;
    std::cout << "GUI: " << (gui ? "enabled" : "disabled") << std::endl;
    
    try {
        // Create robot interface
        std::string urdf_path = robot + "/" + robot + "_spherized.urdf";
        std::vector<std::string> joint_names = getJointNames(robot);
        
        auto simulator = std::make_shared<PyBulletInterface>(urdf_path, joint_names, gui);
        
        // Load the specific problem from problems.pkl
        std::cout << "Loading problem " << problem << " (index " << index << ")..." << std::endl;
        auto [start, goals] = simulator->loadProblem(problem, index);
        
        if (start.empty() || goals.empty()) {
            throw std::runtime_error("Failed to load valid problem data");
        }
        
        std::cout << "Problem loaded successfully!" << std::endl;
        
        // Create planner
        ManipulatorPlanner planner(simulator, start, goals, planning_time);
        
        // Set up the specific planner
        ob::PlannerPtr ompl_planner;
        
        if (planner_name == "PRM") {
            auto prm = std::make_shared<og::PRM>(planner.getSpaceInformation());
            prm->setMaxNearestNeighbors(max_nn);
            ompl_planner = prm;
        } else if (planner_name == "RRT") {
            auto rrt = std::make_shared<og::RRT>(planner.getSpaceInformation());
            rrt->setRange(planner_range);
            rrt->setGoalBias(goal_bias);
            ompl_planner = rrt;
        } else if (planner_name == "RRTConnect") {
            auto rrtc = std::make_shared<og::RRTConnect>(planner.getSpaceInformation());
            rrtc->setRange(planner_range);
            ompl_planner = rrtc;
        } else if (planner_name == "KPIECE") {
            auto kpiece = std::make_shared<og::KPIECE1>(planner.getSpaceInformation());
            kpiece->setRange(planner_range);
            kpiece->setGoalBias(goal_bias);
            ompl_planner = kpiece;
        } else {
            std::cerr << "Unknown planner: " << planner_name << std::endl;
            return 1;
        }
        
        planner.setPlanner(ompl_planner);
        
        // Solve the problem
        std::cout << "\nSolving motion planning problem..." << std::endl;
        auto path = planner.solve();
        
        if (!path.empty()) {
            std::cout << "Solution found!" << std::endl;
            std::cout << "Path length: " << path.size() << " waypoints" << std::endl;
            
            // Print first and last configurations
            std::cout << "Start configuration: ";
            for (double val : path[0]) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
            
            std::cout << "Goal configuration: ";
            for (double val : path.back()) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
            
            // Save path to file
            std::ofstream path_file("solution_path.txt");
            path_file << path.size() << std::endl;
            for (const auto& config : path) {
                for (size_t i = 0; i < config.size(); ++i) {
                    path_file << config[i];
                    if (i < config.size() - 1) path_file << " ";
                }
                path_file << std::endl;
            }
            path_file.close();
            std::cout << "Path saved to solution_path.txt" << std::endl;
            
        } else {
            std::cout << "No solution found!" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 