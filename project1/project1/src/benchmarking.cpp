/*
 * Project 1
 * 
 * Benchmarking program for motion planners
 * 
 * This program benchmarks different OMPL planners on robotic manipulator problems.
 */

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include "ManipulatorPlanner.h"
#include "PyBulletInterface.h"

namespace po = boost::program_options;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

/**
 * @brief Get joint names for the robot
 */
std::vector<std::string> getJointNames(const std::string& robot) {
    if (robot == "ur5") {
        return {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    } else if (robot == "fetch") {
        // Match the exact joint names from Python version constants.py
        return {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", 
                "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", 
                "wrist_flex_joint", "wrist_roll_joint"};
    }
    return {};
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("problem", po::value<std::string>()->default_value("bookshelf_small"), "problem name")
        ("robot", po::value<std::string>()->default_value("ur5"), "robot type")
        ("index", po::value<int>()->default_value(1), "problem index")
        ("runtime_limit", po::value<double>()->default_value(30.0), "runtime limit per run")
        ("memory_limit", po::value<double>()->default_value(10000.0), "memory limit in MB")
        ("run_count", po::value<int>()->default_value(50), "number of runs per planner");
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    // Extract parameters
    std::string problem = vm["problem"].as<std::string>();
    std::string robot = vm["robot"].as<std::string>();
    int index = vm["index"].as<int>();
    double runtime_limit = vm["runtime_limit"].as<double>();
    double memory_limit = vm["memory_limit"].as<double>();
    int run_count = vm["run_count"].as<int>();
    
    std::cout << "OMPL Benchmarking - C++ Version" << std::endl;
    std::cout << "===============================" << std::endl;
    std::cout << "Problem: " << problem << std::endl;
    std::cout << "Robot: " << robot << std::endl;
    std::cout << "Index: " << index << std::endl;
    std::cout << "Runtime limit: " << runtime_limit << " seconds" << std::endl;
    std::cout << "Run count: " << run_count << std::endl;
    
    try {
        // Create robot interface
        std::string urdf_path = robot + "/" + robot + "_spherized.urdf";
        std::vector<std::string> joint_names = getJointNames(robot);
        
        auto simulator = std::make_shared<PyBulletInterface>(urdf_path, joint_names, false);
        
        // Load problem data from pickle file (this also loads the environment)
        auto [start, goals] = simulator->loadProblem(problem, index);
        
        std::cout << "\nLoaded problem data:" << std::endl;
        std::cout << "Start configuration: ";
        for (size_t i = 0; i < start.size(); ++i) {
            std::cout << start[i];
            if (i < start.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
        
        std::cout << "Goals (" << goals.size() << " total):" << std::endl;
        for (size_t i = 0; i < goals.size(); ++i) {
            std::cout << "  Goal " << i + 1 << ": ";
            for (size_t j = 0; j < goals[i].size(); ++j) {
                std::cout << goals[i][j];
                if (j < goals[i].size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
        
        // Create planner
        ManipulatorPlanner planner(simulator, start, goals, runtime_limit);
        
        // Create benchmark
        ot::Benchmark::Request request(runtime_limit, memory_limit, run_count);
        ot::Benchmark benchmark(*planner.getSimpleSetup(), problem);
        
        // Add planners to benchmark
        
        // RRTConnect with different ranges
        auto rrtc1 = std::make_shared<og::RRTConnect>(planner.getSpaceInformation());
        rrtc1->setName("RRTConnect Range 0.5");
        rrtc1->setRange(0.5);
        benchmark.addPlanner(rrtc1);
        
        auto rrtc2 = std::make_shared<og::RRTConnect>(planner.getSpaceInformation());
        rrtc2->setName("RRTConnect Range 5");
        rrtc2->setRange(5.0);
        benchmark.addPlanner(rrtc2);
        
        auto rrtc3 = std::make_shared<og::RRTConnect>(planner.getSpaceInformation());
        rrtc3->setName("RRTConnect Range 25");
        rrtc3->setRange(25.0);
        benchmark.addPlanner(rrtc3);
        
        // RRT with different ranges
        auto rrt1 = std::make_shared<og::RRT>(planner.getSpaceInformation());
        rrt1->setName("RRT Range 0.5");
        rrt1->setRange(0.5);
        benchmark.addPlanner(rrt1);
        
        auto rrt2 = std::make_shared<og::RRT>(planner.getSpaceInformation());
        rrt2->setName("RRT Range 5");
        rrt2->setRange(5.0);
        benchmark.addPlanner(rrt2);
        
        auto rrt3 = std::make_shared<og::RRT>(planner.getSpaceInformation());
        rrt3->setName("RRT Range 25");
        rrt3->setRange(25.0);
        benchmark.addPlanner(rrt3);
        
        // KPIECE with default settings
        auto kpiece = std::make_shared<og::KPIECE1>(planner.getSpaceInformation());
        kpiece->setName("KPIECE");
        benchmark.addPlanner(kpiece);
        
        // PRM with default settings
        auto prm = std::make_shared<og::PRM>(planner.getSpaceInformation());
        prm->setName("PRM");
        benchmark.addPlanner(prm);
        
        std::cout << "\nStarting benchmark..." << std::endl;
        std::cout << "This may take a while (up to " << (runtime_limit * run_count * 8 / 60.0) 
                  << " minutes)" << std::endl;
        
        // Run the benchmark
        benchmark.benchmark(request);
        
        // Save results
        std::cout << "\nBenchmark completed!" << std::endl;
        std::cout << "Results saved to log files." << std::endl;
        benchmark.saveResultsToFile();
        
        std::cout << "\nTo process results, run:" << std::endl;
        std::cout << "python3 ompl_benchmark_statistics.py *.log" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 