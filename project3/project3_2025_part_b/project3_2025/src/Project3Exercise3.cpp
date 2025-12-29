///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
//////////////////////////////////////

#include "RTP.h"
#include "ManipulatorPlanner.h"
#include "PyBulletInterface.h"
#include "PathAnalyzer.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <memory>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>

using namespace ompl;

/**
 * @brief Planner configuration structure
 * 
 *  parameters for all OMPL planners:
 * - type: Planner algorithm name ("RTP", "RRT", "RRT-Connect", "PRM", "EST")
 * - planning_time: Maximum time allowed for planning (seconds)
 * - num_trials: Number of independent runs for statistical analysis
 * 
 * Tree-based planner parameters (RTP, RRT, RRT-Connect, EST):
 * - goal_bias: Probability [0,1] of sampling towards goal vs random sampling
 *   Higher values = more direct paths but less exploration
 * - range: Maximum distance for extending tree towards new sample
 *   0.0 = auto-detect based on state space dimensions
 * 
 * Roadmap-based planner parameters (PRM):
 * - max_neighbors: Maximum number of nearest neighbors to connect each milestone
 *   Higher values = denser roadmap but slower construction
 */
struct PlannerConfig {
    std::string name;
    std::string type;                    // Planner algorithm name
    double planning_time;                // Time limit for planning (seconds)
    double goal_bias;                    // Goal sampling probability [0,1] (tree planners)
    double range;                        // Maximum extension distance (tree planners)
    int max_neighbors;                   // Max neighbors per milestone (PRM)
    
    PlannerConfig(const std::string& n, const std::string& t, double pt = 10.0, double gb = 0.05, 
                  double r = 0.3, int mn = 10)
        : name(n), type(t), planning_time(pt), goal_bias(gb), range(r), max_neighbors(mn) {}
};

/**
 * @brief Statistical results from multiple trials
 */
struct StatisticalResults {
    std::string planner_name;
    int total_trials;
    int successful_trials;
    double success_rate;
    
    // Average metrics
    double avg_planning_time;
    double avg_length;
    double avg_smoothness;
    double avg_clearance;
    
    // Standard deviations
    double std_planning_time;
    double std_length;
    double std_smoothness;
    double std_clearance;
    
    // Min/Max values
    double min_planning_time, max_planning_time;
    double min_length, max_length;
    double min_clearance, max_clearance;
};

// Forward declarations
PathMetrics testSingleTrialWithConfig(const PlannerConfig& config, std::shared_ptr<PyBulletInterface> simulator,
                                     const std::vector<double>& start_config, const std::vector<double>& goal_config);

/**
 * @brief Create environment with primitive shapes
 */
void setupCustomEnvironment(std::shared_ptr<PyBulletInterface> simulator) {
    
    simulator->addSphere(0.15, {0.55, 0, 1.0});
    simulator->addSphere(0.15, {0.35, 0.35, 1});
    simulator->addSphere(0.15, {0, 0.55, 1});
    simulator->addSphere(0.15, {-0.55, 0, 1});
    simulator->addSphere(0.15, {-0.35, -0.35, 1});
    simulator->addSphere(0.15, {0, -0.55, 1});
    simulator->addSphere(0.15, {0.35, -0.35, 1});
    simulator->addSphere(0.15, {0.35, 0.35, 1.5});
    simulator->addSphere(0.15, {0, 0.55, 1.5});
    simulator->addSphere(0.15, {-0.35, 0.35, 1.5});
    simulator->addSphere(0.15, {-0.55, 0, 1.5});
    simulator->addSphere(0.15, {-0.35, -0.35, 1.5});
    simulator->addSphere(0.15, {0, -0.55, 1.5});
    simulator->addSphere(0.15, {0.35, -0.35, 1.5});

    
    std::cout << "Environment setup complete" << std::endl;
}

/**
 * @brief Calculate statistics from multiple trials
 */
StatisticalResults calculateStatistics(const std::vector<PathMetrics>& trials, const std::string& planner_name) {
    StatisticalResults stats;
    stats.planner_name = planner_name;
    stats.total_trials = static_cast<int>(trials.size());
    stats.successful_trials = 0;
    
    std::vector<double> planning_times, lengths, smoothness_values, clearances;
    
    // Collect successful trial data
    for (const auto& trial : trials) {
        if (trial.success) {
            stats.successful_trials++;
            planning_times.push_back(trial.planning_time * 1000.0); // Convert to ms
            lengths.push_back(trial.length);
            smoothness_values.push_back(trial.smoothness);
            clearances.push_back(trial.min_clearance);
        }
    }
    
    stats.success_rate = static_cast<double>(stats.successful_trials) / stats.total_trials * 100.0;
    
    if (stats.successful_trials > 0) {
        // Calculate averages
        auto avg = [](const std::vector<double>& v) { 
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size(); 
        };
        
        stats.avg_planning_time = avg(planning_times);
        stats.avg_length = avg(lengths);
        stats.avg_smoothness = avg(smoothness_values);
        stats.avg_clearance = avg(clearances);
        
        
        // Calculate min/max values
        stats.min_planning_time = *std::min_element(planning_times.begin(), planning_times.end());
        stats.max_planning_time = *std::max_element(planning_times.begin(), planning_times.end());
        stats.min_length = *std::min_element(lengths.begin(), lengths.end());
        stats.max_length = *std::max_element(lengths.begin(), lengths.end());
        stats.min_clearance = *std::min_element(clearances.begin(), clearances.end());
        stats.max_clearance = *std::max_element(clearances.begin(), clearances.end());
    } else {
        // No successful trials - set all to zero
        stats.avg_planning_time = stats.avg_length = stats.avg_smoothness = stats.avg_clearance = 0.0;
        stats.min_planning_time = stats.max_planning_time = 0.0;
        stats.min_length = stats.max_length = 0.0;
        stats.min_clearance = stats.max_clearance = 0.0;
    }
    
    return stats;
}



/**
 * @brief Test a single trial with specific start and goal configurations
 */
PathMetrics testSingleTrialWithConfig(const PlannerConfig& config,
                                     std::shared_ptr<PyBulletInterface> simulator,
                                     const std::vector<double>& start_config,
                                     const std::vector<double>& goal_config,
                                     bool save = true) {
    
    // Create fresh planner instance with specific start and goal configurations
    std::vector<std::vector<double>> goals = {goal_config};
    auto planner = std::make_shared<ManipulatorPlanner>(simulator, start_config, goals, config.planning_time);
    
    // Create and configure planner
    base::PlannerPtr ompl_planner;
    
    if (config.type == "RTP") {
        ompl_planner = std::make_shared<geometric::RTP>(planner->getSpaceInformation());
        planner->setPlanner(ompl_planner);
        planner->setRTPParameters(config.goal_bias);
    } else if (config.type == "RRT") {
        ompl_planner = std::make_shared<geometric::RRT>(planner->getSpaceInformation());
        planner->setPlanner(ompl_planner);
        planner->setRRTParameters(config.goal_bias, config.range);
    } else if (config.type == "RRT-Connect") {
        ompl_planner = std::make_shared<geometric::RRTConnect>(planner->getSpaceInformation());
        planner->setPlanner(ompl_planner);
        planner->setRRTParameters(config.goal_bias, config.range);
    } else if (config.type == "PRM") {
        ompl_planner = std::make_shared<geometric::PRM>(planner->getSpaceInformation());
        planner->setPlanner(ompl_planner);
        planner->setPRMParameters(config.max_neighbors);
    }
    
    // planner->setPlanner(ompl_planner);
    
    // Time the planning
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> path = planner->solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double planning_time = std::chrono::duration<double>(end_time - start_time).count();

    // Process and save path
    std::string filename = config.name + "_path.txt";
    std::replace(filename.begin(), filename.end(), ' ', '_');
    std::replace(filename.begin(), filename.end(), '-', '_');
    if (!path.empty() && save){
        std::vector<std::vector<double>> path = planner->process();
        PathAnalyzer::savePath(path, filename);
    }
    
    // Analyze path
    PathMetrics metrics = PathAnalyzer::analyzeCompletePath(path, simulator, planning_time);
    
    return metrics;
}

/**
 * @brief Main function for manipulator planning
 */
int main(int /* argc */, char** /* argv */) {
    std::cout << "COMP 450 - Project 3 Exercise 3: Manipulator Planning \n";
    
    try {
        // Initialize PyBullet interface
        std::vector<std::string> joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        auto simulator = std::make_shared<PyBulletInterface>(
            "provided/ur5/ur5.urdf", joint_names, false);  // Headless mode for Docker
        
        // Setup custom environment
        setupCustomEnvironment(simulator);

        // Define 5 different start and goal configuration pairs for comprehensive testing
        std::vector<std::pair<std::vector<double>, std::vector<double>>> config_pairs = {
            // Pair 1
            {{-1.653, -1.205, 1.058, 0.198, 1.190, -0.231}, {1.091, -1.025, 1.058, 0.198, 1.190, -0.231}},
            // Pair 2
            {{-2.135, -0.397, -1.687, 0.430, 0.0, 0.0}, {1.157, -0.463, -2.447, 0.430, 0.00, 0.0}},
            // Pair 3
            {{-1.587, -0.529, -0.033, 0.231, 0.0, 0.0}, {-2.000, -1.000, 0.800, -0.800, 0.000, 0.500}},
            // Pair 4
            {{-1.885, -0.785, -0.524, -1.571, 0.0, 0.0}, {1.488, -0.364, -0.198, -1.455, -1.290, -0.926}},
            // Pair 5
            {{0.132, -0.860, 1.124, -1.571, 0.0, 0.0}, {-2.546, -0.827, 1.257, -2.679, 0.0, 0.0}}
        };
        
        // Number of trials per planner.
        int num_trials = 5;


        /* STUDENT TODO START: 
            Modify the parameters of the different types of planners below to compare their performance.
            You can compare multiple configurations of the same planner by giving them unique names, as 
            shown in the RTP example below. You can also add different types of planners by uncommenting
            the lines below. Please do not modify the number of trials or the start/goal configurations
            when submitting your scores to the leaderboard. 
        */
        
        // Define planner configurations for comparison
        // Each planner will run 5 trials on each of the 5 start/goal pairs (25 total trials per planner)
        std::vector<PlannerConfig> configs = {
            
            // ========== RTP (Random Tree Planner) ==========
            // Students will implement RTP class in src/RTP.cpp
            // Parameters: planning_time, goal_bias
            // - planning_time: Maximum time allowed for planning (seconds)
            // - goal_bias: Probability [0,1] of sampling towards goal vs random sampling

            // PlannerConfig("RTP-00", "RTP", 5.0, 0.00),
            // PlannerConfig("RTP-01", "RTP", 5.0, 0.01),
            // PlannerConfig("RTP-02", "RTP", 5.0, 0.02),
            // PlannerConfig("RTP-05", "RTP", 5.0, 0.05),
            // PlannerConfig("RTP-08", "RTP", 5.0, 0.08),
            // PlannerConfig("RTP-10", "RTP", 5.0, 0.10),
            // PlannerConfig("RTP-20", "RTP", 5.0, 0.20),
            // PlannerConfig("RTP-40", "RTP", 5.0, 0.40),
            // PlannerConfig("RTP-60", "RTP", 5.0, 0.60),
            // PlannerConfig("RTP-80", "RTP", 5.0, 0.80),
            // PlannerConfig("RTP-90", "RTP", 5.0, 0.90),
            // PlannerConfig("RTP-100", "RTP", 5.0, 1.00),    

            // ========== RRT (Rapidly-exploring Random Tree) ==========
            // Parameters: planning_time, goal_bias, range
            // - planning_time: Maximum time allowed for planning (seconds)
            // - goal_bias: Probability [0,1] of sampling towards goal (higher = more direct paths)
            // - range: Maximum distance for extending tree (0.0 = auto-detect based on state space)
            //PlannerConfig("RRT-1", "RRT", 5.0, 0.02, 0.2),


            // PlannerConfig("RRT-0.0", "RRT", 5.0, 0.0, 0.5),
            // PlannerConfig("RRT-0.01", "RRT", 5.0, 0.01, 0.5),
            // PlannerConfig("RRT-0.02", "RRT", 5.0, 0.02, 0.5),           
            // PlannerConfig("RRT-0.05", "RRT", 5.0, 0.05, 0.5),
            // PlannerConfig("RRT-0.08", "RRT", 5.0, 0.08, 0.5),
            // PlannerConfig("RRT-0.1", "RRT", 5.0, 0.1, 0.5),
            // PlannerConfig("RRT-0.2", "RRT", 5.0, 0.2, 0.5),
            // PlannerConfig("RRT-0.4", "RRT", 5.0, 0.4, 0.5),
            // PlannerConfig("RRT-0.6", "RRT", 5.0, 0.6, 0.5),
            // PlannerConfig("RRT-0.8", "RRT", 5.0, 0.8, 0.5),
            // PlannerConfig("RRT-0.9", "RRT", 5.0, 0.9, 0.5),
            // PlannerConfig("RRT-1.0", "RRT", 5.0, 1.0, 0.5),

            
            // ========== RRT-Connect (Bidirectional RRT) ==========
            // Parameters: planning_time, goal_bias, range
            // - planning_time: Maximum time allowed for planning (seconds)
            // - goal_bias: Not used in RRT-Connect (bidirectional growth handles goal bias)
            // - range: Maximum distance for extending trees (0.0 = auto-detect)
            // Note: RRT-Connect grows trees from both start and goal simultaneously
            // PlannerConfig("RRT-Connect-0.00", "RRT-Connect", 5.0, 0.00, 0.5),
            // PlannerConfig("RRT-Connect-0.01", "RRT-Connect", 5.0, 0.01, 0.5),
            // PlannerConfig("RRT-Connect-0.02", "RRT-Connect", 5.0, 0.02, 0.5),
            // PlannerConfig("RRT-Connect-0.05", "RRT-Connect", 5.0, 0.05, 0.5),
            // PlannerConfig("RRT-Connect-0.08", "RRT-Connect", 5.0, 0.08, 0.5),
            // PlannerConfig("RRT-Connect-0.1", "RRT-Connect", 5.0, 0.1, 0.5),
            // PlannerConfig("RRT-Connect-0.2", "RRT-Connect", 5.0, 0.2, 0.5),
            // PlannerConfig("RRT-Connect-0.4", "RRT-Connect", 5.0, 0.4, 0.5),
            // PlannerConfig("RRT-Connect-0.6", "RRT-Connect", 5.0, 0.6, 0.5),
            // PlannerConfig("RRT-Connect-0.8", "RRT-Connect", 5.0, 0.8, 0.5),
            // PlannerConfig("RRT-Connect-0.9", "RRT-Connect", 5.0, 0.9, 0.5),
            // PlannerConfig("RRT-Connect-1.0", "RRT-Connect", 5.0, 1.0, 0.5),
            
            // ========== PRM (Probabilistic Roadmap) ==========
            // Parameters: planning_time, goal_bias (unused), range (unused), max_neighbors
            // - planning_time: Time for roadmap construction + query phase (seconds)
            // - max_neighbors: Maximum number of nearest neighbors to connect to each milestone
            // Note: PRM builds a roadmap of valid configurations, then searches for path
            
            // PlannerConfig("PRM-1", "PRM", 5.0, 0.0, 0.0, 1),
            // PlannerConfig("PRM-5", "PRM", 5.0, 0.0, 0.0, 5),
            // PlannerConfig("PRM-10", "PRM", 5.0, 0.0, 0.0, 10),
            // PlannerConfig("PRM-15", "PRM", 5.0, 0.0, 0.0, 15),
            // PlannerConfig("PRM-20", "PRM", 5.0, 0.0, 0.0, 20),
            // PlannerConfig("PRM-30", "PRM", 5.0, 0.0, 0.0, 30),
            // PlannerConfig("PRM-40", "PRM", 5.0, 0.0, 0.0, 40),
            // PlannerConfig("PRM-50", "PRM", 5.0, 0.0, 0.0, 50),
            // PlannerConfig("PRM-60", "PRM", 5.0, 0.0, 0.0, 60),
            // PlannerConfig("PRM-80", "PRM", 5.0, 0.0, 0.0, 80),
            // PlannerConfig("PRM-100", "PRM", 5.0, 0.0, 0.0, 100),
            // PlannerConfig("PRM-150", "PRM", 5.0, 0.0, 0.0, 150),
            // PlannerConfig("PRM-150", "PRM", 5.0, 0.0, 0.0, 200),
            // PlannerConfig("PRM-250", "PRM", 5.0, 0.0, 0.0, 250),
            // PlannerConfig("PRM-300", "PRM", 5.0, 0.0, 0.0, 300),
            PlannerConfig("PRM-150", "PRM", 5.0, 0.0, 0.0, 175),
            
        };

        /* STUDENT TODO END */

        std::cout << " Testing Methodology:\n";
        std::cout << "- Each planner will be tested on " << config_pairs.size() << " different start/goal configuration pairs\n";
        std::cout << "- For each configuration pair, the planner will run " << num_trials << " independent trials\n";
        std::cout << "- Total trials per planner: " << num_trials << " × " << config_pairs.size() << " = " << (num_trials * config_pairs.size()) << " trials\n";
        std::cout << "- Average metrics will be calculated across all successful trials\n\n";
        
        std::cout << " Start/Goal Configuration Pairs:\n";
        for (size_t i = 0; i < config_pairs.size(); ++i) {
            std::cout << "  Pair " << (i+1) << ":\n";
            std::cout << "    Start: ";
            for (double val : config_pairs[i].first) std::cout << std::fixed << std::setprecision(3) << val << " ";
            std::cout << "\n    Goal:  ";
            for (double val : config_pairs[i].second) std::cout << std::fixed << std::setprecision(3) << val << " ";
            std::cout << "\n";
        }
        
        
        // Test all planner configurations with multiple trials on all start/goal pairs
        std::vector<StatisticalResults> all_stats;
        
        for (const auto& config : configs) {
            std::cout << "\n" << std::string(80, '=') << "\n";
            std::cout << "TESTING PLANNER: " << config.name << "\n";
            std::cout << std::string(80, '=') << "\n";
            
            // Aggregate results across all configuration pairs
            std::vector<PathMetrics> all_trials;
            bool path_saved = false;
            
            for (size_t pair_idx = 0; pair_idx < config_pairs.size(); ++pair_idx) {
                std::cout << "\n Configuration Pair " << (pair_idx + 1) << "/5\n";
                std::cout << "Start: ";
                for (double val : config_pairs[pair_idx].first) std::cout << std::fixed << std::setprecision(3) << val << " ";
                std::cout << "\nGoal:  ";
                for (double val : config_pairs[pair_idx].second) std::cout << std::fixed << std::setprecision(3) << val << " ";
                std::cout << "\n";
                
                // Run multiple trials for this configuration pair
                for (int trial = 0; trial < num_trials; ++trial) {
                    std::cout << "  Trial " << (trial + 1) << "/" << num_trials << ": ";
                    
                    PathMetrics metrics = testSingleTrialWithConfig(config, simulator, 
                                                                   config_pairs[pair_idx].first, 
                                                                   config_pairs[pair_idx].second,
                                                                   !path_saved);
                    all_trials.push_back(metrics);
                    
                    if (metrics.success) {
                        std::cout << "SUCCESS (time: " << std::fixed << std::setprecision(1) << metrics.planning_time 
                                  << "ms, length: " << std::setprecision(3) << metrics.length << ")\n";
                        path_saved = true; // Stop saving once successful path has been found
                    } else {
                        std::cout << "FAILED\n";
                    }
                }
            }
            
            // Calculate statistics across all trials from all configuration pairs
            StatisticalResults stats = calculateStatistics(all_trials, config.name);
            all_stats.push_back(stats);
            
            if (!path_saved) {
                std::cout << "Warning: Could not generate path file for " << config.name << " (no successful trials)\n";
            }
            
            // Print summary for this planner
            std::cout << "\n " << config.name << " SUMMARY (" << num_trials << " trials × " << config_pairs.size() << " config pairs = " << stats.total_trials << " total trials):\n";
            std::cout << "Success Rate: " << stats.successful_trials << "/" << stats.total_trials 
                      << " (" << std::fixed << std::setprecision(1) << stats.success_rate << "%)\n";
            if (stats.successful_trials > 0) {
                std::cout << "Average Planning Time: " << std::setprecision(1) << stats.avg_planning_time << " ms\n";
                std::cout << "Average Path Length: " << std::setprecision(3) << stats.avg_length << " radians\n";
                std::cout << "Average Smoothness: " << std::setprecision(3) << stats.avg_smoothness << "\n";
                std::cout << "Average Clearance: " << std::setprecision(1) << stats.avg_clearance << " cm\n";
            }
        }
        
        // Summary analysis
        std::cout << "\n PLANNING COMPARISON SUMMARY\n";
        std::cout << "==============================\n";
        
        // Find best performers across all configurations
        double fastest_avg_time = std::numeric_limits<double>::max();
        double shortest_avg_path = std::numeric_limits<double>::max();
        double smoothest_avg_path = std::numeric_limits<double>::max();
        double best_avg_clearance = 0.0;
        double highest_success_rate = 0.0;
        
        std::string fastest_planner, shortest_planner, smoothest_planner, clearest_planner, most_reliable_planner;
        
        for (const auto& stats : all_stats) {
            if (stats.successful_trials > 0) {
                if (stats.avg_planning_time < fastest_avg_time) {
                    fastest_avg_time = stats.avg_planning_time;
                    fastest_planner = stats.planner_name;
                }
                if (stats.avg_length < shortest_avg_path) {
                    shortest_avg_path = stats.avg_length;
                    shortest_planner = stats.planner_name;
                }
                if (stats.avg_smoothness < smoothest_avg_path) {
                    smoothest_avg_path = stats.avg_smoothness;
                    smoothest_planner = stats.planner_name;
                }
                if (stats.avg_clearance > best_avg_clearance) {
                    best_avg_clearance = stats.avg_clearance;
                    clearest_planner = stats.planner_name;
                }
            }
            if (stats.success_rate > highest_success_rate) {
                highest_success_rate = stats.success_rate;
                most_reliable_planner = stats.planner_name;
            }
        }
        
        std::cout << std::fixed;
        std::cout << " Fastest Planner: " << fastest_planner << " (" << std::setprecision(1) << fastest_avg_time << " ms avg)\n";
        std::cout << " Shortest Paths: " << shortest_planner << " (" << std::setprecision(3) << shortest_avg_path << " radians avg)\n";
        std::cout << " Smoothest Paths: " << smoothest_planner << " (" << smoothest_avg_path << " curvature avg)\n";
        std::cout << "  Best Clearance: " << clearest_planner << " (" << std::setprecision(1) << best_avg_clearance << " cm avg)\n";
        std::cout << "  Most Reliable: " << most_reliable_planner << " (" << highest_success_rate << "% success rate)\n";
        
        // Overall statistics
        int total_trials = 0, total_successful = 0;
        for (const auto& stats : all_stats) {
            total_trials += stats.total_trials;
            total_successful += stats.successful_trials;
        }
        std::cout << "\n Overall Success Rate: " << total_successful << "/" << total_trials 
                  << " (" << std::setprecision(1) << (100.0 * total_successful / total_trials) << "%)\n";
        
        // Detailed metrics report for each planner
        std::cout << "\n" << std::string(80, '=') << "\n";
        std::cout << "DETAILED PLANNER METRICS REPORT\n";
        std::cout << std::string(80, '=') << "\n";
        
        for (const auto& stats : all_stats) {
            std::cout << "\n " << stats.planner_name << " DETAILED METRICS:\n";
            std::cout << std::string(50, '-') << "\n";
            
            // Success metrics
            std::cout << " Success Metrics:\n";
            std::cout << "   Success Rate: " << stats.successful_trials << "/" << stats.total_trials 
                      << " (" << std::fixed << std::setprecision(1) << stats.success_rate << "%)\n";
            
            if (stats.successful_trials > 0) {
                // Performance metrics
                std::cout << "\n Performance Metrics:\n";
                std::cout << "   Average Planning Time: " << std::setprecision(1) << stats.avg_planning_time << " ms\n";
                std::cout << "   Planning Time Range: " << stats.min_planning_time << " - " << stats.max_planning_time << " ms\n";
                
                // Path quality metrics
                std::cout << "\n Path Quality Metrics:\n";
                std::cout << "   Average Path Length: " << std::setprecision(3) << stats.avg_length << " radians\n";
                std::cout << "   Path Length Range: " << stats.min_length << " - " << stats.max_length << " radians\n";
                std::cout << "   Average Smoothness: " << std::setprecision(3) << stats.avg_smoothness << " (lower = smoother)\n";
                
                // Safety metrics
                std::cout << "\n  Safety Metrics:\n";
                std::cout << "   Average Clearance: " << std::setprecision(1) << stats.avg_clearance << " cm\n";
                std::cout << "   Clearance Range: " << stats.min_clearance << " - " << stats.max_clearance << " cm\n";
                
                // Performance ranking
                std::cout << "\n Performance Ranking:\n";
                if (stats.planner_name == fastest_planner) std::cout << "    FASTEST planner\n";
                if (stats.planner_name == shortest_planner) std::cout << "    SHORTEST paths\n";
                if (stats.planner_name == smoothest_planner) std::cout << "    SMOOTHEST paths\n";
                if (stats.planner_name == clearest_planner) std::cout << "    BEST clearance\n";
                if (stats.planner_name == most_reliable_planner) std::cout << "    MOST reliable\n";
                
            } else {
                std::cout << "\n No successful trials - unable to compute performance metrics\n";
            }
        }
        
        std::cout << "\n Generated Files:\n";
        for (const auto& config : configs) {
            std::string filename = config.name + "_path.txt";
            std::replace(filename.begin(), filename.end(), ' ', '_');
            std::replace(filename.begin(), filename.end(), '-', '_');
            std::cout << "- " << filename << std::endl;
        }   
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}