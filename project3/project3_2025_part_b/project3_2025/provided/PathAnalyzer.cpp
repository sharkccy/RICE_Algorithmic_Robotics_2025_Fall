#include "PathAnalyzer.h"
#include "PyBulletInterface.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <limits>

double PathAnalyzer::calculateLength(const std::vector<std::vector<double>>& path) {
    if (path.size() < 2) return 0.0;
    
    double total_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double segment_length = 0.0;
        for (size_t j = 0; j < path[i].size(); ++j) {
            double diff = path[i][j] - path[i-1][j];
            segment_length += diff * diff;
        }
        total_length += std::sqrt(segment_length);
    }
    return total_length;
}

double PathAnalyzer::calculateSmoothness(const std::vector<std::vector<double>>& path) {
    if (path.size() < 3) return 0.0;
    
    double total_curvature = 0.0;
    
    // Calculate curvature-based smoothness using second derivatives
    for (size_t i = 1; i < path.size() - 1; ++i) {
        double joint_curvature = 0.0;
        
        for (size_t j = 0; j < path[i].size(); ++j) {
            // Second derivative approximation: q''[i] = q[i+1] - 2*q[i] + q[i-1]
            double second_derivative = path[i+1][j] - 2.0 * path[i][j] + path[i-1][j];
            joint_curvature += second_derivative * second_derivative;
        }
        
        total_curvature += std::sqrt(joint_curvature);
    }
    
    return total_curvature;
}

double PathAnalyzer::calculateMinClearance(const std::vector<std::vector<double>>& path, 
                                          std::shared_ptr<PyBulletInterface> simulator) {
    if (path.empty() || !simulator) return 0.0;
    
    double min_clearance = std::numeric_limits<double>::max();
    
    // Sample path at regular intervals for clearance calculation
    size_t sample_count = std::min(static_cast<size_t>(50), path.size());
    size_t step = std::max(static_cast<size_t>(1), path.size() / sample_count);
    
    for (size_t i = 0; i < path.size(); i += step) {
        try {
            double clearance = simulator->getMinimumDistance(path[i]);
            if (clearance >= 0.0) {  // Valid distance measurement
                min_clearance = std::min(min_clearance, clearance);
            }
        } catch (...) {
            // Skip invalid configurations
            continue;
        }
    }
    
    return (min_clearance == std::numeric_limits<double>::max()) ? 0.0 : min_clearance;
}

PathMetrics PathAnalyzer::analyzeCompletePath(const std::vector<std::vector<double>>& path,
                                             std::shared_ptr<PyBulletInterface> simulator,
                                             double planning_time) {
    PathMetrics metrics;
    
    if (path.empty()) {
        metrics.success = false;
        return metrics;
    }
    
    metrics.success = true;
    metrics.waypoint_count = static_cast<int>(path.size());
    metrics.planning_time = planning_time;
    metrics.length = calculateLength(path);
    metrics.smoothness = calculateSmoothness(path);
    
    // Calculate clearance if simulator is available
    if (simulator) {
        metrics.min_clearance = calculateMinClearance(path, simulator);
    }
    
    return metrics;
}

void PathAnalyzer::printAnalysis(const std::string& planner_name, const PathMetrics& metrics) {
    std::cout << "\n==================================================\n";
    std::cout << "Testing Planner: " << planner_name << "\n";
    std::cout << "==================================================\n";
    
    if (metrics.success) {
        std::cout << " Planning SUCCEEDED\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Planning Time: " << (metrics.planning_time * 1000.0) << " ms\n";
        std::cout << "Path Length: " << metrics.length << " radians\n";
        std::cout << "Path Smoothness: " << metrics.smoothness << " (lower is smoother)\n";
        std::cout << "Min Clearance: " << (metrics.min_clearance * 100.0) << " cm\n";
        std::cout << "Waypoint Count: " << metrics.waypoint_count << "\n";
    } else {
        std::cout << " Planning FAILED\n";
        std::cout << "Planning Time: " << (metrics.planning_time * 1000.0) << " ms\n";
    }
    
    std::cout << "--------------------------------------------------\n";
}

void PathAnalyzer::savePath(const std::vector<std::vector<double>>& path, 
                           const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not save path to " << filename << std::endl;
        return;
    }
    
    // Write header comment
    file << "# Path file - Joint configurations (radians)\n";
    file << "# Format: joint1 joint2 joint3 joint4 joint5 joint6\n";
    
    // Write path waypoints
    for (const auto& config : path) {
        for (size_t i = 0; i < config.size(); ++i) {
            file << std::fixed << std::setprecision(6) << config[i];
            if (i < config.size() - 1) file << " ";
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "Path saved to: " << filename << std::endl;
} 