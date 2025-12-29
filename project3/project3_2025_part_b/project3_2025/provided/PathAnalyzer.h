#pragma once

#include <vector>
#include <string>
#include <memory>

// Forward declaration
class PyBulletInterface;

/**
 * @brief Path analysis metrics structure
 */
struct PathMetrics {
    double length = 0.0;           // Total path length in joint space
    double smoothness = 0.0;       // Curvature-based smoothness measure
    double min_clearance = 0.0;    // Minimum distance to obstacles
    double planning_time = 0.0;    // Planning time in seconds
    int waypoint_count = 0;        // Number of waypoints
    bool success = false;          // Planning success flag
};

/**
 * @brief Path analysis utilities for motion planning
 * 
 * Provides mathematically sound metrics for evaluating path quality
 * including length, smoothness, and obstacle clearance.
 */
class PathAnalyzer {
public:
    /**
     * @brief Calculate Euclidean path length in joint space
     * @param path Vector of joint configurations
     * @return Total path length
     */
    static double calculateLength(const std::vector<std::vector<double>>& path);
    
    /**
     * @brief Calculate path smoothness using curvature-based measure
     * @param path Vector of joint configurations
     * @return Smoothness metric (lower is smoother)
     */
    static double calculateSmoothness(const std::vector<std::vector<double>>& path);
    
    /**
     * @brief Calculate minimum clearance to obstacles along path
     * @param path Vector of joint configurations
     * @param simulator PyBullet interface for distance queries
     * @return Minimum distance to obstacles in meters
     */
    static double calculateMinClearance(const std::vector<std::vector<double>>& path, 
                                       std::shared_ptr<PyBulletInterface> simulator);
    
    /**
     * @brief Analyze complete path and return all metrics
     * @param path Vector of joint configurations
     * @param simulator PyBullet interface for clearance calculation
     * @param planning_time Time taken for planning
     * @return Complete path metrics
     */
    static PathMetrics analyzeCompletePath(const std::vector<std::vector<double>>& path,
                                          std::shared_ptr<PyBulletInterface> simulator,
                                          double planning_time);
    
    /**
     * @brief Print formatted analysis results
     * @param planner_name Name of the planner
     * @param metrics Path metrics to display
     */
    static void printAnalysis(const std::string& planner_name, const PathMetrics& metrics);
    
    /**
     * @brief Save path to file for visualization
     * @param path Vector of joint configurations
     * @param filename Output filename
     */
    static void savePath(const std::vector<std::vector<double>>& path, 
                        const std::string& filename);
}; 