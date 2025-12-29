#pragma once

#include <vector>
#include <string>
#include <memory>

/**
 * @brief Interface to PyBullet simulator for collision checking
 * 
 * This class provides a C++ interface to the PyBullet simulator
 * for collision checking of robotic manipulators.
 */
class PyBulletInterface {
public:
    /**
     * @brief Constructor
     * @param urdf_path Path to the robot URDF file
     * @param joint_names Names of the robot joints
     * @param gui Whether to enable GUI (should be false for headless operation)
     */
    PyBulletInterface(const std::string& urdf_path, 
                     const std::vector<std::string>& joint_names,
                     bool gui = false);
    
    /**
     * @brief Destructor
     */
    ~PyBulletInterface();
    
    /**
     * @brief Add environment from problem dictionary
     * @param problem_file Path to the problem YAML file
     */
    void addEnvironmentFromFile(const std::string& problem_file);
    
    /**
     * @brief Set joint positions
     * @param positions Joint positions vector
     */
    void setJointPositions(const std::vector<double>& positions);
    
    /**
     * @brief Check if current configuration is in collision
     * @return true if in collision, false otherwise
     */
    bool inCollision();
    
    /**
     * @brief Get joint limits
     * @return pair of vectors (lower_limits, upper_limits)
     */
    std::pair<std::vector<double>, std::vector<double>> getJointLimits();
    
    /**
     * @brief Load specific problem from problems.pkl
     * @param problem_name Name of the problem (e.g., "bookshelf_small")
     * @param index Problem instance index
     * @return pair of (start_config, goals_vector)
     */
    std::pair<std::vector<double>, std::vector<std::vector<double>>> loadProblem(
        const std::string& problem_name, int index);
    
    /**
     * @brief Add a box obstacle to the environment
     * @param half_extents Half extents [x, y, z] of the box
     * @param position Position [x, y, z] of the box center
     * @param orientation Quaternion [x, y, z, w] orientation (default: identity)
     */
    void addBox(const std::vector<double>& half_extents, 
                const std::vector<double>& position,
                const std::vector<double>& orientation = {0, 0, 0, 1});
    
    /**
     * @brief Add a sphere obstacle to the environment
     * @param radius Radius of the sphere
     * @param position Position [x, y, z] of the sphere center
     */
    void addSphere(double radius, const std::vector<double>& position);
    
    /**
     * @brief Add a cylinder obstacle to the environment
     * @param radius Radius of the cylinder
     * @param height Height of the cylinder
     * @param position Position [x, y, z] of the cylinder center
     * @param orientation Quaternion [x, y, z, w] orientation (default: identity)
     */
    void addCylinder(double radius, double height,
                     const std::vector<double>& position,
                     const std::vector<double>& orientation = {0, 0, 0, 1});
    
    /**
     * @brief Get minimum distance from robot to obstacles at given configuration
     * @param config Joint configuration to check
     * @return Minimum distance in meters (negative if in collision)
     */
    double getMinimumDistance(const std::vector<double>& config);
    
    /**
     * @brief Visualize trajectory by moving robot through path
     * @param path Vector of joint configurations
     * @param speed Playback speed multiplier (default: 1.0)
     */
    void visualizeTrajectory(const std::vector<std::vector<double>>& path, double speed = 1.0);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
}; 