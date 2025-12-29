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

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
}; 