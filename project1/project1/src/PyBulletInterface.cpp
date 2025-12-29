#include "PyBulletInterface.h"
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/select.h>
#include <errno.h>
#include <cstring>

class PyBulletInterface::Impl {
public:
    std::string urdf_path_;
    std::vector<std::string> joint_names_;
    bool gui_;
    std::vector<double> joint_lows_;
    std::vector<double> joint_highs_;
    
    // Persistent Python process
    int python_pid_;
    int pipe_to_python_[2];   // C++ -> Python
    int pipe_from_python_[2]; // Python -> C++
    bool python_process_active_;
    
    Impl(const std::string& urdf_path, 
         const std::vector<std::string>& joint_names,
         bool gui) 
        : urdf_path_(urdf_path), joint_names_(joint_names), gui_(gui), 
          python_pid_(-1), python_process_active_(false) {
        
        // Initialize joint limits based on robot type
        initializeJointLimits();
        
        // Start persistent Python process
        startPythonProcess();
    }
    
    ~Impl() {
        stopPythonProcess();
    }
    
    void initializeJointLimits() {
        // Determine robot type from URDF path
        if (urdf_path_.find("ur5") != std::string::npos) {
            // UR5 joint limits (in radians) - matching Python version
            joint_lows_ = {-6.28318, -6.28318, -3.14159, -6.28318, -6.28318, -6.28318};
            joint_highs_ = {6.28318, 6.28318, 3.14159, 6.28318, 6.28318, 6.28318};
        } else if (urdf_path_.find("fetch") != std::string::npos) {
            // Fetch joint limits (in radians and meters for prismatic joints)
            // Matching the Python version's joint limits
            joint_lows_ = {0.0, -1.6056, -1.221, -3.14159, -2.251, -3.14159, -2.16, -3.14159};
            joint_highs_ = {0.4, 1.6056, 1.518, 3.14159, 2.251, 3.14159, 2.16, 3.14159};
        } else {
            // Default limits - more conservative
            joint_lows_.resize(joint_names_.size(), -6.28318);
            joint_highs_.resize(joint_names_.size(), 6.28318);
        }
        
        // Ensure we have the right number of limits
        if (joint_lows_.size() != joint_names_.size()) {
            joint_lows_.resize(joint_names_.size(), -6.28318);
            joint_highs_.resize(joint_names_.size(), 6.28318);
        }
    }
    
    void startPythonProcess() {
        // Create pipes for communication
        if (pipe(pipe_to_python_) == -1 || pipe(pipe_from_python_) == -1) {
            throw std::runtime_error("Failed to create pipes for Python communication");
        }
        
        python_pid_ = fork();
        if (python_pid_ == -1) {
            throw std::runtime_error("Failed to fork Python process");
        }
        
        if (python_pid_ == 0) {
            // Child process - run Python
            close(pipe_to_python_[1]);   // Close write end
            close(pipe_from_python_[0]); // Close read end
            
            // Redirect stdin/stdout to pipes
            dup2(pipe_to_python_[0], STDIN_FILENO);
            dup2(pipe_from_python_[1], STDOUT_FILENO);
            
            // Close original pipe ends
            close(pipe_to_python_[0]);
            close(pipe_from_python_[1]);
            
            // Execute Python collision checker
            execlp("python3", "python3", "collision_server.py", nullptr);
            
            // If we get here, exec failed
            std::cerr << "Failed to start Python collision server" << std::endl;
            exit(1);
        } else {
            // Parent process
            close(pipe_to_python_[0]);   // Close read end
            close(pipe_from_python_[1]); // Close write end
            
            python_process_active_ = true;
            
            // Initialize the Python simulator
            initializePythonSimulator();
        }
    }
    
    void stopPythonProcess() {
        if (python_process_active_ && python_pid_ > 0) {
            // Send quit command
            sendCommand("QUIT\n");
            
            // Close pipes
            close(pipe_to_python_[1]);
            close(pipe_from_python_[0]);
            
            // Wait for process to finish or kill it
            int status;
            if (waitpid(python_pid_, &status, WNOHANG) == 0) {
                // Process still running, kill it
                kill(python_pid_, SIGTERM);
                waitpid(python_pid_, &status, 0);
            }
            
            python_process_active_ = false;
        }
    }
    
    void initializePythonSimulator() {
        std::ostringstream cmd;
        cmd << "INIT " << urdf_path_ << " " << (gui_ ? "true" : "false");
        for (const auto& joint : joint_names_) {
            cmd << " " << joint;
        }
        cmd << "\n";
        
        sendCommand(cmd.str());
        
        std::string response = readResponse();
        if (response != "OK") {
            throw std::runtime_error("Failed to initialize Python simulator: " + response);
        }
    }
    
    void sendCommand(const std::string& command) {
        if (!python_process_active_) {
            throw std::runtime_error("Python process not active");
        }
        
        ssize_t written = write(pipe_to_python_[1], command.c_str(), command.length());
        if (written != static_cast<ssize_t>(command.length())) {
            throw std::runtime_error("Failed to write command to Python process");
        }
    }
    
    std::string readResponse(int timeout_seconds = 120) {
        if (!python_process_active_) {
            throw std::runtime_error("Python process not active");
        }
        
        // Use select() to implement timeout
        fd_set read_fds;
        struct timeval timeout;
        
        FD_ZERO(&read_fds);
        FD_SET(pipe_from_python_[0], &read_fds);
        
        timeout.tv_sec = timeout_seconds;
        timeout.tv_usec = 0;
        
        int select_result = select(pipe_from_python_[0] + 1, &read_fds, nullptr, nullptr, &timeout);
        
        if (select_result == -1) {
            throw std::runtime_error("Select failed: " + std::string(strerror(errno)));
        } else if (select_result == 0) {
            // Timeout occurred
            std::cerr << "Warning: Python process response timed out after " << timeout_seconds << " seconds" << std::endl;
            return "VIDEO_SKIPPED";
        }
        
        char buffer[1024];
        ssize_t bytes_read = read(pipe_from_python_[0], buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) {
            throw std::runtime_error("Failed to read response from Python process");
        }
        
        buffer[bytes_read] = '\0';
        std::string response(buffer);
        
        // Remove trailing newline
        if (!response.empty() && response.back() == '\n') {
            response.pop_back();
        }
        
        return response;
    }
};

PyBulletInterface::PyBulletInterface(const std::string& urdf_path, 
                                   const std::vector<std::string>& joint_names,
                                   bool gui) 
    : pimpl_(std::make_unique<Impl>(urdf_path, joint_names, gui)) {
}

PyBulletInterface::~PyBulletInterface() = default;

void PyBulletInterface::addEnvironmentFromFile(const std::string& problem_file) {
    std::string command = "LOAD_ENV " + problem_file + "\n";
    pimpl_->sendCommand(command);
    
    std::string response = pimpl_->readResponse();
    if (response != "OK") {
        std::cerr << "Warning: Failed to load environment: " << response << std::endl;
    }
}

void PyBulletInterface::setJointPositions(const std::vector<double>& positions) {
    std::ostringstream cmd;
    cmd << "SET_JOINTS";
    for (double pos : positions) {
        cmd << " " << pos;
    }
    cmd << "\n";
    
    pimpl_->sendCommand(cmd.str());
    
    std::string response = pimpl_->readResponse();
    if (response != "OK") {
        std::cerr << "Warning: Failed to set joint positions: " << response << std::endl;
    }
}

bool PyBulletInterface::inCollision() {
    pimpl_->sendCommand("CHECK_COLLISION\n");
    
    std::string response = pimpl_->readResponse();
    return (response == "true");
}

std::pair<std::vector<double>, std::vector<double>> PyBulletInterface::getJointLimits() {
    return {pimpl_->joint_lows_, pimpl_->joint_highs_};
}

std::pair<std::vector<double>, std::vector<std::vector<double>>> PyBulletInterface::loadProblem(
    const std::string& problem_name, int index) {
    
    std::ostringstream cmd;
    cmd << "LOAD_PROBLEM " << problem_name << " " << index << "\n";
    pimpl_->sendCommand(cmd.str());
    
    std::string response = pimpl_->readResponse();
    
    if (response.substr(0, 5) == "ERROR") {
        throw std::runtime_error("Failed to load problem: " + response);
    }
    
    // Parse response: START start_config GOALS goal1 goal2 ... GOAL_END
    std::istringstream iss(response);
    std::string token;
    std::vector<double> start;
    std::vector<std::vector<double>> goals;
    
    // Read START
    iss >> token;
    if (token != "START") {
        throw std::runtime_error("Invalid response format: expected START");
    }
    
    // Read start configuration
    while (iss >> token && token != "GOALS") {
        start.push_back(std::stod(token));
    }
    
    // Read goals
    std::vector<double> current_goal;
    while (iss >> token) {
        if (token == "GOAL_END") {
            if (!current_goal.empty()) {
                goals.push_back(current_goal);
                current_goal.clear();
            }
        } else {
            current_goal.push_back(std::stod(token));
        }
    }
    
    return {start, goals};
} 