# Project 1 - C++ Version Makefile

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
# Try multiple possible OMPL installation paths
OMPL_PATHS = /usr/local/include/ompl-1.7 /usr/local/include/ompl /usr/include/ompl
OMPL_INCLUDE = $(firstword $(wildcard $(OMPL_PATHS)))
INCLUDES = -Iinclude -I$(OMPL_INCLUDE) -I/usr/include/eigen3
LIBS = -L/usr/local/lib -lompl -lboost_system -lboost_filesystem -lboost_program_options -lyaml-cpp

# Source and object directories
SRCDIR = src
INCDIR = include
OBJDIR = obj

# Source files
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Executables
RIGID_BODY = rigid_body_planning
RIGID_BODY_CONTROLS = rigid_body_planning_controls
PROJECT1 = project1
BENCHMARKING = benchmarking

# Main targets
all: check-deps $(RIGID_BODY) $(RIGID_BODY_CONTROLS) $(PROJECT1) $(BENCHMARKING)

# Check dependencies
check-deps:
	@echo "Checking dependencies..."
	@if [ -z "$(OMPL_INCLUDE)" ]; then \
		echo "Error: OMPL headers not found in any of: $(OMPL_PATHS)"; \
		echo "Please install OMPL or run 'make install-deps-docker'"; \
		exit 1; \
	else \
		echo "Found OMPL headers at: $(OMPL_INCLUDE)"; \
	fi
	@if ! pkg-config --exists yaml-cpp; then \
		echo "Warning: yaml-cpp not found via pkg-config, trying system libraries"; \
	fi

# Create object directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Object file compilation
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Rigid body planning executable
$(RIGID_BODY): $(OBJDIR)/RigidBodyPlanning.o
	$(CXX) $(CXXFLAGS) $^ $(LIBS) -o $@

# Rigid body planning with controls executable
$(RIGID_BODY_CONTROLS): $(OBJDIR)/RigidBodyPlanningWithControls.o
	$(CXX) $(CXXFLAGS) $^ $(LIBS) -o $@

# Main project1 executable
$(PROJECT1): $(OBJDIR)/project1.o $(OBJDIR)/ManipulatorPlanner.o $(OBJDIR)/PyBulletInterface.o
	$(CXX) $(CXXFLAGS) $^ $(LIBS) -o $@

# Benchmarking executable
$(BENCHMARKING): $(OBJDIR)/benchmarking.o $(OBJDIR)/ManipulatorPlanner.o $(OBJDIR)/PyBulletInterface.o
	$(CXX) $(CXXFLAGS) $^ $(LIBS) -o $@

# Test targets
test-rigid-body: $(RIGID_BODY)
	@echo "Running rigid body planning test..."
	./$(RIGID_BODY)

test-rigid-body-controls: $(RIGID_BODY_CONTROLS)
	@echo "Running rigid body planning with controls test..."
	./$(RIGID_BODY_CONTROLS)

test-project1: $(PROJECT1)
	@echo "Running project1 test..."
	./$(PROJECT1) --help

# Run examples with better error handling
run-ur5-rrt: $(PROJECT1)
	@echo "Running UR5 with RRT planner..."
	@if ./$(PROJECT1) --robot=ur5 --planner=RRT --problem=bookshelf_small --index=1; then \
		echo "UR5 RRT test completed successfully"; \
	else \
		echo "UR5 RRT test failed - check dependencies and robot files"; \
	fi

run-ur5-rrtconnect: $(PROJECT1)
	@echo "Running UR5 with RRTConnect planner..."
	@if ./$(PROJECT1) --robot=ur5 --planner=RRTConnect --problem=bookshelf_small --index=1; then \
		echo "UR5 RRTConnect test completed successfully"; \
	else \
		echo "UR5 RRTConnect test failed - check dependencies and robot files"; \
	fi

run-benchmark: $(BENCHMARKING)
	@echo "Running benchmark (this may take a while)..."
	@if ./$(BENCHMARKING) --problem=bookshelf_small --robot=ur5; then \
		echo "Benchmark completed successfully"; \
	else \
		echo "Benchmark failed - check dependencies and robot files"; \
	fi

# Clean targets
clean:
	rm -rf $(OBJDIR)
	rm -f $(RIGID_BODY) $(RIGID_BODY_CONTROLS) $(PROJECT1) $(BENCHMARKING)
	rm -f *.log *.db
	rm -rf /tmp/project1_collision_check/

# Install dependencies (Ubuntu/Debian)
install-deps:
	@echo "Installing dependencies..."
	@if [ "$$(id -u)" = "0" ]; then \
		apt-get update && \
		apt-get install -y libompl-dev libboost-all-dev libyaml-cpp-dev python3-pip && \
		pip3 install pybullet fire pandas pyyaml xmltodict; \
	else \
		sudo apt-get update && \
		sudo apt-get install -y libompl-dev libboost-all-dev libyaml-cpp-dev python3-pip && \
		pip3 install pybullet fire pandas pyyaml xmltodict; \
	fi
	@echo "Dependencies installed!"

# Install dependencies for Docker environment (as root)
install-deps-docker:
	@echo "Checking Docker environment..."
	@echo "OMPL should already be installed in /usr/local from Dockerfile"
	@if [ ! -f "/usr/local/include/ompl-1.7/ompl/config.h" ] && [ ! -f "/usr/local/include/ompl/config.h" ]; then \
		echo "OMPL not found, please rebuild Docker image"; \
		exit 1; \
	else \
		echo "OMPL found"; \
	fi
	@echo "Installing Python dependencies..."
	@pip3 install pybullet fire pandas pyyaml xmltodict
	@echo "All dependencies should be available in Docker environment!"

# Check if we're in Docker and OMPL is available
check-docker:
	@echo "Checking Docker environment setup..."
	@echo "OMPL headers: $$(ls -la /usr/local/include/ompl* 2>/dev/null | head -3 || echo 'Not found')"
	@echo "OMPL library: $$(ls -la /usr/local/lib/libompl.* 2>/dev/null || echo 'Not found')"
	@echo "Boost libraries: $$(ls /usr/lib/x86_64-linux-gnu/libboost_system* 2>/dev/null || echo 'Not found')"
	@echo "Python modules:"
	@python3 -c "import pybullet; print('  pybullet: OK')" 2>/dev/null || echo "  pybullet: MISSING"
	@python3 -c "import yaml; print('  yaml: OK')" 2>/dev/null || echo "  yaml: MISSING"

# Help target
help:
	@echo "Available targets:"
	@echo "  all                    - Build all executables"
	@echo "  rigid_body_planning    - Build rigid body planning demo"
	@echo "  rigid_body_planning_controls - Build rigid body planning with controls demo"
	@echo "  project1               - Build main project1 executable"
	@echo "  benchmarking           - Build benchmarking executable"
	@echo ""
	@echo "Test targets:"
	@echo "  test-rigid-body        - Test rigid body planning"
	@echo "  test-rigid-body-controls - Test rigid body planning with controls"
	@echo "  test-project1          - Test project1 (show help)"
	@echo ""
	@echo "Run examples:"
	@echo "  run-ur5-rrt            - Run UR5 with RRT planner"
	@echo "  run-ur5-rrtconnect     - Run UR5 with RRTConnect planner"
	@echo "  run-benchmark          - Run benchmarking"
	@echo ""
	@echo "Utility targets:"
	@echo "  clean                  - Remove build files"
	@echo "  install-deps           - Install system dependencies"
	@echo "  install-deps-docker    - Check Docker environment"
	@echo "  check-docker           - Verify Docker setup"
	@echo "  setup-files            - Setup robot files from 2024 version"
	@echo "  check-deps             - Check if dependencies are available"
	@echo "  help                   - Show this help"

.PHONY: all clean test-rigid-body test-rigid-body-controls test-project1 run-ur5-rrt run-ur5-rrtconnect run-benchmark install-deps help check-deps check-docker 