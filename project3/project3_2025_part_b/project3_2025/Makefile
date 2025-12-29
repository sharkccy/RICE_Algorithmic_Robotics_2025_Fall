# COMP/ELEC/MECH 450/550 - Project 3 (2025) Makefile
# Enhanced Manipulator Planning with Analysis & Visualization

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INCLUDES = -I/usr/local/include/ompl-1.7 -I/usr/include/eigen3 -Isrc -Iprovided -I/usr/include/python3.10
LIBS = -L/usr/local/lib -lompl -lboost_system -lboost_filesystem -lpython3.10

# Source files
EXERCISE2_SOURCES = src/Project3Exercise2.cpp src/RTP.cpp src/CollisionChecking.cpp
EXERCISE3_SOURCES = src/Project3Exercise3.cpp src/RTP.cpp src/CollisionChecking.cpp provided/ManipulatorPlanner.cpp provided/PyBulletInterface.cpp provided/PathAnalyzer.cpp

# Object files
EXERCISE2_OBJECTS = $(EXERCISE2_SOURCES:.cpp=.o)
EXERCISE3_OBJECTS = $(EXERCISE3_SOURCES:.cpp=.o)

# Targets
TARGETS = Project3Exercise2 Project3Exercise3

.PHONY: all setup clean verify help start-collision-server stop-collision-server run-exercise3 generate-video generate-video-hd

all: setup $(TARGETS)

setup:
	@echo "Installing Python dependencies..."
	@python3 -c "import xmltodict" 2>/dev/null || pip3 install xmltodict
	@echo "Python dependencies installed successfully"

# Exercise 2: 2D Planning
Project3Exercise2: $(EXERCISE2_OBJECTS)
	@echo "Building Project3Exercise2..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)
	@echo "Build complete: Project3Exercise2"

# Exercise 3: Enhanced Manipulator Planning with Analysis
Project3Exercise3: $(EXERCISE3_OBJECTS)
	@echo "Building Project3Exercise3 (Enhanced Manipulator Planning with Analysis)..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)
	@echo "Build complete: Project3Exercise3"

# Compile source files
%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $@

# Collision server management
start-collision-server:
	@echo "Starting PyBullet collision server..."
	@python3 provided/collision_server.py &
	@echo "Collision server started in background"

stop-collision-server:
	@echo "Stopping collision server..."
	@pkill -f collision_server.py || true
	@echo "Collision server stopped"

# Run Exercise 3 with collision server
run-exercise3: Project3Exercise3
	@echo "Starting collision server..."
	python3 provided/collision_server.py &
	@echo "Running Project3Exercise3..."
	sleep 2  # Give collision server time to start
	./Project3Exercise3
	$(MAKE) stop-collision-server

# Video generation targets
generate-video:
	@echo "Generating trajectory videos..."
	@if [ -f "RTP_path.txt" ]; then \
		echo "Generating RTP trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=RTP_path.txt --output=RTP_trajectory.mp4; \
	fi
	@if [ -f "RRT_path.txt" ]; then \
		echo "Generating RRT trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=RRT_path.txt --output=RRT_trajectory.mp4; \
	fi
	@if [ -f "RRT_Connect_path.txt" ]; then \
		echo "Generating RRT-Connect trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=RRT_Connect_path.txt --output=RRT_Connect_trajectory.mp4; \
	fi
	@if [ -f "PRM_path.txt" ]; then \
		echo "Generating PRM trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=PRM_path.txt --output=PRM_trajectory.mp4; \
	fi
	@if [ -f "EST_path.txt" ]; then \
		echo "Generating EST trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=EST_path.txt --output=EST_trajectory.mp4; \
	fi
	@echo "Video generation complete! Check for *.mp4 files."

generate-video-hd:
	@echo "Generating high-definition trajectory videos..."
	@if [ -f "RTP_path.txt" ]; then \
		echo "Generating RTP HD trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=RTP_path.txt --output=RTP_trajectory_HD.mp4 --width=1920 --height=1080 --fps=30; \
	fi
	@if [ -f "RRT_path.txt" ]; then \
		echo "Generating RRT HD trajectory video..."; \
		python3 provided/visualize_trajectory.py --path=RRT_path.txt --output=RRT_trajectory_HD.mp4 --width=1920 --height=1080 --fps=30; \
	fi

# Verify OMPL installation
verify:
	@echo "Verifying OMPL installation..."
	@echo '#include <ompl/config.h>' > test_ompl.cpp
	@echo '#include <iostream>' >> test_ompl.cpp
	@echo 'int main() { std::cout << "OMPL version: " << OMPL_VERSION << std::endl; return 0; }' >> test_ompl.cpp
	@$(CXX) $(CXXFLAGS) $(INCLUDES) test_ompl.cpp -o test_ompl $(LIBS) && ./test_ompl && rm -f test_ompl test_ompl.cpp
	@echo "OMPL verification complete!"

# Clean build files
clean:
	@echo "Cleaning..."
	rm -f src/*.o provided/*.o $(TARGETS) *_path.txt *.mp4

help:
	@echo "Available commands:"
	@echo "  make                     - Install dependencies and build all exercises"
	@echo "  make setup               - Install required Python dependencies (xmltodict)"
	@echo "  make Project3Exercise2   - Build Exercise 2 (2D Planning)"
	@echo "  make Project3Exercise3   - Build Exercise 3 (Enhanced Manipulator Planning)"
	@echo "  make run-exercise3       - Run Exercise 3 with collision server"
	@echo "  make generate-video      - Generate MP4 videos for all trajectory files"
	@echo "  make generate-video-hd   - Generate HD videos for selected trajectories"
	@echo "  make start-collision-server - Start PyBullet collision server"
	@echo "  make stop-collision-server  - Stop PyBullet collision server"
	@echo "  make verify              - Check OMPL installation"
	@echo "  make clean               - Remove build files and generated videos"
	@echo "  make help                - Show this help"
	@echo ""
	@echo "Quick Start:"
	@echo "  1. Implement RTP in src/RTP.h and src/RTP.cpp"
	@echo "  2. Complete Exercise 2 in src/Project3Exercise2.cpp"
	@echo "  3. Complete Exercise 3 in src/Project3Exercise3.cpp (optional modifications)"
	@echo "  4. Build and run: make run-exercise3"
	@echo "  5. Generate videos: make generate-video"
	@echo ""
	@echo "Note: All provided files are in the 'provided/' directory" 