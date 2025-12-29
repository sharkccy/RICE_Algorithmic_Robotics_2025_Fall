# Project 1 - C++ Version

## Overview

This is the C++ version of Project 1 for COMP 450 Motion Planning. The project demonstrates motion planning using the Open Motion Planning Library (OMPL) with C++ instead of Python bindings.

## Project Structure

```
code_2025/
├── include/                    # Header files
│   ├── ManipulatorPlanner.h   # Main planner class
│   └── PyBulletInterface.h    # PyBullet collision interface
├── src/                       # Source files
│   ├── project1.cpp           # Main manipulator planning program
│   ├── benchmarking.cpp       # Benchmarking program
│   ├── ManipulatorPlanner.cpp # Planner implementation
│   ├── PyBulletInterface.cpp  # Collision interface implementation
│   ├── RigidBodyPlanning.cpp  # Rigid body planning demo
│   └── RigidBodyPlanningWithControls.cpp # Controls demo
├── collision_server.py        # Persistent Python collision server
├── ompllet/                   # PyBullet interface module (copied from project0)
├── ur5/                       # UR5 robot files (copied from 2024 version)
├── fetch/                     # Fetch robot files (copied from 2024 version)
├── Makefile                   # Build system
├── CMakeLists.txt            # Alternative CMake build
└── README.md                 # This file
```


## Building

### Using Makefile (Recommended)
```bash
# Build all executables
make install-deps-docker
make all

# Or build individual components
make install-deps-docker
make project1
make benchmarking
make rigid_body_planning
make rigid_body_planning_controls
```

## Usage

### 1. Rigid Body Planning Examples

#### Simple Rigid Body Planning
```bash
./rigid_body_planning
```

#### Rigid Body Planning with Controls
```bash
./rigid_body_planning_controls
```

### 2. Manipulator Planning

#### Basic Usage
```bash
# Show help and available options
./project1 --help

# Run with default settings (UR5, RRTConnect, bookshelf_small)
./project1

# Run specific configuration
./project1 --robot=ur5 --planner=RRT --problem=bookshelf_small --index=1 --planner_range=0.5

#Run Specific Problems
# problems with RRTConnect
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_small --index=1
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_tall --index=1
./project1 --robot=ur5 --planner=RRTConnect --problem=table_pick --index=1
./project1 --robot=ur5 --planner=RRTConnect --problem=box --index=1
./project1 --robot=ur5 --planner=RRTConnect --problem=table_under_pick --index=2

# Test different planners on problems
./project1 --robot=ur5 --planner=PRM --problem=bookshelf_small --index=1 --max_nn=50
./project1 --robot=ur5 --planner=KPIECE --problem=table_pick --index=1 --planner_range=0.5

# For challenging problems, try different indices
./project1 --robot=ur5 --planner=RRTConnect --problem=cage --index=2
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_thin --index=2
```

#### Available Options
- `--robot`: Robot type (`ur5` or `fetch`)
- `--planner`: Planner type (`PRM`, `RRT`, `RRTConnect`, `KPIECE`)
- `--problem`: Problem scene (e.g., `bookshelf_small`, `table_pick`, etc.)
- `--index`: Problem instance index (1-100)
- `--planner_range`: Range parameter for tree-based planners
- `--goal_bias`: Goal bias parameter for RRT and KPIECE
- `--max_nn`: Max nearest neighbors for PRM
- `--planning_time`: Time limit in seconds

#### Example Commands
```bash
# UR5 with different planners
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_small --index=1
./project1 --robot=ur5 --planner=RRT --problem=table_pick --index=7 --planner_range=0.5 --goal_bias=0.01
./project1 --robot=ur5 --planner=PRM --problem=cage --index=3 --max_nn=50

# Fetch robot examples
./project1 --robot=fetch --planner=RRTConnect --problem=table_pick --index=1
```

### 3. Benchmarking

#### Run Benchmarking
```bash
# Benchmark on bookshelf_small with UR5
./benchmarking --problem=bookshelf_small --robot=ur5

# Benchmark with custom parameters
./benchmarking --problem=table_pick --robot=fetch --runtime_limit=60 --run_count=100
```

#### Process Results
```bash
# Convert log files to database (uses Python script)
python3 ompl_benchmark_statistics.py *.log

# Upload the generated .db file to http://plannerarena.org/ for visualization
```

## Path Output

The C++ version automatically saves solution paths to `solution_path.txt` in the current directory after successful planning.

## Solution Visualization

After running the planner and generating a solution, you can create MP4 videos of the robot moving through the environment using the provided visualization script.

### Basic Video Generation
```bash
# First, run the planner to generate a solution
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_small --index=1

# Then generate an MP4 video of the solution
python3 visualize_solution.py --robot=ur5 --problem=bookshelf_small --index=1
```

### Video Generation Options
```bash
# Custom output filename and frame rate
python3 visualize_solution.py --robot=ur5 --problem=table_pick --index=1 --output=table_pick_solution.mp4 --fps=15

# High-resolution video (1080p at 30 FPS)
python3 visualize_solution.py --robot=ur5 --problem=box --index=1 --width=1920 --height=1080 --fps=30

# Fast playback (higher frame rate)
python3 visualize_solution.py --robot=ur5 --problem=cage --index=2 --fps=30

# Generate video showing only start and goal configurations
python3 visualize_solution.py --robot=ur5 --problem=bookshelf_tall --index=1 --start-goal-only

# Use custom solution file
python3 visualize_solution.py --robot=ur5 --problem=table_pick --index=1 --path=my_custom_path.txt --output=custom_video.mp4
```

### Complete Workflow Example
```bash
# 1. Run the planner
./project1 --robot=ur5 --planner=RRTConnect --problem=bookshelf_small --index=1

# 2. Generate video (default: trajectory_video.mp4)
python3 visualize_solution.py --robot=ur5 --problem=bookshelf_small --index=1

# 3. Try a different problem with custom video settings
./project1 --robot=ur5 --planner=PRM --problem=table_pick --index=1 --max_nn=50
python3 visualize_solution.py --robot=ur5 --problem=table_pick --index=1 --output=table_pick_prm.mp4 --fps=20
```

### Video Features
- **Automatic Environment Loading**: Loads the exact same environment and problem instance used for planning
- **Robot Animation**: Shows the robot moving through each waypoint in the solution path
- **Multiple Formats**: Generates standard MP4 videos viewable on any device
- **Customizable Settings**: Adjustable resolution, frame rate, and output filename
- **Progress Tracking**: Shows progress during video generation
- **Start/Goal Display**: Option to show only start and goal configurations

**Note**: The video generator runs in headless mode (no GUI required) and creates MP4 files that can be viewed on any device. Default output is `trajectory_video.mp4` in the current directory.

## Exercise Instructions

### Exercise 1.1: Rigid Body Planning
1. Run both rigid body planning demos:
   ```bash
   make test-rigid-body
   make test-rigid-body-controls
   ```
2. Compare the two approaches and analyze the differences in setup and performance.

### Exercise 1.2: Manipulator Planning
1. Test each of the seven problem scenes with at least one instance:
   - `bookshelf_tall`
   - `bookshelf_thin` 
   - `table_under_pick`
   - `bookshelf_small`
   - `box`
   - `table_pick`
   - `cage`

2. Example commands:
   ```bash
   ./project1 --problem=bookshelf_tall --index=1
   ./project1 --problem=table_pick --index=5
   ./project1 --problem=cage --index=2
   # ... etc for each scene
   ```

3. Try different planners and observe the differences:
   ```bash
   ./project1 --problem=bookshelf_small --planner=RRT --index=1
   ./project1 --problem=bookshelf_small --planner=RRTConnect --index=1
   ./project1 --problem=bookshelf_small --planner=PRM --index=1
   ./project1 --problem=bookshelf_small --planner=KPIECE --index=1
   ```

### Exercise 2: Benchmarking
1. Run benchmarks on the required problems:
   ```bash
   # UR5 bookshelf_small
   ./benchmarking --problem=bookshelf_small --robot=ur5
   
   # Fetch table_pick  
   ./benchmarking --problem=table_pick --robot=fetch
   ```

2. Process results and upload to Planner Arena:
   ```bash
   python3 ompl_benchmark_statistics.py *.log
   # Upload the .db file to http://plannerarena.org/
   ```

## Makefile Targets

### Build Targets
- `make all` - Build all executables
- `make project1` - Build main project executable
- `make benchmarking` - Build benchmarking executable
- `make rigid_body_planning` - Build rigid body demo
- `make rigid_body_planning_controls` - Build controls demo

### Test Targets
- `make test-rigid-body` - Test rigid body planning
- `make test-rigid-body-controls` - Test controls planning
- `make test-project1` - Show project1 help

### Example Runs
- `make run-ur5-rrt` - Run UR5 with RRT
- `make run-ur5-rrtconnect` - Run UR5 with RRTConnect
- `make run-benchmark` - Run benchmarking

### Utility Targets
- `make clean` - Remove build files
- `make install-deps` - Install system dependencies
- `make check-deps` - Check if dependencies are available
- `make help` - Show available targets

## Technical Details

### Collision Checking Architecture
The C++ version uses a persistent Python process (`collision_server.py`) that maintains a PyBullet simulator instance. This provides:
- **Fast collision checking**: No process startup overhead
- **Direct PyBullet access**: Full PyBullet functionality
- **Robust error handling**: Graceful fallbacks

