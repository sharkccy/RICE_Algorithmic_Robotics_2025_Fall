cmake_minimum_required(VERSION 3.10)
project(Project1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(include)

# Add executable for rigid body planning
add_executable(rigid_body_planning src/RigidBodyPlanning.cpp)
target_link_libraries(rigid_body_planning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

# Add executable for rigid body planning with controls
add_executable(rigid_body_planning_controls src/RigidBodyPlanningWithControls.cpp)
target_link_libraries(rigid_body_planning_controls ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

# Add executable for main project1 manipulator planning
add_executable(project1 src/project1.cpp src/ManipulatorPlanner.cpp src/PyBulletInterface.cpp)
target_link_libraries(project1 ${OMPL_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

# Add executable for benchmarking
add_executable(benchmarking src/benchmarking.cpp src/ManipulatorPlanner.cpp src/PyBulletInterface.cpp)
target_link_libraries(benchmarking ${OMPL_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall") 