#!/usr/bin/env python3
"""
Persistent collision server for project1

This server maintains a PyBullet simulator instance and responds to commands
from the C++ code via stdin/stdout pipes.
"""

import sys
import os
import pickle
from pathlib import Path

# Add the ompllet module to the path - try multiple possible locations
possible_paths = [
    # Local ompllet module in current directory (preferred)
    ".",
    # Current script directory (where this collision_server.py is located)
    str(Path(__file__).parent),
    # Docker workspace paths for project1/code_2025
    "/workspace/project1/code_2025",
    "/workspace/Projects/project1/code_2025", 
    # Docker workspace paths for project0/code_2024 (fallback)
    "/workspace/project0/code_2024",
    "/workspace/Projects/project0/code_2024",
    # Local development environment
    str(Path(__file__).parent.parent.parent / "project0" / "code_2024"),
    # Alternative Docker paths
    "../../Projects/project0/code_2024",
    "../../../Projects/project0/code_2024",
    # Current directory relative paths
    str(Path(__file__).parent.parent.parent.parent / "Projects" / "project0" / "code_2024"),
]

OMPLLET_AVAILABLE = False

for path in possible_paths:
    ompllet_path = os.path.join(path, "ompllet")
    if os.path.exists(ompllet_path):
        sys.path.insert(0, path)
        try:
            import ompllet.pybullet_interface as opi
            import ompllet.constants as oci
            OMPLLET_AVAILABLE = True
            break
        except ImportError as e:
            # Continue trying other paths
            continue

if not OMPLLET_AVAILABLE:
    print("ERROR: ompllet module not available", flush=True)
    sys.exit(1)

class CollisionServer:
    def __init__(self):
        self.simulator = None
        self.robot_type = None
        self.problems_data = None
        
    def load_problems_data(self, robot_type):
        """Load problems data from pickle file"""
        try:
            problems_file = f"{robot_type}/problems.pkl"
            if os.path.exists(problems_file):
                with open(problems_file, 'rb') as f:
                    self.problems_data = pickle.load(f)
                return True
            return False
        except Exception as e:
            print(f"Failed to load problems data: {e}", flush=True)
            return False
    
    def find_urdf_file(self, urdf_path):
        """Find the actual URDF file, trying various locations"""
        # Try the path as-is first
        if os.path.exists(urdf_path):
            return urdf_path
        
        # Try relative to the collision server script location
        script_dir = Path(__file__).parent
        relative_path = script_dir / urdf_path
        if relative_path.exists():
            return str(relative_path)
        
        # Try in the parent directories
        for parent_level in range(1, 4):
            parent_dir = script_dir
            for _ in range(parent_level):
                parent_dir = parent_dir.parent
            test_path = parent_dir / urdf_path
            if test_path.exists():
                return str(test_path)
        
        # Try in Docker workspace
        workspace_paths = [
            f"/workspace/project1/code_2025/{urdf_path}",
            f"/workspace/Projects/project1/code_2025/{urdf_path}",
            f"/workspace/{urdf_path}",
        ]
        
        for workspace_path in workspace_paths:
            if os.path.exists(workspace_path):
                return workspace_path
        
        return None
    
    def handle_init(self, args):
        """Initialize the PyBullet simulator"""
        try:
            if len(args) < 3:
                return "ERROR: INIT requires urdf_path gui joint_names..."
            
            urdf_path = args[1]
            gui = args[2].lower() == 'true'
            joint_names = args[3:]
            
            # Find the actual URDF file
            actual_urdf_path = self.find_urdf_file(urdf_path)
            if not actual_urdf_path:
                return f"ERROR: URDF file not found: {urdf_path}"
            
            # Determine robot type
            if 'ur5' in urdf_path.lower():
                self.robot_type = 'ur5'
            elif 'fetch' in urdf_path.lower():
                self.robot_type = 'fetch'
            else:
                return f"ERROR: Unknown robot type from URDF: {urdf_path}"
            
            if self.robot_type not in oci.ROBOT_JOINTS:
                return f"ERROR: Robot type {self.robot_type} not supported"
            
            # Use the joint names from constants if provided names don't match
            expected_joints = oci.ROBOT_JOINTS[self.robot_type]
            if len(joint_names) != len(expected_joints):
                joint_names = expected_joints
            
            # Create PyBullet simulator (headless for collision checking)
            self.simulator = opi.PyBulletSimulator(actual_urdf_path, joint_names, visualize=gui)
            
            # Load problems data for this robot type
            if not self.load_problems_data(self.robot_type):
                print(f"Warning: Could not load problems data for {self.robot_type}", flush=True)
            
            return "OK"
            
        except Exception as e:
            return f"ERROR: {str(e)}"
    
    def handle_load_problem(self, args):
        """Load specific problem from problems.pkl"""
        try:
            if len(args) < 3:
                return "ERROR: LOAD_PROBLEM requires problem_name index"
            
            if not self.simulator:
                return "ERROR: Simulator not initialized"
            
            if not self.problems_data:
                return "ERROR: Problems data not loaded"
            
            problem_name = args[1]
            index = int(args[2])
            
            # Check if problem exists
            if problem_name not in self.problems_data['problems']:
                available = list(self.problems_data['problems'].keys())
                return f"ERROR: Problem '{problem_name}' not found. Available: {available}"
            
            # Find the specific problem instance
            problems = self.problems_data['problems'][problem_name]
            problem_data = None
            for p in problems:
                if p['index'] == index:
                    problem_data = p
                    break
            
            if not problem_data:
                indices = [p['index'] for p in problems]
                return f"ERROR: Problem '{problem_name}' index {index} not found. Available indices: {min(indices)}-{max(indices)}"
            
            # Load the environment
            self.simulator.add_environment_from_problem_dict(problem_data, False)
            
            # Return the start and goal configurations
            start = problem_data['start']
            goals = problem_data['goals']
            
            # Format response: START start_config GOALS goal1 goal2 ...
            response = "START"
            for val in start:
                response += f" {val}"
            response += " GOALS"
            for goal in goals:
                for val in goal:
                    response += f" {val}"
                response += " GOAL_END"
            
            return response
            
        except Exception as e:
            return f"ERROR: {str(e)}"
    
    def handle_load_env(self, args):
        """Load environment from problem file"""
        try:
            if len(args) < 2:
                return "ERROR: LOAD_ENV requires problem_file"
            
            if not self.simulator:
                return "ERROR: Simulator not initialized"
            
            problem_file = args[1]
            
            # Try to find the problem file
            problem_file_path = Path(problem_file)
            if not problem_file_path.exists():
                script_dir = Path(__file__).parent
                for parent_level in range(0, 4):
                    parent_dir = script_dir
                    for _ in range(parent_level):
                        parent_dir = parent_dir.parent
                    test_path = parent_dir / problem_file
                    if test_path.exists():
                        problem_file_path = test_path
                        break
            
            if problem_file_path.exists():
                import yaml
                with open(problem_file_path, 'r') as f:
                    problem_data = yaml.safe_load(f)
                self.simulator.add_environment_from_problem_dict(problem_data, False)
                return "OK"
            else:
                return f"WARNING: Problem file not found: {problem_file}"
                
        except Exception as e:
            return f"ERROR: {str(e)}"
    
    def handle_set_joints(self, args):
        """Set joint positions"""
        try:
            if len(args) < 2:
                return "ERROR: SET_JOINTS requires joint positions"
            
            if not self.simulator:
                return "ERROR: Simulator not initialized"
            
            positions = [float(x) for x in args[1:]]
            self.simulator.set_joint_positions(positions)
            
            return "OK"
            
        except Exception as e:
            return f"ERROR: {str(e)}"
    
    def handle_check_collision(self, args):
        """Check for collision"""
        try:
            if not self.simulator:
                return "ERROR: Simulator not initialized"
            
            in_collision = self.simulator.in_collision()
            return "true" if in_collision else "false"
            
        except Exception as e:
            return f"ERROR: {str(e)}"
    
    def run(self):
        """Main server loop"""
        try:
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                
                line = line.strip()
                if not line:
                    continue
                
                args = line.split()
                command = args[0]
                
                if command == "INIT":
                    response = self.handle_init(args)
                elif command == "LOAD_PROBLEM":
                    response = self.handle_load_problem(args)
                elif command == "LOAD_ENV":
                    response = self.handle_load_env(args)
                elif command == "SET_JOINTS":
                    response = self.handle_set_joints(args)
                elif command == "CHECK_COLLISION":
                    response = self.handle_check_collision(args)
                else:
                    response = f"ERROR: Unknown command: {command}"
                
                print(response, flush=True)
                
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"ERROR: Server error: {str(e)}", flush=True)

if __name__ == "__main__":
    server = CollisionServer()
    server.run() 