#!/usr/bin/env python3
"""
COMP 450 Project 1 - Solution Path Visualizer

Usage:
    python3 visualize_solution.py --robot=ur5 --problem=bookshelf_small --index=1
    python3 visualize_solution.py --robot=ur5 --problem=table_pick --index=1 --path=custom_path.txt --output=my_video.mp4
    python3 visualize_solution.py --robot=fetch --problem=table_pick --index=1 --fps=15
"""

import sys
import os
import argparse
import pickle
import time
import numpy as np
from pathlib import Path

# Add the ompllet module to the path
sys.path.insert(0, ".")

try:
    import ompllet.pybullet_interface as opi
    import ompllet.constants as oci
    import pybullet as p
    import cv2
except ImportError as e:
    print(f"Error: Required module not found: {e}")
    print("Make sure you're running from the code_2025 directory and have OpenCV installed.")
    sys.exit(1)

class SolutionVideoGenerator:
    """Generates MP4 videos of solution paths with robot and environment"""
    
    def __init__(self, robot_type, output_file="trajectory_video.mp4", fps=10, width=1280, height=720):
        self.robot_type = robot_type
        self.output_file = output_file
        self.fps = fps
        self.width = width
        self.height = height
        self.simulator = None
        self.problems_data = None
        self.video_writer = None
        
    def load_problems_data(self):
        """Load problems data from pickle file"""
        try:
            problems_file = f"{self.robot_type}/problems.pkl"
            if os.path.exists(problems_file):
                with open(problems_file, 'rb') as f:
                    self.problems_data = pickle.load(f)
                return True
            else:
                print(f"Error: Problems file not found: {problems_file}")
                return False
        except Exception as e:
            print(f"Error loading problems data: {e}")
            return False
    
    def setup_simulator(self, problem_name, index):
        """Set up the PyBullet simulator with robot and environment"""
        try:
            # Load problems data
            if not self.load_problems_data():
                return False
            
            # Find the specific problem
            if problem_name not in self.problems_data['problems']:
                available = list(self.problems_data['problems'].keys())
                print(f"Error: Problem '{problem_name}' not found. Available: {available}")
                return False
            
            problems = self.problems_data['problems'][problem_name]
            problem_data = None
            for p in problems:
                if p['index'] == index:
                    problem_data = p
                    break
            
            if not problem_data:
                indices = [p['index'] for p in problems]
                print(f"Error: Problem '{problem_name}' index {index} not found. Available indices: {min(indices)}-{max(indices)}")
                return False
            
            # Create simulator in headless mode for video generation
            urdf_path = f"{self.robot_type}/{self.robot_type}_spherized.urdf"
            joint_names = oci.ROBOT_JOINTS[self.robot_type]
            
            print(f"Setting up {self.robot_type} robot with {problem_name} environment...")
            self.simulator = opi.PyBulletSimulator(urdf_path, joint_names, visualize=False)  # Headless mode
            
            # Load environment
            self.simulator.add_environment_from_problem_dict(problem_data, False)
            
            # Add ground plane for better visualization
            self.add_ground_plane()
            
            # Store problem data for reference
            self.problem_data = problem_data
            
            # Get environment bounds for better camera positioning
            self.setup_environment_info()
            
            print("✓ Simulator setup complete!")
            return True
            
        except Exception as e:
            print(f"Error setting up simulator: {e}")
            return False
    
    def add_ground_plane(self):
        """Add a ground plane for better visualization"""
        try:
            # Add a large ground plane
            p.loadURDF("plane.urdf", [0, 0, -0.1])
            print(" Ground plane added")
        except Exception as e:
            print(f"Warning: Could not add ground plane: {e}")
            # Create a simple box as ground plane
            try:
                ground_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05])
                ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05], rgbaColor=[0.7, 0.7, 0.7, 1])
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_shape, 
                                baseVisualShapeIndex=ground_visual, basePosition=[0, 0, -0.1])
                print(" Custom ground plane added")
            except Exception as e2:
                print(f"Warning: Could not create custom ground plane: {e2}")
    
    def setup_environment_info(self):
        """Get information about the environment for better camera positioning"""
        try:
            # Get all object IDs in the simulation
            num_bodies = p.getNumBodies()
            
            # Find bounds of all objects
            min_bounds = [float('inf')] * 3
            max_bounds = [float('-inf')] * 3
            
            for i in range(num_bodies):
                aabb_min, aabb_max = p.getAABB(i)
                for j in range(3):
                    min_bounds[j] = min(min_bounds[j], aabb_min[j])
                    max_bounds[j] = max(max_bounds[j], aabb_max[j])
            
            # Calculate center and size of environment
            self.env_center = [(min_bounds[i] + max_bounds[i]) / 2 for i in range(3)]
            self.env_size = [max_bounds[i] - min_bounds[i] for i in range(3)]
            
            print(f"Environment center: {[f'{x:.2f}' for x in self.env_center]}")
            print(f"Environment size: {[f'{x:.2f}' for x in self.env_size]}")
            
        except Exception as e:
            print(f"Warning: Could not get environment info: {e}")
            # Use default values
            self.env_center = [0, 0, 0.5]
            self.env_size = [2, 2, 2]
    
    def load_solution_path(self, path_file="solution_path.txt"):
        """Load solution path from file"""
        try:
            if not os.path.exists(path_file):
                print(f"Error: Solution path file not found: {path_file}")
                print("Make sure you've run ./project1 first to generate a solution.")
                return None
            
            path = []
            with open(path_file, 'r') as f:
                lines = f.readlines()
                num_waypoints = int(lines[0].strip())
                
                for i in range(1, num_waypoints + 1):
                    if i < len(lines):
                        config = [float(x) for x in lines[i].strip().split()]
                        path.append(config)
            
            print(f" Loaded solution path with {len(path)} waypoints")
            return path
            
        except Exception as e:
            print(f"Error loading solution path: {e}")
            return None
    
    def setup_camera(self):
        """Setup camera for optimal video recording"""
        # Use environment information for better camera positioning
        if hasattr(self, 'env_center') and hasattr(self, 'env_size'):
            camera_target = self.env_center
            # Calculate appropriate distance based on environment size
            max_size = max(self.env_size)
            camera_distance = max(3.0, max_size * 1.5)  # Ensure we can see everything
        else:
            # Fallback values
            camera_target = [0, 0, 0.8]
            camera_distance = 3.5
        
        camera_yaw = 45
        camera_pitch = -25
        
        # Set up the camera view matrix
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_target,
            distance=camera_distance,
            yaw=camera_yaw,
            pitch=camera_pitch,
            roll=0,
            upAxisIndex=2
        )
        
        # Set up projection matrix
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=50,  # Slightly wider field of view
            aspect=self.width / self.height,
            nearVal=0.1,
            farVal=100.0
        )
        
        print(f"Camera setup: target={[f'{x:.2f}' for x in camera_target]}, distance={camera_distance:.2f}")
        
        return view_matrix, projection_matrix
    
    def capture_frame(self, view_matrix, projection_matrix):
        """Capture current scene as image frame"""
        # Render the scene with proper lighting
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            lightDirection=[1, 1, 1],  # Add proper lighting
            lightColor=[1, 1, 1],
            lightDistance=100,
            shadow=1,
            lightAmbientCoeff=0.6,  # Increased ambient light
            lightDiffuseCoeff=0.8,  # Increased diffuse light
            lightSpecularCoeff=0.3
        )
        
        # Convert to OpenCV format (BGR)
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array.reshape((height, width, 4))  # RGBA
        bgr_array = cv2.cvtColor(rgb_array[:, :, :3], cv2.COLOR_RGB2BGR)  # Convert to BGR, drop alpha
        
        return bgr_array
    
    def debug_environment(self):
        """Print debug information about the environment"""
        try:
            num_bodies = p.getNumBodies()
            print(f"Debug: Found {num_bodies} bodies in simulation:")
            
            for i in range(min(10, num_bodies)):  # Show first 10 bodies
                body_info = p.getBodyInfo(i)
                aabb_min, aabb_max = p.getAABB(i)
                print(f"  Body {i}: {body_info[0].decode('utf-8') if body_info[0] else 'unnamed'}")
                print(f"    AABB: min={[f'{x:.2f}' for x in aabb_min]}, max={[f'{x:.2f}' for x in aabb_max]}")
                
        except Exception as e:
            print(f"Debug error: {e}")
    
    def generate_video(self, path):
        """Generate MP4 video of the robot following the solution path"""
        if not self.simulator or not path:
            return False
        
        print("\n" + "="*60)
        print("GENERATING SOLUTION PATH VIDEO")
        print("="*60)
        print(f"Robot: {self.robot_type}")
        print(f"Path length: {len(path)} waypoints")
        print(f"Output file: {self.output_file}")
        print(f"Video settings: {self.width}x{self.height} @ {self.fps} FPS")
        print(f"Start config: {[f'{x:.3f}' for x in path[0]]}")
        print(f"Goal config:  {[f'{x:.3f}' for x in path[-1]]}")
        print()
        
        try:
            # Setup camera
            view_matrix, projection_matrix = self.setup_camera()
            
            # Debug environment to see what objects are loaded
            self.debug_environment()
            
            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.output_file,
                fourcc,
                self.fps,
                (self.width, self.height)
            )
            
            if not self.video_writer.isOpened():
                print("Error: Could not open video writer")
                return False
            
            print("Generating video frames...")
            
            # Save a test frame for debugging (optional)
            test_frame = self.capture_frame(view_matrix, projection_matrix)
            cv2.imwrite("test_frame.png", test_frame)
            print(" Test frame saved as test_frame.png")
            
            # Add some initial frames showing the start configuration
            self.simulator.set_joint_positions(path[0])
            for _ in range(self.fps):  # 1 second of start position
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
            
            # Generate frames for the path
            total_frames = len(path)
            for i, config in enumerate(path):
                # Set robot configuration
                self.simulator.set_joint_positions(config)
                
                # Capture frame
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
                
                # Show progress
                if i % 50 == 0 or i == len(path) - 1:
                    progress = (i + 1) / len(path) * 100
                    print(f"Progress: {progress:.1f}% ({i+1}/{len(path)} frames)")
            
            # Add some final frames showing the goal configuration
            self.simulator.set_joint_positions(path[-1])
            for _ in range(self.fps):  # 1 second of goal position
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
            
            # Finalize video
            self.video_writer.release()
            
            print(f"\n✓ Video generation complete!")
            print(f"✓ Video saved as: {self.output_file}")
            print(f"✓ Total frames: {total_frames + 2*self.fps}")
            print(f"✓ Video duration: {(total_frames + 2*self.fps) / self.fps:.1f} seconds")
            
            return True
            
        except Exception as e:
            print(f"Error during video generation: {e}")
            if self.video_writer:
                self.video_writer.release()
            return False
    
    def generate_start_goal_video(self, duration_per_config=2.0):
        """Generate a video showing start and goal configurations"""
        if not self.simulator or not hasattr(self, 'problem_data'):
            return False
        
        start = self.problem_data['start']
        goals = self.problem_data['goals']
        
        print(f"Generating start/goal video...")
        
        try:
            # Setup camera and video writer
            view_matrix, projection_matrix = self.setup_camera()
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.output_file,
                fourcc,
                self.fps,
                (self.width, self.height)
            )
            
            frames_per_config = int(duration_per_config * self.fps)
            
            # Show start configuration
            print("Adding start configuration frames...")
            self.simulator.set_joint_positions(start)
            for _ in range(frames_per_config):
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
            
            # Show each goal configuration
            for i, goal in enumerate(goals):
                print(f"Adding goal {i+1} configuration frames...")
                self.simulator.set_joint_positions(goal)
                for _ in range(frames_per_config):
                    frame = self.capture_frame(view_matrix, projection_matrix)
                    self.video_writer.write(frame)
            
            self.video_writer.release()
            
            total_duration = (1 + len(goals)) * duration_per_config
            print(f" Start/goal video saved as: {self.output_file}")
            print(f" Video duration: {total_duration:.1f} seconds")
            
            return True
            
        except Exception as e:
            print(f"Error generating start/goal video: {e}")
            if self.video_writer:
                self.video_writer.release()
            return False

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Generate MP4 videos of solution paths from COMP 450 Project 1",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate video for bookshelf_small problem
  python3 visualize_solution.py --robot=ur5 --problem=bookshelf_small --index=1
  
  # Generate video with custom output file and settings
  python3 visualize_solution.py --robot=ur5 --problem=table_pick --index=1 --output=table_pick_solution.mp4 --fps=15
  
  # Generate high-resolution video
  python3 visualize_solution.py --robot=ur5 --problem=box --index=1 --width=1920 --height=1080 --fps=30
  
  # Use custom path file
  python3 visualize_solution.py --robot=ur5 --problem=cage --index=2 --path=my_path.txt
  
  # Generate start/goal only video
  python3 visualize_solution.py --robot=ur5 --problem=bookshelf_tall --index=1 --start-goal-only
        """
    )
    
    parser.add_argument('--robot', required=True, choices=['ur5', 'fetch'],
                       help='Robot type (ur5 or fetch)')
    parser.add_argument('--problem', required=True,
                       help='Problem name (e.g., bookshelf_small, table_pick, cage)')
    parser.add_argument('--index', type=int, required=True,
                       help='Problem instance index (1-100)')
    parser.add_argument('--path', default='solution_path.txt',
                       help='Path to solution file (default: solution_path.txt)')
    parser.add_argument('--output', default='trajectory_video.mp4',
                       help='Output video file (default: trajectory_video.mp4)')
    parser.add_argument('--fps', type=int, default=10,
                       help='Video frame rate (default: 10 FPS)')
    parser.add_argument('--width', type=int, default=1280,
                       help='Video width in pixels (default: 1280)')
    parser.add_argument('--height', type=int, default=720,
                       help='Video height in pixels (default: 720)')
    parser.add_argument('--start-goal-only', action='store_true',
                       help='Generate video showing only start and goal configurations')
    
    return parser.parse_args()

def main():
    """Main function"""
    print("COMP 450 Project 1 - Solution Path Video Generator")
    print("=" * 60)
    
    args = parse_arguments()
    
    # Create output directory if needed
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Create video generator
    generator = SolutionVideoGenerator(
        robot_type=args.robot,
        output_file=args.output,
        fps=args.fps,
        width=args.width,
        height=args.height
    )
    
    # Setup simulator with problem environment
    if not generator.setup_simulator(args.problem, args.index):
        sys.exit(1)
    
    success = False
    
    if args.start_goal_only:
        # Generate start/goal only video
        success = generator.generate_start_goal_video()
    else:
        # Load and generate solution path video
        path = generator.load_solution_path(args.path)
        if path:
            success = generator.generate_video(path)
        else:
            print("Failed to load solution path. Generating start/goal video instead.")
            success = generator.generate_start_goal_video()
    
    if success:
        print(f"\n Success! Your video is ready: {args.output}")
        print("You can now view this video on any device!")
    else:
        print("\n Video generation failed.")
        sys.exit(1)

if __name__ == "__main__":
    main() 