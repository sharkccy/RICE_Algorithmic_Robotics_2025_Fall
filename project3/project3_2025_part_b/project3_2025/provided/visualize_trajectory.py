#!/usr/bin/env python3
"""
COMP 450 Project 3 - Trajectory Video Generator

Generates MP4 videos of UR5 robot following planned trajectories in custom environments.

Usage:
    python3 visualize_trajectory.py --path=RTP_path.txt --output=rtp_trajectory.mp4
    python3 visualize_trajectory.py --path=RRT_path.txt --output=rrt_trajectory.mp4
    python3 visualize_trajectory.py --path=PRM_path.txt --output=prm_trajectory.mp4
"""

import sys
import os
import argparse
import time
import numpy as np
from pathlib import Path
import signal

try:
    import pybullet as p
    import cv2
except ImportError as e:
    print(f"Error: Required module not found: {e}")
    print("Make sure you have PyBullet and OpenCV installed:")
    print("  pip3 install pybullet opencv-python")
    sys.exit(1)

class TimeoutError(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutError("Operation timed out")

class TrajectoryVideoGenerator:
    """Generates MP4 videos of UR5 trajectories in custom environments"""
    
    def __init__(self, output_file="trajectory_video.mp4", fps=12, width=960, height=540):
        self.output_file = output_file
        self.fps = fps
        self.width = width
        self.height = height
        self.physics_client = None
        self.robot_id = None
        self.video_writer = None
        
    def setup_simulator(self):
        """Set up PyBullet simulator with UR5 and custom environment"""
        try:
            print("Setting up PyBullet simulator...")
            
            # Connect to PyBullet in DIRECT mode for video generation
            self.physics_client = p.connect(p.DIRECT)
            
            # Suppress PyBullet warnings and info messages
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setPhysicsEngineParameter(enableFileCaching=0)
            
            # Set gravity
            p.setGravity(0, 0, -9.81)
            
            # Create ground plane manually
            ground_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05])
            ground_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05], 
                                                rgbaColor=[0.7, 0.7, 0.7, 1])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_shape,
                            baseVisualShapeIndex=ground_visual, basePosition=[0, 0, -0.05])
            
            # Load UR5 robot
            # Check for URDF - USE SAME URDF AS PLANNER
            urdf_path = "provided/ur5/ur5.urdf"
            if not os.path.exists(urdf_path):
                urdf_path = "ur5/ur5.urdf"  # Fallback for reference version
                if not os.path.exists(urdf_path):
                    print(f"Error: UR5 URDF not found at provided/ur5/ur5.urdf or ur5/ur5.urdf")
                    print("Available directories:")
                    for d in ["provided/ur5", "ur5"]:
                        if os.path.exists(d):
                            print(f"  {d}/:")
                            for f in os.listdir(d):
                                print(f"    {f}")
                    return False
            
            # Load UR5 with better inertia handling
            self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
            print(f"\n✓ UR5 robot loaded (ID: {self.robot_id})")
            
            # Set joint damping to reduce warnings and improve stability
            num_joints = p.getNumJoints(self.robot_id)
            for i in range(num_joints):
                p.changeDynamics(self.robot_id, i, jointDamping=0.1)
            
            # Create custom environment with primitive obstacles
            self.create_custom_environment()
            
            # Set up lighting and rendering
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable GUI
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Enable shadows
            
            print("✓ Simulator setup complete!")
            return True
            
        except Exception as e:
            print(f"Error setting up simulator: {e}")
            return False
    
    def create_custom_environment(self):
        # Use the same obstacles as the planner
        obstacles = [
            # Lower level spheres (1.0m height) - hexagonal pattern around robot base
            {"type": "sphere", "pos": [0.55, 0, 1.0], "radius": 0.15, "color": [1, 0, 0, 1]},        # Front
            {"type": "sphere", "pos": [0.35, 0.35, 1], "radius": 0.15, "color": [0, 1, 0, 1]},       # Front-right
            {"type": "sphere", "pos": [0, 0.55, 1], "radius": 0.15, "color": [0, 0, 1, 1]},          # Right
            {"type": "sphere", "pos": [-0.55, 0, 1], "radius": 0.15, "color": [1, 1, 0, 1]},         # Back
            {"type": "sphere", "pos": [-0.35, -0.35, 1], "radius": 0.15, "color": [1, 0, 1, 1]},     # Back-left
            {"type": "sphere", "pos": [0, -0.55, 1], "radius": 0.15, "color": [0, 1, 1, 1]},         # Left
            {"type": "sphere", "pos": [0.35, -0.35, 1], "radius": 0.15, "color": [0.5, 0.5, 0.5, 1]}, # Front-left
            
            # Upper level spheres (1.5m height) - offset pattern
            {"type": "sphere", "pos": [0.35, 0.35, 1.5], "radius": 0.15, "color": [1, 0.5, 0, 1]},   # Front-right upper
            {"type": "sphere", "pos": [0, 0.55, 1.5], "radius": 0.15, "color": [0.5, 1, 0, 1]},      # Right upper
            {"type": "sphere", "pos": [-0.35, 0.35, 1.5], "radius": 0.15, "color": [0, 0.5, 1, 1]},  # Back-right upper
            {"type": "sphere", "pos": [-0.55, 0, 1.5], "radius": 0.15, "color": [1, 0, 0.5, 1]},     # Back upper
            {"type": "sphere", "pos": [-0.35, -0.35, 1.5], "radius": 0.15, "color": [0.5, 0, 1, 1]}, # Back-left upper
            {"type": "sphere", "pos": [0, -0.55, 1.5], "radius": 0.15, "color": [1, 1, 0.5, 1]},     # Left upper
            {"type": "sphere", "pos": [0.35, -0.35, 1.5], "radius": 0.15, "color": [0.5, 1, 1, 1]},  # Front-left upper
        ]
        
        for obs in obstacles:
            if obs["type"] == "box":
                shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=obs["size"])
                visual = p.createVisualShape(p.GEOM_BOX, halfExtents=obs["size"], rgbaColor=obs["color"])
            elif obs["type"] == "cylinder":
                shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=obs["radius"], height=obs["height"])
                visual = p.createVisualShape(p.GEOM_CYLINDER, radius=obs["radius"], length=obs["height"], rgbaColor=obs["color"])
            elif obs["type"] == "sphere":
                shape = p.createCollisionShape(p.GEOM_SPHERE, radius=obs["radius"])
                visual = p.createVisualShape(p.GEOM_SPHERE, radius=obs["radius"], rgbaColor=obs["color"])
            
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                             baseVisualShapeIndex=visual, basePosition=obs["pos"])
    
    def load_trajectory_path(self, path_file):
        """Load trajectory from text file (format: joint configurations, one per line)"""
        try:
            if not os.path.exists(path_file):
                print(f"Error: Trajectory file not found: {path_file}")
                return None
            
            trajectory = []
            with open(path_file, 'r') as f:
                lines = f.readlines()
                
                if len(lines) < 1:
                    print("Error: Empty trajectory file")
                    return None
                
                # Read all lines, skipping comments
                for i, line in enumerate(lines):
                    line = line.strip()
                    
                    # Skip empty lines and comments
                    if not line or line.startswith('#'):
                        continue
                    
                    try:
                        config = [float(x) for x in line.split()]
                        if len(config) == 6:  # UR5 has 6 joints
                            trajectory.append(config)
                        else:
                            print(f"Warning: Line {i+1} has {len(config)} joints, expected 6")
                    except ValueError:
                        print(f"Warning: Could not parse line {i+1}: {line}")
                        continue
            
            print(f"✓ Loaded trajectory with {len(trajectory)} waypoints")
            return trajectory
            
        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return None
    
    def precompute_camera_matrices(self, total_frames):
        """Precompute all camera matrices for performance optimization"""
        camera_matrices = []
        
        # Base camera parameters
        robot_base = [0, 0, 0.5]  # UR5 base height
        orbit_radius = 3.0
        camera_target = [robot_base[0] + 0.3, robot_base[1], robot_base[2] + 0.4]
        
        # Precompute projection matrix once (doesn't change)
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=75,  # Increased FOV for better workspace view
            aspect=self.width / self.height,
            nearVal=0.1,
            farVal=100.0
        )
        
        # Precompute all view matrices with anti-collision perspective
        for frame_index in range(total_frames):
            orbit_progress = frame_index / max(1, total_frames - 1)
            # Improved camera angles to clearly show robot avoiding obstacles
            camera_yaw = 45 + (orbit_progress * 90)  # Orbit from 45° to 135° (better obstacle visibility)
            camera_pitch = -35 - (orbit_progress * 5)  # Steeper downward angle to see clearance
            
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target,
                distance=orbit_radius,
                yaw=camera_yaw,
                pitch=camera_pitch,
                roll=0,
                upAxisIndex=2
            )
            
            camera_matrices.append((view_matrix, projection_matrix))
        
        return camera_matrices
    
    def capture_frame(self, view_matrix, projection_matrix):
        """Capture current scene as image frame"""
        # Render the scene with maximum speed optimizations
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_TINY_RENDERER,  # Fastest renderer
            lightDirection=[1, 1, 1],
            lightColor=[1, 1, 1],
            lightDistance=30,  # Further reduced for speed
            shadow=0,  # Shadows disabled
            lightAmbientCoeff=0.9,  # Higher ambient for speed (less computation)
            lightDiffuseCoeff=0.5,  # Reduced diffuse
            lightSpecularCoeff=0.0,  # No specular highlights for speed
            flags=p.ER_NO_SEGMENTATION_MASK  # Skip segmentation for speed
        )
        
        # Convert to OpenCV format (BGR)
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array.reshape((height, width, 4))  # RGBA
        bgr_array = cv2.cvtColor(rgb_array[:, :, :3], cv2.COLOR_RGB2BGR)  # Convert to BGR, drop alpha
        
        return bgr_array
    
    def set_robot_configuration(self, joint_positions):
        """Set UR5 joint positions"""
        if self.robot_id is None:
            return False
        
        # UR5 controllable joints are at indices 1-6 (index 0 is fixed offset_joint)
        # Map the 6 joint values to the correct joint indices
        joint_indices = [1, 2, 3, 4, 5, 6]  # shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        for i, joint_idx in enumerate(joint_indices):
            if i < len(joint_positions):
                p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        return True
    
    def generate_video(self, trajectory):
        """Generate MP4 video of the robot following the trajectory"""
        if not trajectory:
            return False
        
        print("\n" + "="*60)
        print("GENERATING TRAJECTORY VIDEO")
        print("="*60)
        print(f"Trajectory length: {len(trajectory)} waypoints")
        print(f"Output file: {self.output_file}")
        print(f"Video settings: {self.width}x{self.height} @ {self.fps} FPS")
        print(f"Start config: {[f'{x:.3f}' for x in trajectory[0]]}")
        print(f"Goal config:  {[f'{x:.3f}' for x in trajectory[-1]]}")
        print()
        
        try:
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
            
            # Calculate total frames including pauses
            pause_frames = max(1, self.fps // 3)  # Reduced pause time for speed (0.33s vs 0.5s)
            total_video_frames = len(trajectory) + 2 * pause_frames
            
            # Precompute all camera matrices for performance
            camera_matrices = self.precompute_camera_matrices(total_video_frames)
            
            frame_count = 0
            scaling_factor = 5
            total_frames = pause_frames * 2 + len(trajectory)//scaling_factor
            
            # Add initial pause frames showing the start configuration
            self.set_robot_configuration(trajectory[0])
            for _ in range(pause_frames):
                view_matrix, projection_matrix = camera_matrices[frame_count]
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
                frame_count += 1
            
            for i, config in enumerate(trajectory):
                # Reduce number of frames produced to decrease visualization time
                if (i % scaling_factor == 0):
                    self.set_robot_configuration(config)
                    view_matrix, projection_matrix = camera_matrices[frame_count]
                    frame = self.capture_frame(view_matrix, projection_matrix)
                    self.video_writer.write(frame)
                    frame_count += 1
                
                    # Show progress
                    if frame_count % 10 == 0:
                        progress = (frame_count) / total_frames * 100
                        print(f"Progress: {progress:.1f}% ({frame_count}/{total_frames} frames)")
            
            # Add final pause frames showing the goal configuration
            self.set_robot_configuration(trajectory[-1])
            for _ in range(pause_frames):
                view_matrix, projection_matrix = camera_matrices[frame_count]
                frame = self.capture_frame(view_matrix, projection_matrix)
                self.video_writer.write(frame)
                frame_count += 1

            print(f"Progress: {100}% ({frame_count}/{total_frames} frames)")
            
            # Finalize video
            self.video_writer.release()
            
            print(f"\n✓ Trajectory video generation complete!")
            print(f" Video saved as: {self.output_file}")
            print(f" Total frames: {frame_count}")
            print(f" Video duration: {frame_count / self.fps:.1f} seconds")
            
            return True
            
        except Exception as e:
            print(f"Error during video generation: {e}")
            if self.video_writer:
                self.video_writer.release()
            return False
    
    def cleanup(self):
        """Clean up PyBullet connection"""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Generate MP4 videos of UR5 trajectories from COMP 450 Project 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python3 visualize_trajectory.py --path=RTP_path.txt --output=rtp_trajectory.mp4
        """
    )
    
    parser.add_argument('--path', required=True,
                       help='Path to trajectory file (e.g., RTP_path.txt, RRT_path.txt)')
    parser.add_argument('--output', default='trajectory_video.mp4',
                       help='Output video file (default: trajectory_video.mp4)')
    
    return parser.parse_args()

def main():
    """Main function"""
    print("COMP 450 Project 3 - Trajectory Video Generator")
    print("=" * 60)
    
    args = parse_arguments()
    
    # Create output directory if needed
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Create video generator
    generator = TrajectoryVideoGenerator(
        output_file=args.output,
        fps=12,
        width=960,
        height=540
    )
    
    try:
        # Setup simulator
        if not generator.setup_simulator():
            sys.exit(1)
        
        # Load trajectory
        trajectory = generator.load_trajectory_path(args.path)
        if not trajectory:
            sys.exit(1)
        
        # Generate video
        success = generator.generate_video(trajectory)
        
        if success:
            print(f"\n Success! Your video is ready: {args.output}")
        else:
            print("\n Video generation failed.")
            sys.exit(1)
    
    finally:
        # Always cleanup
        generator.cleanup()

if __name__ == "__main__":
    main() 