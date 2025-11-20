#!/usr/bin/env python3
"""
Spacelab manipulation planning example using PyHPP backend.

This example demonstrates how to use the agimus_spacelab package with PyHPP
to set up a multi-robot manipulation scenario with UR10 and VISPA robots.
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab.pyhpp import PyHPPManipulationPlanner
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from spacelab_config import (
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)


def build_initial_configuration():
    """
    Construct the initial configuration for all robots and objects.
    
    Returns:
        np.ndarray: Initial configuration vector
    """
    config = InitialConfigurations
    
    # Start with robot configurations
    # UR10 (6 DOF) + VISPA_BASE (2 DOF) + VISPA_ARM (6 DOF)
    q_robot = np.array(config.UR10 + config.VISPA_BASE + config.VISPA_ARM)
    
    # Add object poses (convert XYZRPY to XYZQUAT for freeflyer)
    object_poses = [
        config.RS1,
        config.SCREW_DRIVER,
        config.FRAME_GRIPPER,
        config.CLEAT_GRIPPER,
    ]
    
    q_objects = []
    for pose_xyzrpy in object_poses:
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_objects.extend(pose_xyzquat.tolist())
    
    # Combine robot and object configurations
    q_init = np.concatenate([q_robot, q_objects])
    
    return q_init


def build_goal_configuration(q_init):
    """
    Build a goal configuration with modified object poses.
    
    Args:
        q_init: Initial configuration
        
    Returns:
        np.ndarray: Goal configuration vector
    """
    # Start with initial configuration
    q_goal = q_init.copy()
    
    # Modify RS1 position (assuming it's the first object after robots)
    robot_dof = len(InitialConfigurations.UR10 + 
                   InitialConfigurations.VISPA_BASE + 
                   InitialConfigurations.VISPA_ARM)
    
    # Move RS1 0.2m in x direction
    q_goal[robot_dof] += 0.2
    
    return q_goal


def setup_spacelab_scene():
    """
    Set up the complete Spacelab scene with robots and objects.
    
    Returns:
        PyHPPManipulationPlanner: Configured planner instance
    """
    print("=" * 70)
    print("Spacelab Manipulation Planning (PyHPP Backend)")
    print("=" * 70)
    
    # 1. Initialize planner
    print("\n1. Initializing PyHPP planner...")
    planner = PyHPPManipulationPlanner()
    
    # 2. Load robot
    print("2. Loading Spacelab robot (UR10 + VISPA)...")
    pkg = "package://spacelab_mock_hardware/description"
    planner.load_robot(
        name="spacelab-robots",
        urdf_path=f"{pkg}/urdf/allRobots_spacelab_robot.urdf",
        srdf_path=f"{pkg}/srdf/allRobots_spacelab_robot.srdf",
    )
    
    # 3. Load environment
    print("3. Loading environment...")
    planner.load_environment(
        name="spacelab-scene",
        urdf_path=f"{pkg}/urdf/ground_demo.urdf"
    )
    
    # 4. Load objects
    print("4. Loading manipulable objects...")
    objects = [
        ("RS1", f"{pkg}/urdf/RS.urdf"),
        ("screw_driver", f"{pkg}/urdf/screw_driver.urdf"),
        ("frame_gripper", f"{pkg}/urdf/frame_gripper.urdf"),
        ("cleat_gripper", f"{pkg}/urdf/cleat_gripper.urdf"),
    ]
    
    for obj_name, obj_path in objects:
        planner.load_object(obj_name, obj_path, root_joint_type="freeflyer")
    
    # 5. Set joint bounds for free-flying objects
    print("5. Setting joint bounds...")
    freeflyer_joints = [
        "RS1/root_joint",
        "screw_driver/root_joint",
        "frame_gripper/root_joint",
        "cleat_gripper/root_joint",
    ]
    
    bounds = JointBounds.freeflyer_bounds()
    for joint in freeflyer_joints:
        planner.set_joint_bounds(joint, bounds)
    
    return planner


def create_manipulation_graph(planner):
    """
    Create constraint graph for manipulation planning.
    
    Args:
        planner: PyHPPManipulationPlanner instance
    
    Returns:
        Constraint graph instance
    """
    print("\n6. Creating constraint graph...")
    
    # Create graph with basic structure
    graph = planner.create_constraint_graph(
        name="manipulation_graph",
        grippers=list(ManipulationConfig.GRIPPERS.values()),
        objects=ManipulationConfig.OBJECTS,
        rules="auto",
    )
    
    print("✓ Constraint graph created and initialized!")
    
    return graph


def main(solve=False, visualize=True):
    """
    Main execution function.
    
    Args:
        solve: Whether to solve the planning problem
        visualize: Whether to visualize the scene
    """
    # Set up scene
    planner = setup_spacelab_scene()
    
    # Build configurations
    print("\n7. Building configurations...")
    q_init = build_initial_configuration()
    q_goal = build_goal_configuration(q_init)
    
    print(f"   Configuration size: {len(q_init)} DOF")
    robot_dof = len(InitialConfigurations.UR10 + 
                   InitialConfigurations.VISPA_BASE + 
                   InitialConfigurations.VISPA_ARM)
    print(f"   - Robot DOF: {robot_dof}")
    print(f"   - Objects DOF: {len(q_init) - robot_dof}")
    
    # Set configurations
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    # Create constraint graph
    graph = create_manipulation_graph(planner)
    
    # Visualize
    if visualize:
        print("\n8. Starting visualization...")
        try:
            planner.visualize(q_init)
            print("✓ Initial configuration displayed!")
        except Exception as e:
            print(f"⚠ Visualization failed: {e}")
            print("  (This is normal if running without display)")
    
    # Solve if requested
    if solve:
        print("\n9. Solving planning problem...")
        success = planner.solve(max_iterations=5000)
        
        if success:
            print("✓ Solution found!")
            print("\nTo play the path:")
            print("  planner.play_path()")
        else:
            print("✗ No solution found")
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    
    print("\nAvailable commands:")
    print("  planner.visualize(q_init)       - Display initial configuration")
    print("  planner.visualize(q_goal)       - Display goal configuration")
    print("  planner.solve()                 - Solve planning problem")
    print("  planner.play_path()             - Play solution path")
    print("  planner.get_robot()             - Get device instance")
    print("  planner.get_problem()           - Get problem instance")
    print("  planner.get_graph()             - Get constraint graph")
    
    if solve and planner.get_path():
        print("\nTo animate the solution:")
        print("  planner.play_path()")
    
    return planner, graph


if __name__ == "__main__":
    import sys
    
    # Parse command line arguments
    solve = "--solve" in sys.argv
    no_viz = "--no-viz" in sys.argv
    
    # Run example
    planner, graph = main(solve=solve, visualize=not no_viz)
    
    print("\n" + "=" * 70)
    print("Example completed!")
    print("=" * 70)
    print("\nThe 'planner' and 'graph' objects are available for use.")
    
    if solve and planner.get_path():
        print("\nTry: planner.play_path()")
    else:
        print("\nTry: planner.solve() then planner.play_path()")
