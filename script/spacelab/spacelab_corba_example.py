#!/usr/bin/env python3
"""
Spacelab manipulation planning example using CORBA backend.

This example demonstrates how to use the agimus_spacelab package with CORBA
to set up a multi-robot manipulation scenario with UR10 and VISPA robots.
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab.corba import CorbaManipulationPlanner
from agimus_spacelab.config import RuleGenerator
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from spacelab_config import (
    InitialConfigurations,
    RobotJoints,
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


def setup_spacelab_scene():
    """
    Set up the complete Spacelab scene with robots and objects.
    
    Returns:
        CorbaManipulationPlanner: Configured planner instance
    """
    print("=" * 70)
    print("Spacelab Manipulation Planning (CORBA Backend)")
    print("=" * 70)
    
    # 1. Initialize planner
    print("\n1. Initializing CORBA planner...")
    planner = CorbaManipulationPlanner()
    
    # 2. Load robot
    print("2. Loading Spacelab robot (UR10 + VISPA)...")
    planner.load_robot(
        name="spacelab-robots",
        urdf_path="package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf",
        srdf_path="package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf",
    )
    
    # 3. Load environment
    print("3. Loading environment...")
    planner.load_environment(
        name="ground_demo",
        urdf_path="package://spacelab_mock_hardware/description/urdf/ground_demo.urdf"
    )
    
    # 4. Load objects
    print("4. Loading manipulable objects...")
    objects = [
        ("RS1", "package://spacelab_mock_hardware/description/urdf/RS.urdf"),
        ("screw_driver", "package://spacelab_mock_hardware/description/urdf/screw_driver.urdf"),
        ("frame_gripper", "package://spacelab_mock_hardware/description/urdf/frame_gripper.urdf"),
        ("cleat_gripper", "package://spacelab_mock_hardware/description/urdf/cleat_gripper.urdf"),
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
    
    bounds = JointBounds.freeflyer()
    for joint in freeflyer_joints:
        planner.set_joint_bounds(joint, bounds)
    
    return planner


def create_manipulation_graph(planner, rule_strategy="auto"):
    """
    Create constraint graph for manipulation planning.
    
    Args:
        planner: CorbaManipulationPlanner instance
        rule_strategy: Rule generation strategy ("auto", "all", etc.)
    
    Returns:
        Constraint graph instance
    """
    print(f"\n6. Creating constraint graph (strategy: {rule_strategy})...")
    
    # Generate rules
    if rule_strategy == "auto":
        rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
        RuleGenerator.print_rule_summary(rules, ManipulationConfig)
    elif rule_strategy == "all":
        rules = "all"
    else:
        rules = "auto"
    
    # Create graph
    graph = planner.create_constraint_graph(
        name="manipulation_graph",
        grippers=list(ManipulationConfig.GRIPPERS.values()),
        objects=ManipulationConfig.OBJECTS,
        rules=rules,
    )
    
    print("✓ Constraint graph created and initialized!")
    
    return graph


def main(solve=False, rule_strategy="auto"):
    """
    Main execution function.
    
    Args:
        solve: Whether to solve the planning problem
        rule_strategy: Rule generation strategy for constraint graph
    """
    # Set up scene
    planner = setup_spacelab_scene()
    
    # Build initial configuration
    print("\n7. Building initial configuration...")
    q_init = build_initial_configuration()
    print(f"   Configuration size: {len(q_init)} DOF")
    print(f"   - Robot DOF: {len(InitialConfigurations.UR10 + InitialConfigurations.VISPA_BASE + InitialConfigurations.VISPA_ARM)}")
    print(f"   - Objects DOF: {len(q_init) - len(InitialConfigurations.UR10 + InitialConfigurations.VISPA_BASE + InitialConfigurations.VISPA_ARM)}")
    
    # Set initial configuration
    planner.set_initial_config(q_init)
    
    # Create constraint graph
    graph = create_manipulation_graph(planner, rule_strategy)
    
    # Visualize
    print("\n8. Starting visualization...")
    planner.visualize(q_init)
    print("✓ Initial configuration displayed!")
    
    # Solve if requested
    if solve:
        print("\n9. Solving planning problem...")
        print("   Note: You need to define a goal configuration first!")
        print("   Example:")
        print("     q_goal = build_goal_configuration()")
        print("     planner.add_goal_config(q_goal)")
        print("     success = planner.solve()")
        print("     if success:")
        print("         planner.play_path(0)")
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    
    print("\nAvailable commands:")
    print("  planner.visualize(q_init)       - Display initial configuration")
    print("  planner.add_goal_config(q_goal) - Add goal configuration")
    print("  planner.solve()                 - Solve planning problem")
    print("  planner.play_path(0)            - Play solution path")
    print("  planner.get_robot()             - Get robot instance")
    print("  planner.get_problem_solver()    - Get problem solver")
    
    print("\nTo solve a planning problem:")
    print("  1. Define q_goal = build_goal_configuration()")
    print("  2. planner.add_goal_config(q_goal)")
    print("  3. planner.solve()")
    print("  4. planner.play_path(0)")
    
    return planner, graph


if __name__ == "__main__":
    import sys
    
    # Parse command line arguments
    solve = "--solve" in sys.argv
    
    # Choose rule strategy
    rule_strategy = "auto"
    if "--rules-all" in sys.argv:
        rule_strategy = "all"
    
    # Run example
    planner, graph = main(solve=solve, rule_strategy=rule_strategy)
    
    print("\n" + "=" * 70)
    print("Example completed!")
    print("=" * 70)
    print("\nThe 'planner' and 'graph' objects are available for interactive use.")
    print("Try: planner.visualize(q_init)")
