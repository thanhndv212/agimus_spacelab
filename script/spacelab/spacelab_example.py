#!/usr/bin/env python3
"""
Spacelab manipulation planning example with configurable backend.

This example demonstrates how to use the agimus_spacelab package with either
PyHPP or CORBA backend to set up a multi-robot manipulation scenario with 
UR10 and VISPA robots.

Usage:
    python spacelab_example.py [--backend {pyhpp,corba}] [--solve] [--no-viz] [--rules-all]
"""

import sys
from pathlib import Path
import numpy as np
import argparse

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

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
        config.RS2,
        config.RS3,
        config.RS4,
        config.RS5,
        config.RS6,
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


def setup_spacelab_scene(backend="pyhpp"):
    """
    Set up the complete Spacelab scene with robots and objects.
    
    Args:
        backend: Planning backend to use ("pyhpp" or "corba")
    
    Returns:
        Planner instance (PyHPPManipulationPlanner or CorbaManipulationPlanner)
    """
    print("=" * 70)
    print(f"Spacelab Manipulation Planning ({backend.upper()} Backend)")
    print("=" * 70)
    
    # 1. Initialize planner based on backend
    print(f"\n1. Initializing {backend.upper()} planner...")
    if backend == "pyhpp":
        from agimus_spacelab.pyhpp import PyHPPManipulationPlanner
        planner = PyHPPManipulationPlanner()
        pkg_prefix = "package://spacelab_mock_hardware/description"
    else:  # corba
        from agimus_spacelab.corba import CorbaManipulationPlanner
        planner = CorbaManipulationPlanner()
        pkg_prefix = "package://spacelab_mock_hardware/description"
    
    # 2. Load robot
    print("2. Loading Spacelab robot (UR10 + VISPA)...")
    if backend == "pyhpp":
        planner.load_robot(
            name="spacelab",
            urdf_path=f"{pkg_prefix}/urdf/allRobots_spacelab_robot.urdf",
            srdf_path=f"{pkg_prefix}/srdf/allRobots_spacelab_robot.srdf",
        )
    else:  # corba
        planner.load_robot(
            composite_name="spacelab",
            robot_name="spacelab",
            urdf_path=f"{pkg_prefix}/urdf/allRobots_spacelab_robot.urdf",
            srdf_path=f"{pkg_prefix}/srdf/allRobots_spacelab_robot.srdf",
        )
    
    # 3. Load environment
    print("3. Loading environment...")
    if backend == "pyhpp":
        planner.load_environment(
            name="spacelab-scene",
            urdf_path=f"{pkg_prefix}/urdf/ground_demo.urdf"
        )
    else:  # corba
        planner.load_environment(
            name="ground_demo",
            urdf_path=f"{pkg_prefix}/urdf/ground_demo.urdf"
        )
    
    # 4. Load objects
    print("4. Loading manipulable objects...")
    objects = [
        ("RS1", f"{pkg_prefix}/urdf/RS1.urdf"),
        ("RS2", f"{pkg_prefix}/urdf/RS2.urdf"),
        ("RS3", f"{pkg_prefix}/urdf/RS3.urdf"),
        ("RS4", f"{pkg_prefix}/urdf/RS4.urdf"),
        ("RS5", f"{pkg_prefix}/urdf/RS5.urdf"),
        ("RS6", f"{pkg_prefix}/urdf/RS6.urdf"),
        ("screw_driver", f"{pkg_prefix}/urdf/screw_driver.urdf"),
        ("frame_gripper", f"{pkg_prefix}/urdf/frame_gripper.urdf"),
        ("cleat_gripper", f"{pkg_prefix}/urdf/cleat_gripper.urdf"),
    ]
    
    for obj_name, obj_path in objects:
        planner.load_object(obj_name, obj_path, root_joint_type="freeflyer")
    
    # 5. Set joint bounds for free-flying objects
    print("5. Setting joint bounds...")
    freeflyer_joints = [
        "RS1/root_joint",
        "RS2/root_joint",
        "RS3/root_joint",
        "RS4/root_joint",
        "RS5/root_joint",
        "RS6/root_joint",
        "screw_driver/root_joint",
        "frame_gripper/root_joint",
        "cleat_gripper/root_joint",
    ]
    
    bounds = JointBounds.freeflyer_bounds()
    for joint in freeflyer_joints:
        planner.set_joint_bounds(joint, bounds)
    
    return planner


def create_manipulation_graph(planner, backend="pyhpp", rule_strategy="auto"):
    """
    Create constraint graph for manipulation planning.
    
    Args:
        planner: Planner instance
        backend: Planning backend ("pyhpp" or "corba")
        rule_strategy: Rule generation strategy ("auto", "all", etc.)
    
    Returns:
        Constraint graph instance
    """
    print(f"\n6. Creating constraint graph (strategy: {rule_strategy})...")
    
    # Generate rules based on backend and strategy
    if backend == "corba" and rule_strategy == "auto":
        from agimus_spacelab.config import RuleGenerator
        rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
        RuleGenerator.print_rule_summary(rules, ManipulationConfig)
    elif rule_strategy == "all":
        rules = "all"
    else:
        rules = "auto"
    
    # Create graph with basic structure
    graph = planner.create_constraint_graph(
        name="manipulation_graph",
        grippers=list(ManipulationConfig.GRIPPERS.values()),
        objects=ManipulationConfig.OBJECTS,
        rules=rules,
    )
    
    print("✓ Constraint graph created and initialized!")
    
    return graph


def print_usage_instructions(planner, backend, solve, has_path):
    """
    Print usage instructions based on the current state.
    
    Args:
        planner: Planner instance
        backend: Planning backend used
        solve: Whether solving was attempted
        has_path: Whether a solution path exists
    """
    print("\nAvailable commands:")
    print("  planner.visualize(q_init)       - Display initial configuration")
    
    if backend == "pyhpp":
        print("  planner.visualize(q_goal)       - Display goal configuration")
        print("  planner.solve()                 - Solve planning problem")
        print("  planner.play_path()             - Play solution path")
        print("  planner.get_robot()             - Get device instance")
        print("  planner.get_problem()           - Get problem instance")
        print("  planner.get_graph()             - Get constraint graph")
    else:  # corba
        print("  planner.add_goal_config(q_goal) - Add goal configuration")
        print("  planner.solve()                 - Solve planning problem")
        print("  planner.play_path(0)            - Play solution path")
        print("  planner.get_robot()             - Get robot instance")
        print("  planner.get_problem_solver()    - Get problem solver")
    
    if solve and has_path:
        print("\nTo animate the solution:")
        if backend == "pyhpp":
            print("  planner.play_path()")
        else:
            print("  planner.play_path(0)")
    elif not solve:
        if backend == "corba":
            print("\nTo solve a planning problem:")
            print("  1. Define q_goal = build_goal_configuration(q_init)")
            print("  2. planner.add_goal_config(q_goal)")
            print("  3. planner.solve()")
            print("  4. planner.play_path(0)")
        else:
            print("\nTry: planner.solve() then planner.play_path()")


def main(backend="pyhpp", solve=False, visualize=True, rule_strategy="auto"):
    """
    Main execution function.
    
    Args:
        backend: Planning backend to use ("pyhpp" or "corba")
        solve: Whether to solve the planning problem
        visualize: Whether to visualize the scene
        rule_strategy: Rule generation strategy for constraint graph
    """
    # Set up scene
    planner = setup_spacelab_scene(backend)
    
    # Build configurations
    print("\n7. Building configurations...")
    q_init = build_initial_configuration()
    
    robot_dof = len(InitialConfigurations.UR10 + 
                   InitialConfigurations.VISPA_BASE + 
                   InitialConfigurations.VISPA_ARM)
    print(f"   Configuration size: {len(q_init)} DOF")
    print(f"   - Robot DOF: {robot_dof}")
    print(f"   - Objects DOF: {len(q_init) - robot_dof}")
    
    # Set initial configuration
    planner.set_initial_config(q_init)
    
    # For PyHPP, also set goal configuration if solving
    if backend == "pyhpp" and solve:
        q_goal = build_goal_configuration(q_init)
        planner.add_goal_config(q_goal)
    
    # Create constraint graph
    graph = create_manipulation_graph(planner, backend, rule_strategy)
    
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
    has_path = False
    if solve:
        print("\n9. Solving planning problem...")
        
        if backend == "corba":
            print("   Note: For CORBA backend, define goal first:")
            print("     q_goal = build_goal_configuration(q_init)")
            print("     planner.add_goal_config(q_goal)")
            print("     planner.solve()")
        else:
            success = planner.solve(max_iterations=5000)
            
            if success:
                print("✓ Solution found!")
                has_path = True
            else:
                print("✗ No solution found")
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    
    print_usage_instructions(planner, backend, solve, has_path)
    
    return planner, graph


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Spacelab manipulation planning with configurable backend",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use PyHPP backend (default)
  python spacelab_example.py
  
  # Use CORBA backend
  python spacelab_example.py --backend corba
  
  # Solve planning problem with PyHPP
  python spacelab_example.py --backend pyhpp --solve
  
  # Use all grasp rules (more edges in graph)
  python spacelab_example.py --backend corba --rules-all
  
  # Run without visualization
  python spacelab_example.py --no-viz
        """
    )
    
    parser.add_argument(
        "--backend",
        choices=["pyhpp", "corba"],
        default="pyhpp",
        help="Planning backend to use (default: pyhpp)"
    )
    
    parser.add_argument(
        "--solve",
        action="store_true",
        help="Solve the planning problem"
    )
    
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip visualization (useful for headless systems)"
    )
    
    parser.add_argument(
        "--rules-all",
        action="store_true",
        help="Use all possible grasp rules instead of auto-generated"
    )
    
    return parser.parse_args()


if __name__ == "__main__":
    # Parse command line arguments
    args = parse_arguments()
    
    # Determine rule strategy
    rule_strategy = "all" if args.rules_all else "auto"
    
    # Run example
    planner, graph = main(
        backend=args.backend,
        solve=args.solve,
        visualize=not args.no_viz,
        rule_strategy=rule_strategy
    )
    
    print("\n" + "=" * 70)
    print("Example completed!")
    print("=" * 70)
    print(f"\nBackend: {args.backend.upper()}")
    print("The 'planner' and 'graph' objects are available for interactive use.")
    
    if args.backend == "pyhpp" and args.solve and planner.get_path():
        print("\nTry: planner.play_path()")
    else:
        print("\nTry: planner.visualize(q_init)")
