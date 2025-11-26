#!/usr/bin/env python3
"""
Spacelab manipulation planning example: UR10 grasps frame_gripper.

This example demonstrates manipulation planning with CORBA where:
1. UR10 robot approaches the frame_gripper tool
2. UR10 grasps the frame_gripper by its handle (h_FG_tool)
3. UR10 lifts and moves the frame_gripper

Task sequence:
1. placement -> approach-tool -> gripper-above-tool
2. gripper-above-tool -> grasp-tool -> grasp-placement
3. grasp-placement -> take-tool-up -> tool-in-air
4. tool-in-air -> grasp (free motion with tool)

Based on: graspball_inbox_corba_example.py
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import (
    InitialConfigurations,
    RobotJoints,
    JointBounds,
    ManipulationConfig,
)

from agimus_spacelab.corba import CorbaManipulationPlanner
from agimus_spacelab.utils import xyzrpy_to_xyzquat

# CORBA-specific imports
try:
    from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
    from hpp.corbaserver import Client
    # Reset problem before starting
    Client().problem.resetProblem()
    HAS_CORBA = True
except ImportError:
    HAS_CORBA = False
    print("Warning: CORBA backend not available")


# ============================================================================
# Task Configuration
# ============================================================================

class TaskConfig:
    """Configuration specific to the UR10 grasp frame_gripper task."""
    
    # Gripper and object names (must match URDF/SRDF)
    GRIPPER_NAME = "spacelab/ur10_joint_6_7"  # UR10 end-effector link
    TOOL_NAME = "frame_gripper/root_joint"  # Frame gripper freeflyer joint
    
    # Tool in gripper transform [x, y, z, qx, qy, qz, qw]
    # From SRDF: handle position is [0, 0, 0, 0, -0.707, 0, 0.707]
    # This defines how the tool sits in the gripper when grasped
    TOOL_IN_GRIPPER = [0.0, 0.0, 0.1, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
    
    # Gripper above tool transform (approach pose)
    GRIPPER_ABOVE_TOOL = [0.0, 0.0, 0.2, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
    
    # Tool placement on dispenser (where tool rests initially)
    # From InitialConfigurations.FRAME_GRIPPER converted to xyzquat
    TOOL_ON_DISPENSER = None  # Will be computed from initial config
    
    # Tool lifted position (tool held in air)
    TOOL_IN_AIR = None  # Will be computed relative to dispenser
    
    # Tool position relative to dispenser surface [x, y, z, qx, qy, qz, qw]
    # This defines where the tool sits on the dispenser
    TOOL_ON_SURFACE = None  # Will be computed from initial config
    
    # Constraint masks (6 DOF: x, y, z, roll, pitch, yaw)
    GRASP_MASK = [True, True, True, True, True, True]  # All DOF fixed
    
    # Placement mask: fix Z and all rotations (tool flat on surface)
    # This acts like a "surface contact" - tool stays on surface
    PLACEMENT_MASK = [False, False, True, True, True, True]
    
    # Placement complement mask: X, Y free (tool can slide on surface)
    PLACEMENT_COMPLEMENT_MASK = [True, True, False, False, False, False]
    
    # Graph states (order matters for solver performance!)
    GRAPH_NODES = [
        "grasp",               # Tool grasped, free motion
        "tool-in-air",         # Tool grasped and lifted
        "grasp-placement",     # Tool grasped while on dispenser
        "gripper-above-tool",  # Gripper aligned above tool
        "placement",           # Tool on dispenser, gripper free
    ]
    
    # Path planning parameters
    PATH_VALIDATION_STEP = 0.01
    PATH_PROJECTOR_STEP = 0.1
    MAX_RANDOM_ATTEMPTS = 1000
    
    # Collision pair to disable: tool on dispenser surface
    # These are the joint names for collision filtering
    TOOL_CONTACT_JOINT = "frame_gripper/root_joint"
    # Use "universe" for environment/fixed bodies (dispenser is fixed)
    # HPP maps "universe" to index 0 for collision matrix
    DISPENSER_CONTACT_JOINT = "universe"
    
    # Edges where tool-dispenser collision should be disabled
    PLACEMENT_EDGES = [
        "transit",           # Tool on dispenser, gripper free
        "approach-tool",     # Gripper approaching tool on dispenser
        "move-gripper-away", # Gripper moving away from tool
        "grasp-tool",        # Gripper grasping tool on dispenser
        "release-tool",      # Gripper releasing tool on dispenser
        "lift-tool",         # Lifting tool from dispenser
        "lower-tool",        # Lowering tool to dispenser
    ]
    
    @classmethod
    def init_tool_positions(cls):
        """Initialize tool positions from initial configuration."""
        # Convert XYZRPY to XYZQUAT
        tool_pose_xyzrpy = InitialConfigurations.FRAME_GRIPPER
        tool_pose_quat = xyzrpy_to_xyzquat(tool_pose_xyzrpy)
        
        cls.TOOL_ON_DISPENSER = tool_pose_quat.tolist()
        
        # Tool on surface: same position, used for surface constraint
        # The Z value defines the contact height on the dispenser
        cls.TOOL_ON_SURFACE = tool_pose_quat.tolist()
        
        # Tool in air: same x,y but lifted 0.15m in z
        cls.TOOL_IN_AIR = tool_pose_quat.copy()
        cls.TOOL_IN_AIR[2] += 0.15  # Lift by 15cm
        cls.TOOL_IN_AIR = cls.TOOL_IN_AIR.tolist()


# Initialize task configuration
TaskConfig.init_tool_positions()


# ============================================================================
# Constraint Creation
# ============================================================================

def create_transformation_constraints(ps):
    """
    Define all transformation constraints for the task.
    
    Args:
        ps: ProblemSolver instance
    """
    print("  Creating transformation constraints...")
    
    # 1. GRASP constraint: gripper holds tool (all DOF fixed)
    ps.createTransformationConstraint(
        "grasp",
        TaskConfig.GRIPPER_NAME,
        TaskConfig.TOOL_NAME,
        TaskConfig.TOOL_IN_GRIPPER,
        TaskConfig.GRASP_MASK
    )
    print(f"    ✓ grasp: {TaskConfig.GRIPPER_NAME} -> {TaskConfig.TOOL_NAME}")
    
    # 2. PLACEMENT constraint: tool on dispenser surface
    # Fixes Z position and all rotations - acts as a surface contact
    # Tool cannot penetrate because Z is constrained to surface height
    ps.createTransformationConstraint(
        "placement",
        "",  # World frame
        TaskConfig.TOOL_NAME,
        TaskConfig.TOOL_ON_DISPENSER,
        TaskConfig.PLACEMENT_MASK  # Z, roll, pitch, yaw fixed
    )
    print(f"    ✓ placement: tool Z={TaskConfig.TOOL_ON_DISPENSER[2]:.3f} (surface contact)")
    
    # 3. PLACEMENT/COMPLEMENT: tool can slide on surface (X, Y free)
    ps.createTransformationConstraint(
        "placement/complement",
        "",
        TaskConfig.TOOL_NAME,
        TaskConfig.TOOL_ON_DISPENSER,
        TaskConfig.PLACEMENT_COMPLEMENT_MASK  # X, Y free
    )
    print("    ✓ placement/complement: X, Y free (sliding on surface)")
    
    # 4. GRIPPER_TOOL_ALIGNED: gripper above tool (all DOF fixed)
    ps.createTransformationConstraint(
        "gripper_tool_aligned",
        TaskConfig.GRIPPER_NAME,
        TaskConfig.TOOL_NAME,
        TaskConfig.GRIPPER_ABOVE_TOOL,
        [True, True, True, True, True, True]
    )
    print("    ✓ gripper_tool_aligned")
    
    # 5. TOOL_IN_AIR constraint: tool at lifted position
    ps.createTransformationConstraint(
        "tool_in_air",
        "",
        TaskConfig.TOOL_NAME,
        TaskConfig.TOOL_IN_AIR,
        TaskConfig.PLACEMENT_MASK
    )
    print(f"    ✓ tool_in_air: tool at {TaskConfig.TOOL_IN_AIR[:3]}")
    
    # 6. TOOL_IN_AIR/COMPLEMENT: tool can move in x-y, rotate in yaw
    ps.createTransformationConstraint(
        "tool_in_air/complement",
        "",
        TaskConfig.TOOL_NAME,
        TaskConfig.TOOL_IN_AIR,
        TaskConfig.PLACEMENT_COMPLEMENT_MASK
    )
    print("    ✓ tool_in_air/complement")


# ============================================================================
# Constraint Graph Creation
# ============================================================================

def create_constraint_graph(robot, ps):
    """
    Create and configure the constraint graph.
    
    Args:
        robot: Robot instance
        ps: ProblemSolver instance
        
    Returns:
        ConstraintGraph object
    """
    print("  Creating constraint graph...")
    
    graph = ConstraintGraph(robot, "graph")
    
    # Create all nodes (states) - order matters!
    graph.createNode(TaskConfig.GRAPH_NODES)
    print(f"    ✓ Created {len(TaskConfig.GRAPH_NODES)} states")
    
    # ========================================================================
    # Create edges (transitions)
    # ========================================================================
    
    # Self-loops
    graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
    graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
    
    # From placement (tool on dispenser, gripper free)
    graph.createEdge(
        'placement', 'gripper-above-tool',
        'approach-tool', 1, 'placement'
    )
    
    # From gripper-above-tool
    graph.createEdge(
        'gripper-above-tool', 'placement',
        'move-gripper-away', 1, 'placement'
    )
    graph.createEdge(
        'gripper-above-tool', 'grasp-placement',
        'grasp-tool', 1, 'placement'
    )
    
    # From grasp-placement (tool grasped, still on dispenser)
    graph.createEdge(
        'grasp-placement', 'gripper-above-tool',
        'release-tool', 1, 'placement'
    )
    graph.createEdge(
        'grasp-placement', 'tool-in-air',
        'lift-tool', 1, 'grasp'
    )
    
    # From tool-in-air (tool grasped and lifted)
    graph.createEdge(
        'tool-in-air', 'grasp-placement',
        'lower-tool', 1, 'grasp'
    )
    graph.createEdge(
        'tool-in-air', 'grasp',
        'move-tool-away', 1, 'grasp'
    )
    
    # From grasp (tool grasped, free motion)
    graph.createEdge(
        'grasp', 'tool-in-air',
        'approach-dispenser', 1, 'grasp'
    )
    
    print("    ✓ Created edges (transitions)")
    
    # ========================================================================
    # Assign constraints to nodes (states)
    # ========================================================================
    
    graph.addConstraints(
        node="placement",
        constraints=Constraints(numConstraints=["placement"])
    )
    
    graph.addConstraints(
        node="gripper-above-tool",
        constraints=Constraints(
            numConstraints=["placement", "gripper_tool_aligned"]
        )
    )
    
    graph.addConstraints(
        node="grasp-placement",
        constraints=Constraints(numConstraints=["grasp", "placement"])
    )
    
    graph.addConstraints(
        node="tool-in-air",
        constraints=Constraints(numConstraints=["grasp", "tool_in_air"])
    )
    
    graph.addConstraints(
        node="grasp",
        constraints=Constraints(numConstraints=["grasp"])
    )
    
    print("    ✓ Added constraints to nodes")
    
    # ========================================================================
    # Assign constraints to edges
    # ========================================================================
    
    # Tool on dispenser edges: use placement/complement
    for edge_name in ["transit", "approach-tool", "move-gripper-away"]:
        graph.addConstraints(
            edge=edge_name,
            constraints=Constraints(numConstraints=["placement/complement"])
        )
    
    # Grasp/release while on dispenser: use placement/complement
    for edge_name in ["grasp-tool", "release-tool"]:
        graph.addConstraints(
            edge=edge_name,
            constraints=Constraints(numConstraints=["placement/complement"])
        )
    
    # Lift/lower tool: use tool_in_air/complement
    for edge_name in ["lift-tool", "lower-tool"]:
        graph.addConstraints(
            edge=edge_name,
            constraints=Constraints(numConstraints=["tool_in_air/complement"])
        )
    
    # Free motion with tool: no path constraints
    for edge_name in ["transfer", "move-tool-away", "approach-dispenser"]:
        graph.addConstraints(
            edge=edge_name,
            constraints=Constraints()
        )
    
    print("    ✓ Added constraints to edges")
    
    # ========================================================================
    # Set constant right-hand side for constraints
    # ========================================================================
    
    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("placement/complement", False)
    ps.setConstantRightHandSide("tool_in_air/complement", False)
    
    print("    ✓ Set constant right-hand side")
    
    # ========================================================================
    # Set security margins for placement contact (BEFORE initialize!)
    # ========================================================================
    # The tool rests on the dispenser surface - this is expected contact.
    # Use negative security margin to allow penetration/contact.
    # 
    # NOTE: We use setSecurityMarginForEdge instead of removeCollisionPairFromEdge
    # because removeCollisionPairFromEdge has a bug - it doesn't handle the 
    # "universe" joint (index 0) properly and crashes.
    #
    # NOTE: Must be done BEFORE graph.initialize() because setting margins
    # invalidates the edges, requiring re-initialization.
    
    contact_margin = -0.02  # Allow 2cm penetration for contact
    
    for edge_name in TaskConfig.PLACEMENT_EDGES:
        graph.setSecurityMarginForEdge(
            edge_name,
            TaskConfig.TOOL_CONTACT_JOINT,
            TaskConfig.DISPENSER_CONTACT_JOINT,
            contact_margin
        )
    
    print(f"    ✓ Set security margin ({contact_margin}m) for placement edges")
    
    # ========================================================================
    # Initialize graph (AFTER setting security margins!)
    # ========================================================================
    
    graph.initialize()
    print("    ✓ Graph initialized")
    
    return graph


# ============================================================================
# Configuration Generation
# ============================================================================

def build_initial_configuration():
    """
    Construct the initial configuration for all robots and objects.
    
    Returns:
        list: Initial configuration vector
    """
    config = InitialConfigurations
    
    # Robot configurations
    q_robot = config.UR10 + config.VISPA_BASE + config.VISPA_ARM
    
    # Object poses (convert XYZRPY to XYZQUAT)
    object_poses = [
        # config.RS1,
        # config.SCREW_DRIVER,
        config.FRAME_GRIPPER,
        # config.CLEAT_GRIPPER,
    ]
    
    q_objects = []
    for pose_xyzrpy in object_poses:
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_objects.extend(pose_xyzquat.tolist())
    
    return q_robot + q_objects


def generate_configurations(robot, graph, ps, q_init):
    """
    Generate intermediate configurations for the manipulation task.
    
    Args:
        robot: Robot instance
        graph: ConstraintGraph instance
        ps: ProblemSolver instance
        q_init: Initial configuration
        
    Returns:
        dict: Dictionary of generated configurations
    """
    print("\n  Generating configurations...")
    configs = {}
    q1 = list(q_init)
    
    # 1. Project initial config on placement state
    print("    1. Projecting onto 'placement' state...")
    res, configs["q_init"], err = graph.applyNodeConstraints("placement", q1)
    if not res:
        print(f"       ⚠ Failed (error: {err})")
        configs["q_init"] = q1
    else:
        print("       ✓ Initial configuration projected")
    
    # 2. Generate config after approach-tool
    print("    2. Generating 'approach-tool' config...")
    for i in range(TaskConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_at, err = graph.generateTargetConfig(
            "approach-tool", configs["q_init"], q_rand
        )
        if res:
            configs["q_approach"] = q_at
            break
        if (i + 1) % 200 == 0:
            print(f"       Attempt {i + 1}...")
    
    if res:
        print("       ✓ Approach configuration generated")
    else:
        print(f"       ⚠ Failed after {TaskConfig.MAX_RANDOM_ATTEMPTS} attempts")
        return configs
    
    # 3. Project onto gripper-above-tool
    print("    3. Projecting onto 'gripper-above-tool' state...")
    res, configs["q_above"], err = graph.applyNodeConstraints(
        "gripper-above-tool", configs["q_approach"]
    )
    if res:
        print("       ✓ Gripper-above-tool configuration")
    else:
        print(f"       ⚠ Failed (error: {err})")
        configs["q_above"] = configs["q_approach"]
    
    # 4. Generate config after grasp-tool
    print("    4. Generating 'grasp-tool' config...")
    for i in range(TaskConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_gt, err = graph.generateTargetConfig(
            "grasp-tool", configs["q_above"], q_rand
        )
        if res:
            configs["q_grasp"] = q_gt
            break
        if (i + 1) % 200 == 0:
            print(f"       Attempt {i + 1}...")
    
    if res:
        print("       ✓ Grasp configuration generated")
    else:
        print(f"       ⚠ Failed after {TaskConfig.MAX_RANDOM_ATTEMPTS} attempts")
        return configs
    
    # 5. Project onto grasp-placement
    print("    5. Projecting onto 'grasp-placement' state...")
    res, configs["q_grasp_place"], err = graph.applyNodeConstraints(
        "grasp-placement", configs["q_grasp"]
    )
    if res:
        print("       ✓ Grasp-placement configuration")
    else:
        print(f"       ⚠ Failed (error: {err})")
        configs["q_grasp_place"] = configs["q_grasp"]
    
    # 6. Generate config after lift-tool
    print("    6. Generating 'lift-tool' config...")
    for i in range(TaskConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_lt, err = graph.generateTargetConfig(
            "lift-tool", configs["q_grasp_place"], q_rand
        )
        if res:
            configs["q_lifted"] = q_lt
            break
        if (i + 1) % 200 == 0:
            print(f"       Attempt {i + 1}...")
    
    if res:
        print("       ✓ Lifted configuration generated")
    else:
        print(f"       ⚠ Failed after {TaskConfig.MAX_RANDOM_ATTEMPTS} attempts")
        return configs
    
    # 7. Project onto tool-in-air
    print("    7. Projecting onto 'tool-in-air' state...")
    res, configs["q_tool_air"], err = graph.applyNodeConstraints(
        "tool-in-air", configs["q_lifted"]
    )
    if res:
        print("       ✓ Tool-in-air configuration")
    else:
        print(f"       ⚠ Failed (error: {err})")
        configs["q_tool_air"] = configs["q_lifted"]
    
    # 8. Generate goal config (tool moved to new position)
    print("    8. Generating goal configuration...")
    q_goal = list(configs["q_init"])
    # Move frame_gripper to a new position
    # Frame gripper is the 3rd object, each object has 7 DOF (freeflyer)
    robot_dof = len(InitialConfigurations.UR10 +
                   InitialConfigurations.VISPA_BASE +
                   InitialConfigurations.VISPA_ARM)
    frame_gripper_idx = robot_dof  # After RS1 and screw_driver
    q_goal[frame_gripper_idx] += 0.2  # Move 20cm in x
    
    res, configs["q_goal"], err = graph.applyNodeConstraints("placement", q_goal)
    if res:
        print("       ✓ Goal configuration generated")
    else:
        print(f"       ⚠ Failed (error: {err})")
        configs["q_goal"] = q_goal
    
    return configs


# ============================================================================
# Main Functions
# ============================================================================

def setup_scene():
    """
    Set up the complete scene with robots and objects.
    
    Returns:
        tuple: (planner, robot, ps) instances
    """
    print("\n1. Setting up scene...")
    
    # Initialize planner
    planner = CorbaManipulationPlanner()
    
    # Load robot
    print("   Loading robot...")
    planner.load_robot(
        composite_name="spacelab",
        robot_name="spacelab",
        urdf_path="package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf",
        srdf_path="package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf",
    )
    
    # Load environment
    print("   Loading environment...")
    planner.load_environment(
        name="ground_demo",
        urdf_path="package://spacelab_mock_hardware/description/urdf/ground_demo.urdf"
    )
    
    # Load objects
    print("   Loading objects...")
    objects = [
        # ("RS1", "package://spacelab_mock_hardware/description/urdf/RS.urdf"),
        # ("screw_driver", "package://spacelab_mock_hardware/description/urdf/screw_driver.urdf"),
        ("frame_gripper", "package://spacelab_mock_hardware/description/urdf/frame_gripper.urdf"),
        # ("cleat_gripper", "package://spacelab_mock_hardware/description/urdf/cleat_gripper.urdf"),
    ]
    
    for obj_name, obj_path in objects:
        planner.load_object(obj_name, obj_path, root_joint_type="freeflyer")
    
    # Set joint bounds
    print("   Setting joint bounds...")
    freeflyer_joints = [
        # "RS1/root_joint",
        # "screw_driver/root_joint",
        "frame_gripper/root_joint",
        # "cleat_gripper/root_joint",
    ]
    bounds = JointBounds.freeflyer_bounds()
    for joint in freeflyer_joints:
        planner.set_joint_bounds(joint, bounds)
    
    # Get robot and problem solver
    robot = planner.get_robot()
    ps = planner.get_problem_solver()
    
    # Configure path validation
    print("   Configuring path validation...")
    ps.selectPathValidation("Discretized", TaskConfig.PATH_VALIDATION_STEP)
    ps.selectPathProjector("Progressive", TaskConfig.PATH_PROJECTOR_STEP)
    
    print("   ✓ Scene setup complete")
    
    return planner, robot, ps


def main(visualize=True, solve=True):
    """
    Main execution function.
    
    Args:
        visualize: Whether to visualize the scene
        solve: Whether to solve the planning problem
    """
    if not HAS_CORBA:
        print("Error: CORBA backend not available")
        return False
    
    print("=" * 70)
    print("Spacelab Manipulation: UR10 Grasps Frame Gripper")
    print("=" * 70)
    
    # 1. Setup scene
    planner, robot, ps = setup_scene()
    joints = robot.jointNames
    for i, joint in enumerate(joints):
        rank = robot.rankInConfiguration[joint]
        print(f'{i:3d}. {joint} (config rank: {rank})')
    # 2. Create constraints
    print("\n2. Creating constraints...")
    create_transformation_constraints(ps)
    
    # 3. Create constraint graph
    print("\n3. Creating constraint graph...")
    graph = create_constraint_graph(robot, ps)
    
    # 4. Generate configurations
    print("\n4. Generating configurations...")
    q_init = build_initial_configuration()
    configs = generate_configurations(robot, graph, ps, q_init)
    
    # 5. Visualize
    if visualize:
        print("\n5. Starting visualization...")
        try:
            planner.visualize(configs.get("q_init", q_init))
            print("   ✓ Initial configuration displayed")
        except Exception as e:
            print(f"   ⚠ Visualization failed: {e}")
    
    # 6. Solve
    if solve and "q_goal" in configs:
        print("\n6. Solving planning problem...")
        ps.setInitialConfig(configs["q_init"])
        ps.addGoalConfig(configs["q_goal"])
        
        print("   This may take a while...")
        try:
            ps.solve()
            print("   ✓ Solution found!")
            
            if visualize:
                print("\n7. Playing solution path...")
                try:
                    planner.play_path(0)
                    print("   ✓ Path playback complete")
                except Exception as e:
                    print(f"   ⚠ Path playback failed: {e}")
        except Exception as e:
            print(f"   ⚠ Planning failed: {e}")
            return False
    
    print("\n" + "=" * 70)
    print("Setup Complete!")
    print("=" * 70)
    
    print("\nGenerated configurations:")
    for name, q in configs.items():
        print(f"  {name}: {len(q)} DOF")
    
    print("\nTo visualize configurations:")
    print("  planner.visualize(configs['q_init'])")
    print("  planner.visualize(configs['q_above'])")
    print("  planner.visualize(configs['q_goal'])")
    
    print("\nTo solve and play:")
    print("  ps.solve()")
    print("  planner.play_path(0)")
    
    return True, planner, robot, ps, graph, configs


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Spacelab manipulation: UR10 grasps frame_gripper"
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization"
    )
    parser.add_argument(
        "--no-solve",
        action="store_true",
        help="Skip solving (just setup)"
    )
    
    args = parser.parse_args()
    
    result = main(
        visualize=not args.no_viz,
        solve=not args.no_solve
    )
    
    if result and result is not True:
        success, planner, robot, ps, graph, configs = result
        
        print("\n" + "=" * 70)
        print("Objects available for interactive use:")
        print("=" * 70)
        print("  planner  - CorbaManipulationPlanner")
        print("  robot    - Robot instance")
        print("  ps       - ProblemSolver")
        print("  graph    - ConstraintGraph")
        print("  configs  - Generated configurations")
