#!/usr/bin/env python3
"""
Shared utilities and base classes for Spacelab manipulation tasks.

This module provides reusable components for building manipulation planning tasks:
- SpaceLabSceneBuilder: Set up robots, objects, and environment
- TaskConstraintGraph: Base class for constraint graph creation
- ConfigurationGenerator: Generate and validate configurations
- Utilities for constraint creation and collision management
"""

import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
import numpy as np

# Add config directory to path
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import (
    InitialConfigurations,
    RobotJoints,
    JointBounds,
    ManipulationConfig,
)

from agimus_spacelab.utils import xyzrpy_to_xyzquat, xyzquat_to_se3

# Import PyHPP constraint types (optional, for dual backend support)
try:
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, StdVec_Bool as Mask
    HAS_PYHPP_CONSTRAINTS = True
except ImportError:
    HAS_PYHPP_CONSTRAINTS = False
    RelativeTransformation = None
    Transformation = None
    ComparisonTypes = None
    ComparisonType = None
    Implicit = None
    SE3 = None
    Mask = None

# Import unified planner interfaces
try:
    from agimus_spacelab.corba import (
        CorbaManipulationPlanner,
        HAS_CORBA,
    )
except ImportError:
    HAS_CORBA = False
    CorbaManipulationPlanner = None

try:
    from agimus_spacelab.pyhpp import (
        PyHPPManipulationPlanner,
        HAS_PYHPP,
    )
except ImportError:
    HAS_PYHPP = False
    PyHPPManipulationPlanner = None

# Import ConstraintGraph for type hints (CORBA only)
try:
    from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
except ImportError:
    ConstraintGraph = None
    Constraints = None


# ============================================================================
# Scene Setup
# ============================================================================

class SpaceLabSceneBuilder:
    """
    Builder class for setting up Spacelab scenes with robots and objects.
    
    Handles loading robots, environment, objects, and configuring collision checking.
    """
    
    # Default URDF paths
    DEFAULT_PATHS = {
        "robot_urdf": "package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf",
        "robot_srdf": "package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf",
        "environment": "package://spacelab_mock_hardware/description/urdf/ground_demo.urdf",
        "objects": {
            "RS1": "package://spacelab_mock_hardware/description/urdf/RS1.urdf",
            "RS2": "package://spacelab_mock_hardware/description/urdf/RS2.urdf",
            "RS3": "package://spacelab_mock_hardware/description/urdf/RS3.urdf",
            "RS4": "package://spacelab_mock_hardware/description/urdf/RS4.urdf",
            "RS5": "package://spacelab_mock_hardware/description/urdf/RS5.urdf",
            "RS6": "package://spacelab_mock_hardware/description/urdf/RS6.urdf",
            "screw_driver": "package://spacelab_mock_hardware/description/urdf/screw_driver.urdf",
            "frame_gripper": "package://spacelab_mock_hardware/description/urdf/frame_gripper.urdf",
            "cleat_gripper": "package://spacelab_mock_hardware/description/urdf/cleat_gripper.urdf",
        }
    }
    
    def __init__(self, planner: Optional[Any] = None,
                 backend: str = "corba"):
        """
        Initialize scene builder.
        
        Args:
            planner: Existing planner instance, or None to create new one
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.backend = backend.lower()
        self.loaded_objects = []
        
        if self.backend == "corba":
            if not HAS_CORBA:
                raise ImportError("CORBA backend not available")
            self.planner = planner or CorbaManipulationPlanner()
        elif self.backend == "pyhpp":
            if not HAS_PYHPP:
                raise ImportError("PyHPP backend not available")
            self.planner = planner or PyHPPManipulationPlanner()
        else:
            raise ValueError(f"Unknown backend: {backend}. Use 'corba' or 'pyhpp'")
        
    def load_robot(self, composite_name: str = "spacelab",
                   robot_name: str = "spacelab") -> 'SpaceLabSceneBuilder':
        """Load the composite robot (UR10 + VISPA)."""
        print("   Loading robot...")
        if self.backend == "corba":
            self.planner.load_robot(
                composite_name=composite_name,
                robot_name=robot_name,
                urdf_path=self.DEFAULT_PATHS["robot_urdf"],
                srdf_path=self.DEFAULT_PATHS["robot_srdf"],
            )
        else:  # pyhpp
            self.planner.load_robot(
                name=robot_name,
                urdf_path=self.DEFAULT_PATHS["robot_urdf"],
                srdf_path=self.DEFAULT_PATHS["robot_srdf"],
                root_joint_type="anchor"
            )
        return self
        
    def load_environment(self, name: str = "ground_demo") -> 'SpaceLabSceneBuilder':
        """Load the environment (dispenser, ground, etc.)."""
        print("   Loading environment...")
        self.planner.load_environment(
            name=name,
            urdf_path=self.DEFAULT_PATHS["environment"]
        )
        return self
        
    def load_objects(self, object_names: List[str]) -> 'SpaceLabSceneBuilder':
        """
        Load multiple objects.
        
        Args:
            object_names: List of object names to load
        """
        print(f"   Loading {len(object_names)} object(s)...")
        for obj_name in object_names:
            if obj_name not in self.DEFAULT_PATHS["objects"]:
                print(f"      ⚠ Unknown object: {obj_name}")
                continue
            
            self.planner.load_object(
                name=obj_name,
                urdf_path=self.DEFAULT_PATHS["objects"][obj_name],
                root_joint_type="freeflyer"
            )
            self.loaded_objects.append(obj_name)
            
        return self
        
    def set_joint_bounds(self) -> 'SpaceLabSceneBuilder':
        """Set joint bounds for all loaded freeflyer objects."""
        print("   Setting joint bounds...")
        bounds = JointBounds.freeflyer_bounds()
        
        for obj_name in self.loaded_objects:
            joint_name = f"{obj_name}/root_joint"
            self.planner.set_joint_bounds(joint_name, bounds)
            
        return self
        
    def configure_path_validation(self,
                                   validation_step: float = 0.01,
                                   projector_step: float = 0.1) -> 'SpaceLabSceneBuilder':
        """Configure path validation parameters."""
        print("   Configuring path validation...")
        if self.backend == "corba":
            ps = self.planner.get_problem_solver()
            ps.selectPathValidation("Discretized", validation_step)
            ps.selectPathProjector("Progressive", projector_step)
        else:  # pyhpp
            # PyHPP uses dichotomy and progressive projector
            self.planner.set_dichotomy(True)
            self.planner.set_progressive_projector(True)
        return self
        
    def disable_collision_pair(self,
                               obstacle_name: str,
                               joint_name: str,
                               remove_collision: bool = True,
                               remove_distance: bool = False) -> 'SpaceLabSceneBuilder':
        """
        Disable collision checking for a specific obstacle-joint pair.
        
        Args:
            obstacle_name: Name of the obstacle body
            joint_name: Name of the joint
            remove_collision: Remove from collision checking
            remove_distance: Remove from distance checking
        """
        print(f"   Disabling collision: {obstacle_name} <-> {joint_name}")
        if self.backend == "corba":
            ps = self.planner.get_problem_solver()
            ps.removeObstacleFromJoint(
                obstacle_name,
                joint_name,
                remove_collision,
                remove_distance
            )
        else:
            # PyHPP collision management is handled differently
            # Would need to use device collision pairs API
            print("      (PyHPP: collision management not yet implemented)")
        return self
        
    def get_instances(self) -> Tuple[Any, Any, Any]:
        """
        Get planner, robot, and problem solver instances.
        
        Returns:
            Tuple of (planner, robot, ps/problem)
        """
        robot = self.planner.get_robot()
        if self.backend == "corba":
            ps = self.planner.get_problem_solver()
        else:  # pyhpp
            ps = self.planner.get_problem()
        return self.planner, robot, ps
        
    def build(self, 
              objects: List[str],
              validation_step: float = 0.01,
              projector_step: float = 0.1) -> Tuple[Any, Any, Any]:
        """
        Complete scene setup with default configuration.
        
        Args:
            objects: List of object names to load
            validation_step: Path validation discretization step
            projector_step: Path projector step
            
        Returns:
            Tuple of (planner, robot, ps)
        """
        print("\n1. Setting up scene...")
        (self.load_robot()
            .load_environment()
            .load_objects(objects)
            .set_joint_bounds()
            .configure_path_validation(validation_step, projector_step))
        
        print("   ✓ Scene setup complete")
        return self.get_instances()


# ============================================================================
# Constraint Utilities
# ============================================================================

class ConstraintBuilder:
    """Helper class for creating transformation constraints with dual backend support."""
    
    @staticmethod
    def create_grasp_constraint(ps, name: str, gripper: str, tool: str,
                                transform: List[float], 
                                mask: List[bool] = None,
                                robot=None, backend: str = "corba") -> Any:
        """
        Create a grasp constraint (rigid attachment).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            name: Constraint name
            gripper: Gripper joint/frame name
            tool: Tool joint/frame name
            transform: [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for DOF constraints (default: all True)
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        if mask is None:
            mask = [True] * 6
        
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint IDs
            joint_gripper = robot.model().getJointId(gripper)
            joint_tool = robot.model().getJointId(tool)
            
            # Convert transform to SE3
            grasp_tf = xyzquat_to_se3(transform)
            
            # Create mask
            mask_vec = Mask()
            mask_vec[:] = tuple(mask)
            
            # Create relative transformation constraint
            pc = RelativeTransformation.create(
                name, robot.asPinDevice(),
                joint_gripper, joint_tool,
                grasp_tf, SE3.Identity(), mask_vec
            )
            
            # Create comparison types (all EqualToZero for grasp)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * sum(mask))
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, mask_vec)
            print(f"    ✓ {name}: {gripper} -> {tool} (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                name, gripper, tool, transform, mask
            )
            print(f"    ✓ {name}: {gripper} -> {tool}")
            return None
        
    @staticmethod
    def create_placement_constraint(ps, name: str, tool: str,
                                    world_pose: List[float],
                                    mask: List[bool],
                                    robot=None, backend: str = "corba") -> Any:
        """
        Create a placement constraint (object on surface).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            name: Constraint name
            tool: Tool joint name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for DOF constraints
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint ID
            joint_tool = robot.model().getJointId(tool)
            
            # Convert pose to SE3
            placement_tf = xyzquat_to_se3(world_pose)
            
            # Create transformation constraint
            pc = Transformation.create(
                name, robot.asPinDevice(),
                joint_tool, SE3.Identity(), placement_tf, mask
            )
            
            # Create comparison types (EqualToZero for placement)
            num_constrained = sum(mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * num_constrained)
            implicit_mask = [True] * num_constrained
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, implicit_mask)
            print(f"    ✓ {name}: tool at {world_pose[:3]} (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                name, "", tool, world_pose, mask
            )
            print(f"    ✓ {name}: tool at {world_pose[:3]}")
            return None
        
    @staticmethod
    def create_complement_constraint(ps, base_name: str, tool: str,
                                     world_pose: List[float],
                                     complement_mask: List[bool],
                                     robot=None, backend: str = "corba") -> Any:
        """
        Create complement constraint (free DOFs).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            base_name: Base constraint name
            tool: Tool joint name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            complement_mask: Boolean mask for complement DOFs
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        constraint_name = f"{base_name}/complement"
        
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint ID
            joint_tool = robot.model().getJointId(tool)
            
            # Convert pose to SE3
            placement_tf = xyzquat_to_se3(world_pose)
            
            # Create transformation constraint
            pc = Transformation.create(
                constraint_name, robot.asPinDevice(),
                joint_tool, SE3.Identity(), placement_tf, complement_mask
            )
            
            # Complement uses Equality comparison type
            num_constrained = sum(complement_mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.Equality] * num_constrained)
            implicit_mask = [True] * num_constrained
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, implicit_mask)
            print(f"    ✓ {constraint_name}: free DOFs (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                constraint_name, "", tool, world_pose, complement_mask
            )
            print(f"    ✓ {constraint_name}: free DOFs")
            return None


# ============================================================================
# Configuration Management
# ============================================================================

class ConfigurationGenerator:
    """
    Generate and validate configurations for manipulation tasks.
    
    Handles projection, random sampling, and waypoint generation.
    """
    
    def __init__(self, robot, graph, ps, backend: str = "corba",
                 max_attempts: int = 1000):
        """
        Initialize configuration generator.
        
        Args:
            robot: Robot instance
            graph: ConstraintGraph or Graph instance
            ps: ProblemSolver or Problem instance
            backend: "corba" or "pyhpp"
            max_attempts: Maximum random sampling attempts
        """
        self.robot = robot
        self.graph = graph
        self.ps = ps
        self.backend = backend.lower()
        self.max_attempts = max_attempts
        self.configs = {}
        
    def project_on_node(self, node_name: str, q: List[float],
                        config_label: Optional[str] = None) -> Tuple[bool, List[float]]:
        """
        Project configuration onto node constraints.
        
        Args:
            node_name: Name of the graph node
            q: Configuration to project
            config_label: Optional label to store result
            
        Returns:
            Tuple of (success, projected_config)
        """
        if self.backend == "corba":
            res, q_proj, err = self.graph.applyNodeConstraints(node_name, list(q))
        else:  # pyhpp
            res, q_proj, err = self.graph.applyConstraints(node_name, list(q))
        
        if config_label:
            if res:
                self.configs[config_label] = q_proj
                print(f"       ✓ {config_label} projected onto '{node_name}'")
            else:
                self.configs[config_label] = list(q)
                print(f"       ⚠ Projection failed (error: {err:.3f}), using input")
                
        return res, q_proj if res else list(q)
        
    def generate_via_edge(self, edge_name: str, q_from: List[float],
                          config_label: Optional[str] = None,
                          verbose: bool = True) -> Tuple[bool, Optional[List[float]]]:
        """
        Generate target configuration by shooting random configs along edge.
        
        Args:
            edge_name: Name of the edge
            q_from: Source configuration
            config_label: Optional label to store result
            verbose: Print progress every 200 attempts
            
        Returns:
            Tuple of (success, generated_config or None)
        """
        for i in range(self.max_attempts):
            # Generate random config - API is same for both backends
            if self.backend == "corba":
                q_rand = self.robot.shootRandomConfig()
            else:  # pyhpp
                import numpy as np
                q_rand = self.robot.randomConfiguration()
                if isinstance(q_rand, np.ndarray):
                    q_rand = q_rand.tolist()
            
            res, q_target, err = self.graph.generateTargetConfig(
                edge_name, q_from, q_rand
            )
            if res:
                if config_label:
                    self.configs[config_label] = q_target
                    print(f"       ✓ {config_label} generated via '{edge_name}'")
                return True, q_target
                
            if verbose and (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}/{self.max_attempts}...")
                
        if config_label:
            print(f"       ⚠ Failed after {self.max_attempts} attempts")
        return False, None
        
    def build_robot_config(self, ur10_angles: List[float] = None,
                           vispa_base: List[float] = None,
                           vispa_arm: List[float] = None) -> List[float]:
        """
        Build robot configuration from joint angles.
        
        Args:
            ur10_angles: UR10 joint angles (default: from InitialConfigurations)
            vispa_base: VISPA base config (default: from InitialConfigurations)
            vispa_arm: VISPA arm config (default: from InitialConfigurations)
            
        Returns:
            Combined robot configuration
        """
        ur10 = ur10_angles if ur10_angles else InitialConfigurations.UR10
        vb = vispa_base if vispa_base else InitialConfigurations.VISPA_BASE
        va = vispa_arm if vispa_arm else InitialConfigurations.VISPA_ARM
        
        return list(ur10) + list(vb) + list(va)
        
    def build_object_configs(self, object_names: List[str]) -> List[float]:
        """
        Build object configurations from initial poses.
        
        Args:
            object_names: List of object names
            
        Returns:
            Concatenated object configurations in XYZQUAT format
        """
        q_objects = []
        
        for obj_name in object_names:
            # Get initial pose in XYZRPY
            obj_attr = obj_name.replace("-", "_").replace(" ", "_").upper()
            if hasattr(InitialConfigurations, obj_attr):
                pose_xyzrpy = getattr(InitialConfigurations, obj_attr)
                pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
                q_objects.extend(pose_xyzquat.tolist())
            else:
                print(f"       ⚠ No initial config for {obj_name}")
                # Add zero configuration as fallback
                q_objects.extend([0.0] * 7)
                
        return q_objects
        
    def get_robot_dof(self) -> int:
        """Get total robot DOF (UR10 + VISPA_BASE + VISPA_ARM)."""
        return len(InitialConfigurations.UR10 + 
                   InitialConfigurations.VISPA_BASE + 
                   InitialConfigurations.VISPA_ARM)
        
    def modify_object_pose(self, q: List[float], object_index: int,
                           translation_delta: Optional[List[float]] = None,
                           quaternion: Optional[List[float]] = None) -> List[float]:
        """
        Modify object pose in configuration.
        
        Args:
            q: Full configuration
            object_index: Index of object (0 for first object after robot)
            translation_delta: [dx, dy, dz] to add to position
            quaternion: New quaternion [qx, qy, qz, qw] (replaces existing)
            
        Returns:
            Modified configuration
        """
        q_new = list(q)
        robot_dof = self.get_robot_dof()
        obj_start = robot_dof + object_index * 7
        
        if translation_delta:
            for i in range(3):
                q_new[obj_start + i] += translation_delta[i]
                
        if quaternion:
            for i in range(4):
                q_new[obj_start + 3 + i] = quaternion[i]
                
        return q_new


# ============================================================================
# Base Task Class
# ============================================================================

class ManipulationTask:
    """
    Base class for manipulation tasks.
    
    Provides common structure for task definition, constraint creation,
    graph building, and configuration management.
    """
    
    def __init__(self, task_name: str, backend: str = "corba"):
        """
        Initialize manipulation task.
        
        Args:
            task_name: Descriptive name for the task
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.task_name = task_name
        self.backend = backend.lower()
        self.scene_builder = SpaceLabSceneBuilder(backend=backend)
        self.planner = None
        self.robot = None
        self.ps = None
        self.graph = None
        self.config_gen = None
        
    def get_objects(self) -> List[str]:
        """
        Define which objects are needed for this task.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement get_objects()")
        
    def create_constraints(self) -> None:
        """
        Create all transformation constraints for the task.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement create_constraints()")
        
    def create_graph(self) -> ConstraintGraph:
        """
        Create and configure the constraint graph.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement create_graph()")
        
    def setup_collision_management(self) -> None:
        """
        Configure collision checking (disable expected contacts, etc.).
        Override in subclass if needed.
        """
        pass
        
    def build_initial_config(self) -> List[float]:
        """
        Build the initial configuration.
        Override in subclass if custom logic needed.
        """
        objects = self.get_objects()
        q_robot = self.config_gen.build_robot_config()
        q_objects = self.config_gen.build_object_configs(objects)
        return q_robot + q_objects
        
    def generate_configurations(self, q_init: List[float]) -> Dict[str, List[float]]:
        """
        Generate all intermediate configurations.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement generate_configurations()")
        
    def setup(self, validation_step: float = 0.01, projector_step: float = 0.1):
        """
        Complete task setup: scene, constraints, graph.
        
        Args:
            validation_step: Path validation discretization
            projector_step: Path projector step
        """
        print("=" * 70)
        print(f"{self.task_name}")
        print("=" * 70)
        
        # 1. Scene setup
        self.planner, self.robot, self.ps = self.scene_builder.build(
            objects=self.get_objects(),
            validation_step=validation_step,
            projector_step=projector_step
        )
        
        # 2. Custom collision management
        self.setup_collision_management()
        
        # 3. Create constraints
        print("\n2. Creating constraints...")
        self.create_constraints()
        
        # 4. Create graph
        print("\n3. Creating constraint graph...")
        self.graph = self.create_graph()
        
        # 5. Initialize configuration generator
        self.config_gen = ConfigurationGenerator(
            self.robot, self.graph, self.ps, backend=self.backend
        )
        
        print("\n   ✓ Task setup complete")
        
    def run(self, visualize: bool = True, solve: bool = False) -> Dict[str, Any]:
        """
        Run the complete task workflow.
        
        Args:
            visualize: Whether to visualize configurations
            solve: Whether to solve the planning problem
            
        Returns:
            Dictionary with configs, paths, and other results
        """
        if not self.graph:
            raise RuntimeError("Must call setup() before run()")
            
        # 4. Generate configurations
        print("\n4. Generating configurations...")
        q_init = self.build_initial_config()
        configs = self.generate_configurations(q_init)
        
        # 5. Visualize
        if visualize:
            print("\n5. Starting visualization...")
            try:
                self.planner.visualize(configs.get("q_init", q_init))
                print("   ✓ Initial configuration displayed")
            except Exception as e:
                print(f"   ⚠ Visualization failed: {e}")
                
        # 6. Solve
        if solve and "q_goal" in configs:
            print("\n6. Solving planning problem...")
            self.ps.setInitialConfig(configs["q_init"])
            self.ps.addGoalConfig(configs["q_goal"])
            
            try:
                self.ps.solve()
                print("   ✓ Solution found!")
                
                if visualize:
                    print("\n7. Playing solution path...")
                    try:
                        self.planner.play_path(0)
                        print("   ✓ Path playback complete")
                    except Exception as e:
                        print(f"   ⚠ Path playback failed: {e}")
            except Exception as e:
                print(f"   ⚠ Planning failed: {e}")
                
        return {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
        }


# ============================================================================
# Utility Functions
# ============================================================================

def print_joint_info(robot):
    """Print all joints with their configuration ranks."""
    print("\nJoint Information:")
    joints = robot.jointNames
    for i, joint in enumerate(joints):
        rank = robot.rankInConfiguration[joint]
        print(f"  {i:3d}. {joint} (config rank: {rank})")


def visualize_handle_frames(robot, planner, q, handle_names: List[str]):
    """
    Add visualization for handle frames and approaching directions.
    
    Args:
        robot: Robot instance
        planner: Planner with viewer
        q: Configuration to visualize
        handle_names: List of handle names to visualize
    """
    try:
        import numpy as np
        from pinocchio import SE3, Quaternion
        
        v = planner.viewer if hasattr(planner, 'viewer') else None
        if not v:
            print("   ⚠ No viewer available")
            return
            
        robot.setCurrentConfig(q)
        
        for handle_name in handle_names:
            try:
                # Get handle info
                approach_dir = list(robot.getHandleApproachingDirection(handle_name))
                handle_pos = robot.getHandlePositionInJoint(handle_name)
                
                # Add frame visualization
                safe_name = handle_name.replace('/', '_')
                frame_name = f"hpp-gui/{safe_name}_frame"
                arrow_name = f"hpp-gui/{safe_name}_approach"
                
                v.client.gui.addXYZaxis(frame_name, [0, 0.8, 0, 1], 0.008, 0.1)
                print(f"  Added frame: {handle_name} (approach: {approach_dir})")
                
            except Exception as e:
                print(f"  ⚠ Failed to add frame for {handle_name}: {e}")
                
    except ImportError:
        print("   ⚠ Visualization requires pinocchio")


def visualize_constraint_graph(
    graph,
    output_path: str = "constraint_graph",
    include_subgraph: bool = True,
    show_png: bool = False,
    states_dict=None,
    edges_dict=None,
    edge_topology=None
):
    """
    Visualize constraint graph structure using NetworkX and Graphviz.

    Creates a visual representation of the constraint graph nodes and edges,
    with optional PNG generation and display.

    Args:
        graph: ConstraintGraph instance (CORBA) or Graph instance (PyHPP)
        output_path: Base path for output files (without extension)
        include_subgraph: Include subgraph details if available
        show_png: If True, attempt to open the PNG after generation
        states_dict: Optional dict of states (for PyHPP backend)
        edges_dict: Optional dict of edges (for PyHPP backend)
        edge_topology: Optional dict mapping edge names to (from, to) tuples

    Returns:
        Path to the generated PNG file if successful, None otherwise
    """
    try:
        import networkx as nx
        import matplotlib.pyplot as plt
        import warnings
        warnings.filterwarnings('ignore', category=DeprecationWarning)

        # Detect backend type and get graph structure accordingly
        is_pyhpp = hasattr(graph, 'getStates')  # PyHPP Graph has getStates()
        
        if is_pyhpp:
            # PyHPP backend - use provided dictionaries if available
            if states_dict is None or edges_dict is None:
                print("  ⚠ PyHPP graph requires states_dict and edges_dict parameters")
                return None
            nodes = list(states_dict.keys())
            edges = list(edges_dict.keys())
        else:
            # CORBA backend - use nodes and edges dictionaries
            nodes = list(graph.nodes.keys())
            edges = list(graph.edges.keys())

        print("\n📊 Constraint Graph Structure:")
        print(f"  Nodes: {len(nodes)}")
        print(f"  Edges: {len(edges)}")

        # Create directed graph
        G = nx.DiGraph()

        # Add nodes with labels
        for node in nodes:
            G.add_node(node, label=node)

        # Add edges with labels
        edge_labels = {}
        for edge_name in edges:
            try:
                if is_pyhpp:
                    # PyHPP: use topology dictionary if available
                    if edge_topology and edge_name in edge_topology:
                        from_node, to_node = edge_topology[edge_name]
                    else:
                        print(f"  ⚠ No topology for edge '{edge_name}'")
                        continue
                else:
                    # CORBA: use getNodesConnectedByEdge
                    edge_info = graph.getNodesConnectedByEdge(edge_name)
                    from_node = edge_info[0]
                    to_node = edge_info[1]
                
                G.add_edge(from_node, to_node)
                edge_labels[(from_node, to_node)] = edge_name
            except Exception as e:
                print(f"  ⚠ Could not get info for edge '{edge_name}': {e}")

        # Set up the plot
        fig, ax = plt.subplots(figsize=(14, 10))

        # Use hierarchical layout for better readability
        try:
            pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
        except Exception:
            pos = nx.spring_layout(G, seed=42)

        # Draw nodes
        node_colors = [
            'lightblue' if 'free' in node.lower() else 'lightgreen'
            for node in G.nodes()
        ]
        nx.draw_networkx_nodes(
            G, pos, node_color=node_colors,
            node_size=3000, alpha=0.9, ax=ax
        )

        # Draw node labels
        nx.draw_networkx_labels(G, pos, font_size=9, font_weight='bold', ax=ax)

        # Draw edges
        nx.draw_networkx_edges(
            G, pos, edge_color='gray',
            arrows=True, arrowsize=20,
            arrowstyle='->', width=2, ax=ax
        )

        # Draw edge labels
        nx.draw_networkx_edge_labels(G, pos, edge_labels, font_size=7, ax=ax)

        # Add title and info
        title = "Constraint Graph Visualization"
        if include_subgraph:
            title += f"\n{len(nodes)} nodes, {len(edges)} edges"
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
        ax.axis('off')

        plt.tight_layout()

        # Save to file
        png_path = f"{output_path}.png"
        plt.savefig(png_path, dpi=150, bbox_inches='tight')
        print(f"  ✓ Saved graph visualization to: {png_path}")

        # Print node/edge details
        print("  Nodes:")
        for node in nodes:
            print(f"    • {node}")

        print("  Edges:")
        for from_node, to_node in G.edges():
            edge_name = edge_labels.get((from_node, to_node), "?")
            print(f"    • {edge_name}: {from_node} → {to_node}")

        # Optionally display
        if show_png:
            try:
                plt.show()
            except Exception:
                print("  ⚠ Could not display PNG (no display available)")
        else:
            plt.close()

        return png_path

    except ImportError as e:
        print(f"  ⚠ Visualization requires networkx and matplotlib: {e}")
        print("  Install with: pip install networkx matplotlib")
        return None
    except Exception as e:
        print(f"  ⚠ Failed to visualize graph: {e}")
        import traceback
        traceback.print_exc()
        return None


__all__ = [
    "SpaceLabSceneBuilder",
    "ConstraintBuilder",
    "ConfigurationGenerator",
    "ManipulationTask",
    "print_joint_info",
    "visualize_handle_frames",
]
