#!/usr/bin/env python3
"""
Task: UR10 grasps frame_gripper from dispenser.

This task demonstrates a complete pick-and-place sequence:
1. Tool rests on dispenser (placement)
2. Gripper approaches tool (gripper-above-tool)
3. Gripper grasps tool on dispenser (grasp-placement)
4. Tool is lifted from dispenser (tool-in-air)
5. Free motion with grasped tool (grasp)

Task features:
- Surface contact modeling (tool on dispenser)
- Complement constraints for sliding motion
- Security margins for expected contacts
- Multi-waypoint configuration generation
"""

import sys
from pathlib import Path
from typing import List, Dict

# Add script directory to path
sys.path.insert(0, str(Path(__file__).parent))

from spacelab_tools import (
    ManipulationTask,
    ConstraintBuilder,
    print_joint_info,
    visualize_constraint_graph,
)

# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import InitialConfigurations
from agimus_spacelab.utils import xyzrpy_to_xyzquat

# Import backend availability flags for validation
try:
    from agimus_spacelab.corba import HAS_CORBA
except ImportError:
    HAS_CORBA = False

try:
    from agimus_spacelab.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

# CORBA-specific imports for manual graph building (when not using factory)
try:
    from hpp.corbaserver.manipulation import (
        ConstraintGraph,
        ConstraintGraphFactory,
        Constraints,
        Rule,
    )
except ImportError:
    ConstraintGraph = None
    ConstraintGraphFactory = None
    Constraints = None
    Rule = None

# PyHPP-specific imports for manual graph building
try:
    from pyhpp.manipulation import Graph
    from pyhpp.manipulation.constraint_graph_factory import (
        ConstraintGraphFactory as PyHPPConstraintGraphFactory,
        Rule as PyHPPRule
    )
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, Quaternion, StdVec_Bool as Mask
    import numpy as np
except ImportError:
    Graph = None
    PyHPPConstraintGraphFactory = None
    PyHPPRule = None
    RelativeTransformation = None
    Transformation = None


# ============================================================================
# Task Configuration
# ============================================================================

class GraspFrameGripperConfig:
    """Configuration for UR10 grasping frame_gripper task."""
    
    # Gripper and object names
    GRIPPER_NAME = "spacelab/ur10_joint_6_7"
    TOOL_NAME = "frame_gripper/root_joint"
    
    # Grasp transform (tool in gripper frame)
    TOOL_IN_GRIPPER = [0.0, 0.0, 0.1, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
    
    # Pre-grasp transform (gripper above tool)
    GRIPPER_ABOVE_TOOL = [0.0, 0.0, 0.2, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
    
    # Constraint masks
    GRASP_MASK = [True, True, True, True, True, True]  # All DOF fixed
    PLACEMENT_MASK = [False, False, True, True, True, True]  # Z + rotations fixed
    PLACEMENT_COMPLEMENT_MASK = [True, True, False, False, False, False]  # X, Y free
    
    # Graph nodes (states)
    GRAPH_NODES = [
        "grasp",               # Tool grasped, free motion
        "tool-in-air",         # Tool grasped and lifted
        "grasp-placement",     # Tool grasped while on dispenser
        "gripper-above-tool",  # Gripper aligned above tool
        "placement",           # Tool on dispenser, gripper free
    ]
    
    # Collision management
    TOOL_CONTACT_JOINT = "frame_gripper/root_joint"
    DISPENSER_CONTACT_JOINT = "universe"
    CONTACT_MARGIN = -0.02  # Allow 2cm penetration for surface contact
    
    # Edges with surface contact
    PLACEMENT_EDGES = [
        "transit", "approach-tool", "move-gripper-away",
        "grasp-tool", "release-tool", "lift-tool", "lower-tool",
    ]
    
    # Path planning parameters
    PATH_VALIDATION_STEP = 0.01
    PATH_PROJECTOR_STEP = 0.1
    MAX_RANDOM_ATTEMPTS = 1000
    
    # Tool poses (computed from initial config)
    TOOL_ON_DISPENSER = None
    TOOL_IN_AIR = None
    
    @classmethod
    def init_poses(cls):
        """Initialize tool poses from configuration."""
        tool_pose_xyzrpy = InitialConfigurations.FRAME_GRIPPER
        tool_pose_quat = xyzrpy_to_xyzquat(tool_pose_xyzrpy)
        
        cls.TOOL_ON_DISPENSER = tool_pose_quat.tolist()
        
        # Lifted position
        cls.TOOL_IN_AIR = tool_pose_quat.copy()
        cls.TOOL_IN_AIR[2] += 0.15  # Lift 15cm
        cls.TOOL_IN_AIR = cls.TOOL_IN_AIR.tolist()


# Initialize poses
GraspFrameGripperConfig.init_poses()


# ============================================================================
# Task Implementation
# ============================================================================

class GraspFrameGripperTask(ManipulationTask):
    """UR10 grasps frame_gripper from dispenser."""
    
    def __init__(self, use_factory: bool = False, backend: str = "corba"):
        """
        Initialize task.
        
        Args:
            use_factory: If True, use ConstraintGraphFactory for automatic
                        graph generation. If False, build graph manually.
            backend: "corba" or "pyhpp" - which backend to use
        """
        super().__init__(
            "Spacelab Manipulation: UR10 Grasps Frame Gripper",
            backend=backend
        )
        self.config = GraspFrameGripperConfig
        self.use_factory = use_factory
        
        # Validate CORBA-specific imports for manual graph building
        if self.backend == "corba" and not self.use_factory:
            if ConstraintGraph is None:
                raise ImportError(
                    "CORBA graph classes not available. "
                    "Either use --factory mode or install hpp.corbaserver"
                )
        
    def get_objects(self) -> List[str]:
        """Only need frame_gripper for this task."""
        return ["frame_gripper"]
        
    def setup_collision_management(self) -> None:
        """Disable collision between tool and dispenser surface."""
        self.scene_builder.disable_collision_pair(
            "ground_demo/link_TD_0",  # Dispenser surface
            self.config.TOOL_CONTACT_JOINT,
            remove_collision=True,
            remove_distance=False
        )
        
    def _create_pyhpp_constraints(self) -> Dict:
        """Create constraints using PyHPP APIs."""
        constraints = {}
        robot = self.robot
        cfg = self.config
        
        # Get joint IDs
        joint_gripper = robot.model().getJointId(cfg.GRIPPER_NAME)
        joint_tool = robot.model().getJointId(cfg.TOOL_NAME)
        Id = SE3.Identity()
        
        # Helper to convert [x,y,z,qx,qy,qz,qw] to SE3
        def pose_to_se3(pose):
            return SE3(
                Quaternion(pose[6], pose[3], pose[4], pose[5]),
                np.array(pose[:3])
            )
        
        # 1. Grasp constraint (gripper holds tool)
        grasp_tf = pose_to_se3(cfg.TOOL_IN_GRIPPER)
        mask_grasp = Mask()
        mask_grasp[:] = tuple(cfg.GRASP_MASK)
        
        pc_grasp = RelativeTransformation.create(
            'grasp', robot.asPinDevice(),
            joint_gripper, joint_tool,
            grasp_tf, Id, mask_grasp
        )
        cts_grasp = ComparisonTypes()
        cts_grasp[:] = tuple([ComparisonType.EqualToZero] * 6)
        constraints['grasp'] = Implicit.create(
            pc_grasp, cts_grasp, mask_grasp
        )
        
        # 2. Placement constraint (tool on dispenser)
        placement_tf = pose_to_se3(cfg.TOOL_ON_DISPENSER)
        mask_place = cfg.PLACEMENT_MASK
        
        pc_place = Transformation.create(
            'placement', robot.asPinDevice(),
            joint_tool, Id, placement_tf, mask_place
        )
        cts_place = ComparisonTypes()
        cts_place[:] = tuple([ComparisonType.EqualToZero] * 4)
        implicit_mask_place = [True] * 4
        constraints['placement'] = Implicit.create(
            pc_place, cts_place, implicit_mask_place
        )
        
        # 3. Placement complement (X, Y free)
        mask_place_comp = cfg.PLACEMENT_COMPLEMENT_MASK
        
        pc_place_comp = Transformation.create(
            'placement/complement', robot.asPinDevice(),
            joint_tool, Id, placement_tf, mask_place_comp
        )
        cts_place_comp = ComparisonTypes()
        cts_place_comp[:] = tuple([ComparisonType.Equality] * 2)
        implicit_mask_comp = [True] * 2
        constraints['placement/complement'] = Implicit.create(
            pc_place_comp, cts_place_comp, implicit_mask_comp
        )
        
        # 4. Gripper-tool aligned (gripper above tool)
        aligned_tf = pose_to_se3(cfg.GRIPPER_ABOVE_TOOL)
        mask_aligned = Mask()
        mask_aligned[:] = tuple(cfg.GRASP_MASK)
        
        pc_aligned = RelativeTransformation.create(
            'gripper_tool_aligned', robot.asPinDevice(),
            joint_gripper, joint_tool,
            aligned_tf, Id, mask_aligned
        )
        cts_aligned = ComparisonTypes()
        cts_aligned[:] = tuple([ComparisonType.EqualToZero] * 6)
        constraints['gripper_tool_aligned'] = Implicit.create(
            pc_aligned, cts_aligned, mask_aligned
        )
        
        # 5. Tool in air constraint
        tool_air_tf = pose_to_se3(cfg.TOOL_IN_AIR)
        mask_air = cfg.PLACEMENT_MASK
        
        pc_air = Transformation.create(
            'tool_in_air', robot.asPinDevice(),
            joint_tool, Id, tool_air_tf, mask_air
        )
        cts_air = ComparisonTypes()
        cts_air[:] = tuple([ComparisonType.EqualToZero] * 4)
        implicit_mask_air = [True] * 4
        constraints['tool_in_air'] = Implicit.create(
            pc_air, cts_air, implicit_mask_air
        )
        
        # 6. Tool in air complement
        mask_air_comp = cfg.PLACEMENT_COMPLEMENT_MASK
        
        pc_air_comp = Transformation.create(
            'tool_in_air/complement', robot.asPinDevice(),
            joint_tool, Id, tool_air_tf, mask_air_comp
        )
        cts_air_comp = ComparisonTypes()
        cts_air_comp[:] = tuple([ComparisonType.Equality] * 2)
        implicit_mask_air_comp = [True] * 2
        constraints['tool_in_air/complement'] = Implicit.create(
            pc_air_comp, cts_air_comp, implicit_mask_air_comp
        )
        
        print("    ✓ Created PyHPP constraints")
        return constraints
    
    def create_constraints(self) -> None:
        """Create all transformation constraints."""
        # Factory mode creates constraints automatically
        if self.use_factory:
            if self.backend == "pyhpp":
                # PyHPP factory needs placement constraints
                self.pyhpp_constraints = self._create_pyhpp_placement_constraints()
            else:
                print("    (Factory mode: constraints created automatically)")
            return
        
        # PyHPP uses different constraint creation
        if self.backend == "pyhpp":
            self.pyhpp_constraints = self._create_pyhpp_constraints()
            return
            
        cb = ConstraintBuilder
        cfg = self.config
        
        # 1. Grasp: gripper holds tool
        cb.create_grasp_constraint(
            self.ps, "grasp",
            cfg.GRIPPER_NAME, cfg.TOOL_NAME,
            cfg.TOOL_IN_GRIPPER, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 2. Placement: tool on dispenser (Z and rotations fixed)
        cb.create_placement_constraint(
            self.ps, "placement",
            cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 3. Placement complement: X, Y free (sliding on surface)
        cb.create_complement_constraint(
            self.ps, "placement",
            cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 4. Gripper-tool aligned: gripper above tool
        cb.create_grasp_constraint(
            self.ps, "gripper_tool_aligned",
            cfg.GRIPPER_NAME, cfg.TOOL_NAME,
            cfg.GRIPPER_ABOVE_TOOL, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 5. Tool in air: lifted position
        cb.create_placement_constraint(
            self.ps, "tool_in_air",
            cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 6. Tool in air complement
        cb.create_complement_constraint(
            self.ps, "tool_in_air",
            cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
    def create_graph(self):
        """Create and configure constraint graph."""
        if self.use_factory:
            return self._create_graph_with_factory()
        else:
            return self._create_graph_manual()
    
    def _create_graph_with_factory(self):
        """Create graph using ConstraintGraphFactory (automatic)."""
        print(
            "    Using ConstraintGraphFactory for automatic graph generation"
        )
        
        if self.backend == "pyhpp":
            return self._create_pyhpp_graph_with_factory()
        
        # CORBA backend
        graph = ConstraintGraph(self.robot, "graph")
        factory = ConstraintGraphFactory(graph)
        
        # Set grippers
        grippers = ["spacelab/g_ur10_tool"]
        factory.setGrippers(grippers)
        print(f"    ✓ Set grippers: {grippers}")
        
        # Set objects with handles and contact surfaces
        objects = ["frame_gripper"]
        handles_per_object = [["frame_gripper/h_FG_tool"]]
        contact_surfaces_per_object = [[]]  # No contact surfaces for now
        factory.setObjects(
            objects, handles_per_object, contact_surfaces_per_object
        )
        print(f"    ✓ Set objects: {objects}")
        
        # Set environment contacts (dispenser surface)
        env_contacts = ["ground_demo/pancake_table_table_top"]
        factory.environmentContacts(env_contacts)
        print(f"    ✓ Set environment contacts: {env_contacts}")
        
        # Set rules (allow all gripper-handle pairs)
        rules = [Rule([".*"], [".*"], True)]
        factory.setRules(rules)
        print("    ✓ Set rules: allow all")
        
        # Generate graph
        factory.generate()
        print("    ✓ Generated graph structure")
        
        # Initialize graph (factory creates edges automatically)
        graph.initialize()
        print("    ✓ Graph initialized")
        
        # Print factory-generated nodes for reference
        print(f"    ℹ Factory created {len(graph.nodes)} nodes:")
        for node_name in list(graph.nodes.keys())[:5]:  # Show first 5
            print(f"      - {node_name}")
        if len(graph.nodes) > 5:
            print(f"      ... and {len(graph.nodes) - 5} more")
        
        return graph
    
    def _create_pyhpp_graph_with_factory(self):
        """Create graph using PyHPP ConstraintGraphFactory."""
        print("    Using PyHPP ConstraintGraphFactory")
        
        # Create placement constraints for factory
        constraints = self.pyhpp_constraints
        
        # Configure handle masks (required by factory)
        try:
            handle_name = "frame_gripper/h_FG_tool"
            handle = self.robot.handles()[handle_name]
            # Set mask for handle (all DOF constrained)
            handle.mask = [True, True, True, True, True, True]
            print("    ✓ Configured handle mask")
        except (KeyError, Exception) as e:
            print(f"    ⚠ Could not configure handle mask: {e}")
            print("    Continuing without handle mask configuration...")
        
        # Create graph and factory
        graph = Graph("graph", self.robot, self.ps)
        graph.maxIterations(100)
        graph.errorThreshold(0.00001)
        
        # Register constraints
        graph.registerConstraints(
            constraints['place_frame_gripper'],
            constraints['place_frame_gripper/complement'],
            constraints['place_frame_gripper/hold']
        )
        
        factory = PyHPPConstraintGraphFactory(graph, constraints)
        
        # Set grippers
        grippers = ["spacelab/g_ur10_tool"]
        factory.setGrippers(grippers)
        print(f"    ✓ Set grippers: {grippers}")
        
        # Set objects with handles
        objects = ["frame_gripper"]
        handles_per_object = [["frame_gripper/h_FG_tool"]]
        contact_surfaces_per_object = [[]]
        factory.setObjects(
            objects, handles_per_object, contact_surfaces_per_object
        )
        print(f"    ✓ Set objects: {objects}")
        
        # Note: Not setting environment contacts because we don't use
        # contact surfaces. The placement constraint handles surface contact.
        
        # Set rules (allow all gripper-handle pairs)
        rules = [PyHPPRule([".*"], [".*"], True)]
        factory.setRules(rules)
        print("    ✓ Set rules: allow all")
        
        # Generate graph structure
        factory.generate()
        print("    ✓ Generated graph structure")
        
        # Initialize graph
        graph.initialize()
        print("    ✓ Graph initialized")
        
        # Store graph for later access
        self.ps.constraintGraph(graph)
        
        # Extract and store states/edges for visualization
        self._extract_pyhpp_factory_graph_structure(graph)
        
        return graph
    
    def _create_pyhpp_placement_constraints(self) -> Dict:
        """Create placement constraints for PyHPP factory mode."""
        constraints = {}
        robot = self.robot
        cfg = self.config
        
        # Tool placement on dispenser
        placement_tf = self._pose_to_se3(cfg.TOOL_ON_DISPENSER)
        joint_tool = robot.model().getJointId(cfg.TOOL_NAME)
        Id = SE3.Identity()
        
        # Placement constraint
        pc_place = Transformation.create(
            'place_frame_gripper',
            robot.asPinDevice(),
            joint_tool, Id, placement_tf,
            cfg.PLACEMENT_MASK
        )
        cts = ComparisonTypes()
        cts[:] = tuple([ComparisonType.EqualToZero] * 4)
        constraints['place_frame_gripper'] = Implicit.create(
            pc_place, cts, [True] * 4
        )
        
        # Placement complement
        pc_comp = Transformation.create(
            'place_frame_gripper/complement',
            robot.asPinDevice(),
            joint_tool, Id, placement_tf,
            cfg.PLACEMENT_COMPLEMENT_MASK
        )
        cts[:] = tuple([ComparisonType.Equality] * 2)
        constraints['place_frame_gripper/complement'] = Implicit.create(
            pc_comp, cts, [True] * 2
        )
        
        # Combined locked joint for placement
        cts_combined = ComparisonTypes()
        cts_combined[:] = (
            ComparisonType.Equality, ComparisonType.Equality,
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero, ComparisonType.EqualToZero,
            ComparisonType.EqualToZero
        )
        from pyhpp.constraints import LockedJoint
        ll = LockedJoint.createWithComp(
            robot.asPinDevice(),
            cfg.TOOL_NAME,
            np.array(cfg.TOOL_ON_DISPENSER),
            cts_combined
        )
        constraints['place_frame_gripper/hold'] = ll
        
        print("    ✓ Created placement constraints for factory")
        return constraints
    
    def _pose_to_se3(self, pose):
        """Convert [x,y,z,qx,qy,qz,qw] to SE3."""
        return SE3(
            Quaternion(pose[6], pose[3], pose[4], pose[5]),
            np.array(pose[:3])
        )
    
    def _extract_pyhpp_factory_graph_structure(self, graph):
        """Extract states and edges from factory-generated graph."""
        # PyHPP factory creates states and transitions internally
        # We need to extract them for visualization
        # For now, just create empty dicts - visualization will show
        # that factory mode doesn't provide full structure access
        self.pyhpp_states = {}
        self.pyhpp_edges = {}
        self.pyhpp_edge_topology = {}
        print("    ℹ Factory-generated graph structure not fully accessible")
    
    def _create_pyhpp_graph_manual(self):
        """Create graph manually using PyHPP APIs."""
        print("    Building PyHPP graph manually")
        
        graph = Graph("graph", self.robot, self.ps)
        cfg = self.config
        cts = self.pyhpp_constraints
        
        # Create states (order matters for solver!)
        states = {}
        states['grasp'] = graph.createState("grasp", False, 0)
        states['tool-in-air'] = graph.createState("tool-in-air", False, 0)
        states['grasp-placement'] = graph.createState(
            "grasp-placement", False, 0
        )
        states['gripper-above-tool'] = graph.createState(
            "gripper-above-tool", False, 0
        )
        states['placement'] = graph.createState("placement", False, 0)
        print(f"    ✓ Created {len(states)} states")
        
        # Store states and edges for later access
        self.pyhpp_states = states
        self.pyhpp_edges = {}
        self.pyhpp_edge_topology = {}  # Store from/to for each edge
        
        # Create edges (transitions)
        # Self-loops
        self.pyhpp_edges['transit'] = graph.createTransition(
            states['placement'], states['placement'],
            "transit", 1, states['placement']
        )
        self.pyhpp_edge_topology['transit'] = ('placement', 'placement')
        
        self.pyhpp_edges['transfer'] = graph.createTransition(
            states['grasp'], states['grasp'],
            "transfer", 1, states['grasp']
        )
        self.pyhpp_edge_topology['transfer'] = ('grasp', 'grasp')
        
        # From placement
        self.pyhpp_edges['approach-tool'] = graph.createTransition(
            states['placement'], states['gripper-above-tool'],
            "approach-tool", 1, states['placement']
        )
        self.pyhpp_edge_topology['approach-tool'] = (
            'placement', 'gripper-above-tool'
        )
        
        # From gripper-above-tool
        self.pyhpp_edges['move-gripper-away'] = graph.createTransition(
            states['gripper-above-tool'], states['placement'],
            "move-gripper-away", 1, states['placement']
        )
        self.pyhpp_edge_topology['move-gripper-away'] = (
            'gripper-above-tool', 'placement'
        )
        
        self.pyhpp_edges['grasp-tool'] = graph.createTransition(
            states['gripper-above-tool'], states['grasp-placement'],
            "grasp-tool", 1, states['placement']
        )
        self.pyhpp_edge_topology['grasp-tool'] = (
            'gripper-above-tool', 'grasp-placement'
        )
        
        # From grasp-placement
        self.pyhpp_edges['release-tool'] = graph.createTransition(
            states['grasp-placement'], states['gripper-above-tool'],
            "release-tool", 1, states['placement']
        )
        self.pyhpp_edge_topology['release-tool'] = (
            'grasp-placement', 'gripper-above-tool'
        )
        
        self.pyhpp_edges['lift-tool'] = graph.createTransition(
            states['grasp-placement'], states['tool-in-air'],
            "lift-tool", 1, states['grasp']
        )
        self.pyhpp_edge_topology['lift-tool'] = (
            'grasp-placement', 'tool-in-air'
        )
        
        # From tool-in-air
        self.pyhpp_edges['lower-tool'] = graph.createTransition(
            states['tool-in-air'], states['grasp-placement'],
            "lower-tool", 1, states['grasp']
        )
        self.pyhpp_edge_topology['lower-tool'] = (
            'tool-in-air', 'grasp-placement'
        )
        
        self.pyhpp_edges['move-tool-away'] = graph.createTransition(
            states['tool-in-air'], states['grasp'],
            "move-tool-away", 1, states['grasp']
        )
        self.pyhpp_edge_topology['move-tool-away'] = (
            'tool-in-air', 'grasp'
        )
        
        # From grasp
        self.pyhpp_edges['approach-dispenser'] = graph.createTransition(
            states['grasp'], states['tool-in-air'],
            "approach-dispenser", 1, states['grasp']
        )
        self.pyhpp_edge_topology['approach-dispenser'] = (
            'grasp', 'tool-in-air'
        )
        print("    ✓ Created transitions")
        
        # Add constraints to states
        graph.addNumericalConstraint(
            states['placement'], cts['placement']
        )
        graph.addNumericalConstraint(
            states['gripper-above-tool'], cts['placement']
        )
        graph.addNumericalConstraint(
            states['gripper-above-tool'], cts['gripper_tool_aligned']
        )
        graph.addNumericalConstraint(
            states['grasp-placement'], cts['grasp']
        )
        graph.addNumericalConstraint(
            states['grasp-placement'], cts['placement']
        )
        graph.addNumericalConstraint(
            states['tool-in-air'], cts['grasp']
        )
        graph.addNumericalConstraint(
            states['tool-in-air'], cts['tool_in_air']
        )
        graph.addNumericalConstraint(states['grasp'], cts['grasp'])
        print("    ✓ Added constraints to states")
        
        # Add constraints to edges
        for edge_name in ['transit', 'approach-tool', 'move-gripper-away',
                          'grasp-tool', 'release-tool']:
            graph.addNumericalConstraintsToTransition(
                self.pyhpp_edges[edge_name], [cts['placement/complement']]
            )
        
        for edge_name in ['lift-tool', 'lower-tool']:
            graph.addNumericalConstraintsToTransition(
                self.pyhpp_edges[edge_name], [cts['tool_in_air/complement']]
            )
        print("    ✓ Added constraints to edges")
        
        # Configure graph
        graph.maxIterations(100)
        graph.errorThreshold(0.00001)
        
        # Initialize
        graph.initialize()
        print("    ✓ Graph initialized")
        
        # Store in problem
        self.ps.constraintGraph(graph)
        
        return graph
    
    def _create_graph_manual(self):
        """Create graph manually (original implementation)."""
        print("    Building graph manually")
        
        if self.backend == "pyhpp":
            return self._create_pyhpp_graph_manual()
        
        graph = ConstraintGraph(self.robot, "graph")
        cfg = self.config
        
        # Create nodes
        graph.createNode(cfg.GRAPH_NODES)
        print(f"    ✓ Created {len(cfg.GRAPH_NODES)} states")
        
        # Create edges
        self._create_edges(graph)
        print("    ✓ Created edges (transitions)")
        
        # Assign node constraints
        self._assign_node_constraints(graph)
        print("    ✓ Added constraints to nodes")
        
        # Assign edge constraints
        self._assign_edge_constraints(graph)
        print("    ✓ Added constraints to edges")
        
        # Set constant RHS
        self.ps.setConstantRightHandSide("placement", True)
        self.ps.setConstantRightHandSide("placement/complement", False)
        self.ps.setConstantRightHandSide("tool_in_air/complement", False)
        print("    ✓ Set constant right-hand side")
        
        # Set security margins BEFORE initialize
        self._set_security_margins(graph)
        print(
            f"    ✓ Set security margin ({cfg.CONTACT_MARGIN}m) "
            "for placement edges"
        )
        
        # Initialize graph
        graph.initialize()
        print("    ✓ Graph initialized")
        
        return graph
        
    def _create_edges(self, graph):
        """Create all state transitions."""
        # Self-loops
        graph.createEdge(
            'placement', 'placement', 'transit', 1, 'placement'
        )
        graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
        
        # From placement
        graph.createEdge(
            'placement', 'gripper-above-tool', 'approach-tool',
            1, 'placement'
        )
        
        # From gripper-above-tool
        graph.createEdge(
            'gripper-above-tool', 'placement', 'move-gripper-away',
            1, 'placement'
        )
        graph.createEdge(
            'gripper-above-tool', 'grasp-placement', 'grasp-tool',
            1, 'placement'
        )
        
        # From grasp-placement
        graph.createEdge(
            'grasp-placement', 'gripper-above-tool', 'release-tool',
            1, 'placement'
        )
        graph.createEdge(
            'grasp-placement', 'tool-in-air', 'lift-tool', 1, 'grasp'
        )
        
        # From tool-in-air
        graph.createEdge(
            'tool-in-air', 'grasp-placement', 'lower-tool', 1, 'grasp'
        )
        graph.createEdge(
            'tool-in-air', 'grasp', 'move-tool-away', 1, 'grasp'
        )
        
        # From grasp
        graph.createEdge(
            'grasp', 'tool-in-air', 'approach-dispenser', 1, 'grasp'
        )
        
    def _assign_node_constraints(self, graph):
        """Assign constraints to states."""
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
        
    def _assign_edge_constraints(self, graph):
        """Assign path constraints to edges."""
        # Edges with placement/complement (tool on surface)
        placement_edges = [
            "transit", "approach-tool", "move-gripper-away",
            "grasp-tool", "release-tool"
        ]
        for edge in placement_edges:
            graph.addConstraints(
                edge=edge,
                constraints=Constraints(
                    numConstraints=["placement/complement"]
                )
            )
            
        # Edges with tool_in_air/complement
        for edge in ["lift-tool", "lower-tool"]:
            graph.addConstraints(
                edge=edge,
                constraints=Constraints(
                    numConstraints=["tool_in_air/complement"]
                )
            )
            
        # Free motion edges
        for edge in ["transfer", "move-tool-away", "approach-dispenser"]:
            graph.addConstraints(edge=edge, constraints=Constraints())
            
    def _set_security_margins(self, graph):
        """Set security margins for surface contact edges."""
        cfg = self.config
        for edge_name in cfg.PLACEMENT_EDGES:
            graph.setSecurityMarginForEdge(
                edge_name,
                cfg.TOOL_CONTACT_JOINT,
                cfg.DISPENSER_CONTACT_JOINT,
                cfg.CONTACT_MARGIN
            )
    
    def _generate_configs_pyhpp(self, q_init: List[float]) -> Dict:
        """Generate configurations using PyHPP APIs."""
        configs = {}
        cfg = self.config
        
        # Access stored states and edges from graph creation
        graph = self.graph
        states = self.pyhpp_states
        edges = self.pyhpp_edges
        
        # Get configuration shooter
        shooter = self.ps.configurationShooter()
        
        # 1. Project onto placement
        print("    1. Projecting onto 'placement' state...")
        state_placement = states.get("placement")
        if state_placement:
            result = graph.applyStateConstraints(
                state_placement, np.array(q_init)
            )
            configs["q_init"] = result.configuration.tolist()
            print(f"       \u2713 q_init projected")
        else:
            configs["q_init"] = q_init
            print("       \u26a0 State not found, using input")
        
        # 2. Generate approach config
        print("    2. Generating 'approach-tool' config...")
        edge_approach = edges.get("approach-tool")
        if edge_approach:
            for i in range(cfg.MAX_RANDOM_ATTEMPTS):
                q_rand = shooter.shoot()
                result = graph.generateTargetConfig(
                    edge_approach, np.array(configs["q_init"]), q_rand
                )
                if result.success:
                    configs["q_approach"] = result.configuration.tolist()
                    print(f"       \u2713 Generated after {i+1} attempts")
                    break
            if "q_approach" not in configs:
                print("       \u26a0 Failed to generate, stopping")
                return configs
        
        # 3. Project onto gripper-above-tool
        print("    3. Projecting onto 'gripper-above-tool' state...")
        state_above = states.get("gripper-above-tool")
        if state_above:
            result = graph.applyStateConstraints(
                state_above, np.array(configs["q_approach"])
            )
            configs["q_above"] = result.configuration.tolist()
            print(f"       \u2713 q_above projected")
        
        # 4. Generate grasp config
        print("    4. Generating 'grasp-tool' config...")
        edge_grasp = edges.get("grasp-tool")
        if edge_grasp:
            for i in range(cfg.MAX_RANDOM_ATTEMPTS):
                q_rand = shooter.shoot()
                result = graph.generateTargetConfig(
                    edge_grasp, np.array(configs["q_above"]), q_rand
                )
                if result.success:
                    configs["q_grasp"] = result.configuration.tolist()
                    print(f"       \u2713 Generated after {i+1} attempts")
                    break
            if "q_grasp" not in configs:
                print("       \u26a0 Failed to generate, stopping")
                return configs
        
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        state_grasp_place = states.get("grasp-placement")
        if state_grasp_place:
            result = graph.applyStateConstraints(
                state_grasp_place, np.array(configs["q_grasp"])
            )
            configs["q_grasp_place"] = result.configuration.tolist()
            print(f"       \u2713 q_grasp_place projected")
        
        # 6. Generate lift config
        print("    6. Generating 'lift-tool' config...")
        edge_lift = edges.get("lift-tool")
        if edge_lift:
            for i in range(cfg.MAX_RANDOM_ATTEMPTS):
                q_rand = shooter.shoot()
                result = graph.generateTargetConfig(
                    edge_lift, np.array(configs["q_grasp_place"]), q_rand
                )
                if result.success:
                    configs["q_lifted"] = result.configuration.tolist()
                    print(f"       \u2713 Generated after {i+1} attempts")
                    break
            if "q_lifted" not in configs:
                print("       \u26a0 Failed to generate, stopping")
                return configs
        
        # 7. Project onto tool-in-air
        print("    7. Projecting onto 'tool-in-air' state...")
        state_air = states.get("tool-in-air")
        if state_air:
            result = graph.applyStateConstraints(
                state_air, np.array(configs["q_lifted"])
            )
            configs["q_tool_air"] = result.configuration.tolist()
            print(f"       \u2713 q_tool_air projected")
        
        # 8. Generate goal
        print("    8. Generating goal configuration...")
        q_goal_modified = self.config_gen.modify_object_pose(
            configs["q_init"],
            object_index=0,
            translation_delta=[0.2, 0.0, 0.0]
        )
        if state_placement:
            result = graph.applyStateConstraints(
                state_placement, np.array(q_goal_modified)
            )
            configs["q_goal"] = result.configuration.tolist()
            print(f"       \u2713 q_goal projected")
        
        return configs
            
    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations."""
        # Factory mode creates different node/edge names
        # For now, just return initial config for factory mode
        if self.use_factory:
            print("    Factory mode: Using initial configuration only")
            print("    (Detailed waypoint generation not yet implemented)")
            return {"q_init": q_init}
        
        # PyHPP uses different configuration generation API
        if self.backend == "pyhpp":
            return self._generate_configs_pyhpp(q_init)
        
        cg = self.config_gen
        cfg = self.config
        
        # Update max attempts
        cg.max_attempts = cfg.MAX_RANDOM_ATTEMPTS
        
        # 1. Project initial onto placement
        print("    1. Projecting onto 'placement' state...")
        cg.project_on_node("placement", q_init, "q_init")
        
        # 2. Generate approach config
        print("    2. Generating 'approach-tool' config...")
        res, _ = cg.generate_via_edge(
            "approach-tool", cg.configs["q_init"], "q_approach"
        )
        if not res:
            return cg.configs
            
        # 3. Project onto gripper-above-tool
        print("    3. Projecting onto 'gripper-above-tool' state...")
        cg.project_on_node(
            "gripper-above-tool", cg.configs["q_approach"], "q_above"
        )
        
        # 4. Generate grasp config
        print("    4. Generating 'grasp-tool' config...")
        res, _ = cg.generate_via_edge(
            "grasp-tool", cg.configs["q_above"], "q_grasp"
        )
        if not res:
            return cg.configs
            
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        cg.project_on_node(
            "grasp-placement", cg.configs["q_grasp"], "q_grasp_place"
        )
        
        # 6. Generate lift config
        print("    6. Generating 'lift-tool' config...")
        res, _ = cg.generate_via_edge(
            "lift-tool", cg.configs["q_grasp_place"], "q_lifted"
        )
        if not res:
            return cg.configs
            
        # 7. Project onto tool-in-air
        print("    7. Projecting onto 'tool-in-air' state...")
        cg.project_on_node("tool-in-air", cg.configs["q_lifted"], "q_tool_air")
        
        # 8. Generate goal (move tool 20cm in x)
        print("    8. Generating goal configuration...")
        q_goal_modified = cg.modify_object_pose(
            cg.configs["q_init"],
            object_index=0,  # frame_gripper is first object
            translation_delta=[0.2, 0.0, 0.0]
        )
        cg.project_on_node("placement", q_goal_modified, "q_goal")
        
        return cg.configs


# ============================================================================
# Main Execution
# ============================================================================

def main(
    visualize: bool = True,
    solve: bool = False,
    show_joints: bool = False,
    use_factory: bool = False,
    backend: str = "corba"
):
    """
    Run the grasp frame_gripper task.
    
    Args:
        visualize: Show configurations in viewer
        solve: Attempt to solve planning problem
        show_joints: Print joint information
        use_factory: Use ConstraintGraphFactory (automatic) instead of
            manual graph
        backend: "corba" or "pyhpp" - which backend to use
    """
    # Validate backend availability
    if backend == "corba" and not HAS_CORBA:
        print("Error: CORBA backend not available")
        return None
    elif backend == "pyhpp" and not HAS_PYHPP:
        print("Error: PyHPP backend not available. Install pyhpp.")
        return None
        
    print(f"Using backend: {backend.upper()}")
    
    # Create and setup task
    task = GraspFrameGripperTask(use_factory=use_factory, backend=backend)
    task.setup(
        validation_step=GraspFrameGripperConfig.PATH_VALIDATION_STEP,
        projector_step=GraspFrameGripperConfig.PATH_PROJECTOR_STEP
    )
    
    # Print joint info if requested
    if show_joints:
        print_joint_info(task.robot)
        
    # Run task
    result = task.run(visualize=visualize, solve=solve)
    
    # Print summary
    print("\n" + "=" * 70)
    print("Task Complete!")
    print("=" * 70)

    configs = result["configs"]
    print(f"\nGenerated {len(configs)} configurations:")
    for name, q in configs.items():
        print(f"  {name}: {len(q)} DOF")

    print("\nTo visualize specific configs:")
    print("  task.planner.visualize(result['configs']['q_init'])")
    print("  task.planner.visualize(result['configs']['q_above'])")
    print("  task.planner.visualize(result['configs']['q_goal'])")

    if solve:
        print("\nTo replay path:")
        print("  task.planner.play_path(0)")

    # Visualize constraint graph
    print("\n📊 Generating constraint graph visualization...")
    # Pass state/edge dicts for PyHPP backend
    states_dict = getattr(task, 'pyhpp_states', None)
    edges_dict = getattr(task, 'pyhpp_edges', None)
    edge_topology = getattr(task, 'pyhpp_edge_topology', None)
    viz_path = visualize_constraint_graph(
        task.graph,
        output_path="constraint_graph",
        states_dict=states_dict,
        edges_dict=edges_dict,
        edge_topology=edge_topology
    )
    if viz_path:
        print(f"✓ Graph visualization saved to: {viz_path}")
    
    return task, result


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="UR10 grasps frame_gripper from dispenser"
    )
    parser.add_argument(
        "--no-viz", action="store_true", help="Disable visualization"
    )
    parser.add_argument(
        "--solve", action="store_true", help="Solve planning problem"
    )
    parser.add_argument(
        "--show-joints", action="store_true", help="Print joint info"
    )
    parser.add_argument(
        "--factory", action="store_true",
        help="Use ConstraintGraphFactory (automatic graph generation)"
    )
    parser.add_argument(
        "--backend", type=str, default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use: corba (default) or pyhpp"
    )
    
    args = parser.parse_args()
    
    task, result = main(
        visualize=not args.no_viz,
        solve=args.solve,
        show_joints=args.show_joints,
        use_factory=args.factory,
        backend=args.backend
    )
    
    if task and result:
        print("\n" + "=" * 70)
        print("Available objects:")
        print("=" * 70)
        print("  task   - GraspFrameGripperTask instance")
        print("  result - Dictionary with configs, planner, robot, ps, graph")
