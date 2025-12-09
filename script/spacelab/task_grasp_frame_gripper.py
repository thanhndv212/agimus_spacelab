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

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import GraphBuilder, ConstraintBuilder
from agimus_spacelab.visualization import print_joint_info, visualize_constraint_graph, displayHandle, displayGripper
from agimus_spacelab.utils import xyzrpy_to_xyzquat

# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import TaskConfigurations

# Import backend availability flags for validation
try:
    from agimus_spacelab.backends import get_available_backends
    _available = get_available_backends()
    HAS_CORBA = 'corba' in _available
    HAS_PYHPP = 'pyhpp' in _available
except ImportError:
    HAS_CORBA = False
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
    from pinocchio import SE3, StdVec_Bool as Mask
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
GraspFrameGripperConfig = TaskConfigurations.GraspFrameGripper
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
            task_name="Spacelab Manipulation: UR10 Grasps Frame Gripper",
            backend=backend
        )
        self.config = GraspFrameGripperConfig
        self.use_factory = use_factory
        
    def get_robot_names(self) -> List[str]:
        return self.config.ROBOT_NAMES
    
    def get_composite_names(self) -> List[str]:
        return self.config.ROBOT_NAMES
    
    def get_object_names(self) -> List[str]:
        return self.config.OBJECTS
    
    def get_environment_names(self) -> List[str]:
        return self.config.ENVIRONMENT_NAMES
        
    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return self.config.ROBOTS

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return self.config.OBJECTS
        
    def setup_collision_management(self) -> None:
        """Disable collision between tool and dispenser surface."""
        self.scene_builder.disable_collision_pair(
            "ground_demo/link_TD_0",  # Dispenser surface
            self.config.TOOL_CONTACT_JOINT,
            remove_collision=True,
            remove_distance=False
        )
    
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
            if self.backend == "corba":
                return self._create_corba_graph_with_factory()
            if self.backend == "pyhpp":
                return self._create_pyhpp_graph_with_factory()
        else:
            if self.backend == "corba":
                return self._create_corba_graph_manual()
            if self.backend == "pyhpp":
                return self._create_pyhpp_graph_manual()
    
    def _create_corba_graph_with_factory(self):
        """Create graph using ConstraintGraphFactory (automatic)."""
        print(
            "    Using ConstraintGraphFactory for automatic graph generation"
        )
        
        # CORBA backend
        graph = ConstraintGraph(self.robot, "graph")
        factory = ConstraintGraphFactory(graph)
        
        # Set grippers
        grippers = self.config.GRIPPERS
        factory.setGrippers(grippers)
        print(f"    ✓ Set grippers: {grippers}")
        
        # Set objects with handles and contact surfaces
        objects = self.config.OBJECTS
        handles_per_object = self.config.HANDLES_PER_OBJECT
        contact_surfaces_per_object = [[] for _ in objects]  # No contact surfaces for now
        factory.setObjects(
            objects, handles_per_object, contact_surfaces_per_object
        )
        print(f"    ✓ Set objects: {objects}")
        
        # Set environment contacts (dispenser surface)
        env_contacts = ["ground_demo/tools_dispenser_surface"]
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

    def _create_corba_graph_manual(self):
        """Create graph manually (original implementation)."""
        print("    Building graph manually")
        
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
    
    # Visualize handles and grippers if viewer available
    if visualize and result.get('viewer'):
        print("\n" + "=" * 70)
        print("Visualizing Handles and Grippers")
        print("=" * 70)
        viewer = result['viewer']
        
        # Display handle
        handle_names = [
            handle
            for handles in GraspFrameGripperConfig.HANDLES_PER_OBJECT
            for handle in handles
        ]
        for handle_name in handle_names:
            if displayHandle(viewer, handle_name):
                print("    ✓ Handle frame displayed")
        
        # Display gripper
        gripper_names = GraspFrameGripperConfig.GRIPPERS
        for gripper_name in gripper_names:
            if displayGripper(viewer, gripper_name):
                print("    ✓ Gripper frame displayed")
        
        viewer.client.gui.refresh()
    
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
