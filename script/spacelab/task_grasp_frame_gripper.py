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
from collections import deque

import numpy as np

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import GraphBuilder, ConstraintBuilder
from agimus_spacelab.config.base_config import BaseTaskConfig
from agimus_spacelab.visualization import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_all_handles,
    visualize_all_grippers,
)

# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import TaskConfigurations  # noqa: E402


# ============================================================================
# Task Configuration
# ============================================================================
GraspFrameGripperConfig = TaskConfigurations.GraspFrameGripper
# Initialize poses
GraspFrameGripperConfig.init_poses()


class _FactoryGraphTaskSpec(BaseTaskConfig):
    """Adapter to use GraphBuilder.build_graph_for_task in factory mode."""

    ROBOT_NAMES = GraspFrameGripperConfig.ROBOT_NAMES
    ENVIRONMENT_NAMES = GraspFrameGripperConfig.ENVIRONMENT_NAMES
    OBJECTS = GraspFrameGripperConfig.OBJECTS

    GRIPPERS = GraspFrameGripperConfig.GRIPPERS
    HANDLES_PER_OBJECT = GraspFrameGripperConfig.HANDLES_PER_OBJECT
    CONTACT_SURFACES_PER_OBJECT = [[] for _ in OBJECTS]
    ENVIRONMENT_CONTACTS = ["ground_demo/tools_dispenser_surface"]
    VALID_PAIRS = GraspFrameGripperConfig.VALID_PAIRS


# ============================================================================
# Task Implementation
# ============================================================================

class GraspFrameGripperTask(ManipulationTask):
    """UR10 grasps frame_gripper from dispenser."""

    # VISPA arms are not used in this task; keep them fixed during planning.
    FREEZE_JOINT_SUBSTRINGS = ["vispa"]
    
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

        # In this scenario, the SRDF gripper/handle frames are on fixed/fake
        # links (children of the EE link / object link). Disable collisions
        # between the whole EE subtree and the object subtree so factory grasp
        # planning doesn't fail on link-level collisions.
        if self.backend == "corba":
            self.scene_builder.disable_collisions_between_subtrees(
                "spacelab/ur10_link_7",
                self.config.TOOL_CONTACT_JOINT,
                remove_collision=True,
                remove_distance=False,
                verbose=True,
                max_pairs=200,
            )
    
    def create_constraints(self) -> None:
        """Create all transformation constraints."""
        # Factory mode creates constraints automatically
        if self.use_factory:
            print("    (Factory mode: constraints created automatically)")
            return
        
        else:
            print("    Creating constraints manually")
            # Get robot reference (differs by backend)
            robot = (
                self.planner.get_robot()
                if self.backend == "pyhpp"
                else self.robot
            )
            cb = ConstraintBuilder
            cfg = self.config
            
            # 1. Grasp: gripper holds tool
            cb.create_grasp_constraint(
                self.ps, "grasp",
                cfg.GRIPPER_NAME, cfg.TOOL_NAME,
                cfg.TOOL_IN_GRIPPER, cfg.GRASP_MASK,
                robot=robot, backend=self.backend
            )
            
            # 2. Placement: tool on dispenser (Z and rotations fixed)
            cb.create_placement_constraint(
                self.ps, "placement",
                cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
                cfg.PLACEMENT_MASK,
                robot=robot, backend=self.backend
            )
            
            # 3. Placement complement: X, Y free (sliding on surface)
            cb.create_complement_constraint(
                self.ps, "placement",
                cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
                cfg.PLACEMENT_COMPLEMENT_MASK,
                robot=robot, backend=self.backend
            )
            
            # 4. Gripper-tool aligned: gripper above tool
            cb.create_grasp_constraint(
                self.ps, "gripper_tool_aligned",
                cfg.GRIPPER_NAME, cfg.TOOL_NAME,
                cfg.GRIPPER_ABOVE_TOOL, cfg.GRASP_MASK,
                robot=robot, backend=self.backend
            )
            
            # 5. Tool in air: lifted position
            cb.create_placement_constraint(
                self.ps, "tool_in_air",
                cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
                cfg.PLACEMENT_MASK,
                robot=robot, backend=self.backend
            )
            
            # 6. Tool in air complement
            cb.create_complement_constraint(
                self.ps, "tool_in_air",
                cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
                cfg.PLACEMENT_COMPLEMENT_MASK,
                robot=robot, backend=self.backend
            )
        
    def create_graph(self):
        """Create and configure constraint graph."""
        # Backend-specific objects
        if self.backend == "pyhpp":
            robot = self.planner.get_robot()
            problem = self.planner.get_problem()
        else:
            robot = self.robot
            problem = self.ps

        # Initialize GraphBuilder
        self.graph_builder = GraphBuilder(
            self.planner, robot, problem, backend=self.backend
        )

        if self.use_factory:
            return self.graph_builder.build_graph_for_task(
                _FactoryGraphTaskSpec, mode="factory"
            )
        else:
            if self.backend == "corba":
                return self._create_corba_graph_manual()
            if self.backend == "pyhpp":
                raise NotImplementedError(
                    "Manual graph mode is not implemented for PyHPP in this "
                    "script. Use --factory with --backend pyhpp."
                )

        raise ValueError(f"Unsupported backend: {self.backend}")

    def _create_corba_graph_manual(self):
        """Create the manual CORBA graph using GraphBuilder helpers."""
        print("    Building graph manually")

        gb = self.graph_builder
        cfg = self.config
        graph_def = getattr(cfg, "GRASP_FG_GRAPH", None)
        if not isinstance(graph_def, dict):
            raise RuntimeError("Missing 'GRASP_FG_GRAPH' graph config")
        gb.create_manual_graph(name="graph")

        # Create states (order matters for solver performance)
        state_names = getattr(cfg, "GRAPH_NODES", None) or list(
            graph_def.get("states", {}).keys()
        )
        gb.add_states(state_names)
        print(f"    ✓ Created {len(state_names)} states")

        # Create edges from declarative definition
        for edge_name, edge_info in graph_def.get("edges", {}).items():
            gb.add_edge(
                edge_info["from"],
                edge_info["to"],
                edge_name,
                edge_info.get("weight", 1),
                edge_info["in"],
            )
        print("    ✓ Created edges (transitions)")

        # Add constraints to states from declarative definition
        for state_name, state_info in graph_def.get("states", {}).items():
            constraint_names = state_info.get("constraints", [])
            if constraint_names:
                gb.add_state_constraints(
                    state_name, [], constraint_names=constraint_names
                )
        print("    ✓ Added constraints to nodes")

        # Add constraints to edges from declarative definition
        edge_constraints_def = graph_def.get("edge_constraints", {})
        for constraint_name, edge_list in edge_constraints_def.items():
            for edge_name in edge_list:
                gb.add_edge_constraints(
                    edge_name, [], constraint_names=[constraint_name]
                )

        # Free motion edges (no path constraints)
        for edge_name in graph_def.get("free_motion_edges", []):
            gb.add_edge_constraints(edge_name, [], constraint_names=[])
        print("    ✓ Added constraints to edges")

        # Set constant RHS (CORBA only)
        for constraint_name, is_constant in graph_def.get(
            "constant_rhs", {}
        ).items():
            self.ps.setConstantRightHandSide(constraint_name, is_constant)
        print("    ✓ Set constant right-hand side")

        # Set security margins BEFORE initialize
        for edge_name in cfg.PLACEMENT_EDGES:
            gb.graph.setSecurityMarginForEdge(
                edge_name,
                cfg.TOOL_CONTACT_JOINT,
                cfg.DISPENSER_CONTACT_JOINT,
                cfg.CONTACT_MARGIN,
            )
        print(
            f"    ✓ Set security margin ({cfg.CONTACT_MARGIN}m) "
            "for placement edges"
        )

        # Initialize graph
        gb.finalize_manual_graph()
        print("    ✓ Graph initialized")
        return gb.graph

    def _factory_state_from_config(self, q: List[float]) -> str:
        """Best-effort detection of the current factory state name."""
        graph = self.graph

        if self.backend == "corba" and hasattr(graph, "getNode"):
            # CORBA: ConstraintGraph.getNode(config) returns a node name.
            try:
                return graph.getNode(list(q))
            except Exception:
                pass

        if self.backend == "pyhpp" and hasattr(graph, "getState"):
            try:
                state = graph.getState(np.array(q))
                name = getattr(state, "name", None)
                if callable(name):
                    return name()
                if isinstance(name, str):
                    return name
            except Exception:
                pass

        # Fallback: guess a reasonable start state.
        place_name = f"place_{self.config.OBJECTS[0]}"
        if place_name in getattr(self.graph_builder, "states", {}):
            return place_name
        # As a last resort, return any state name.
        states = list(getattr(self.graph_builder, "states", {}).keys())
        return states[0] if states else ""

    def _freeze_vispa_joints(self, q: List[float], q_ref: List[float]) -> List[float]:
        """Keep VISPA and VISPA2 joint values constant.

        This task only moves UR10 and the grasped object. Projections and random
        shooting can otherwise drift VISPA/VISPA2 joints.
        """
        if self.backend != "corba":
            return q

        robot = self.robot
        if robot is None:
            return q

        q_out = list(q)
        try:
            joint_names = robot.getJointNames()
        except Exception:
            return q_out

        for joint in joint_names:
            if "vispa" not in joint:
                continue
            try:
                rank = robot.rankInConfiguration[joint]
                size = robot.getJointConfigSize(joint)
            except Exception:
                continue
            if size <= 0:
                continue
            if rank + size > len(q_out) or rank + size > len(q_ref):
                continue
            q_out[rank : rank + size] = q_ref[rank : rank + size]

        return q_out

    @staticmethod
    def _bfs_edge_path(
        start_state: str,
        goal_state: str,
        edge_topology: Dict[str, tuple[str, str]],
    ) -> List[str]:
        """Find a directed edge-name path from start_state to goal_state."""
        adjacency: Dict[str, List[tuple[str, str]]] = {}
        for edge_name, (src, dst) in edge_topology.items():
            adjacency.setdefault(src, []).append((dst, edge_name))

        queue: deque[str] = deque([start_state])
        prev_state: Dict[str, str] = {}
        prev_edge: Dict[str, str] = {}
        visited = {start_state}

        while queue:
            state = queue.popleft()
            if state == goal_state:
                break
            for nxt, edge_name in adjacency.get(state, []):
                if nxt in visited:
                    continue
                visited.add(nxt)
                prev_state[nxt] = state
                prev_edge[nxt] = edge_name
                queue.append(nxt)

        if goal_state not in visited:
            return []

        # Reconstruct path
        edges: List[str] = []
        cur = goal_state
        while cur != start_state:
            edges.append(prev_edge[cur])
            cur = prev_state[cur]
        edges.reverse()
        return edges

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations."""
        cg = self.config_gen
        cfg = self.config

        q_ref = list(q_init)

        # Update max attempts
        cg.max_attempts = cfg.MAX_RANDOM_ATTEMPTS
        
        if self.use_factory:
            print("    Factory mode: generating a pick+hold goal")

            gripper = cfg.GRIPPERS[0]
            handle = cfg.HANDLES_PER_OBJECT[0][0]
            desired_grasp = f"{gripper} grasps {handle}"

            all_states = list(self.graph_builder.get_states().keys())
            goal_candidates = [s for s in all_states if s == desired_grasp]
            if not goal_candidates:
                goal_candidates = [s for s in all_states if desired_grasp in s]
            if not goal_candidates:
                raise RuntimeError(
                    "Could not find a grasp state containing '%s'. "
                    "Available states containing 'grasps': %s"
                    % (
                        desired_grasp,
                        [s for s in all_states if "grasps" in s][:20],
                    )
                )
            goal_state = goal_candidates[0]

            start_state = self._factory_state_from_config(q_init)
            if not start_state:
                raise RuntimeError("Could not determine start state")

            # Ensure we start exactly in the detected start state.
            cg.project_on_node(start_state, q_init, "q_init")
            cg.configs["q_init"] = self._freeze_vispa_joints(
                cg.configs["q_init"], q_ref
            )

            edge_path = self._bfs_edge_path(
                start_state,
                goal_state,
                self.graph_builder.get_edge_topology(),
            )
            if not edge_path:
                topo = self.graph_builder.get_edge_topology()
                print(
                    "    ! No edge path found from '%s' to '%s' "
                    "(extracted %d transition endpoints). "
                    "Falling back to direct projection into goal state."
                    % (start_state, goal_state, len(topo))
                )

                q_current = cg.configs["q_init"]
                try:
                    cg.project_on_node(goal_state, q_current, "q_goal")
                    cg.configs["q_goal"] = self._freeze_vispa_joints(
                        cg.configs["q_goal"], q_ref
                    )
                except Exception as exc:
                    raise RuntimeError(
                        "No path found in factory graph from '%s' to '%s', "
                        "and direct projection into '%s' failed: %s "
                        "(extracted %d transition endpoints)"
                        % (
                            start_state,
                            goal_state,
                            goal_state,
                            exc,
                            len(topo),
                        )
                    )

                print(f"    ✓ Goal state (projected): {goal_state}")
                return cg.configs

            q_current = cg.configs["q_init"]
            for i, edge_name in enumerate(edge_path):
                safe_edge = edge_name.replace(" ", "_").replace("/", "_")
                label = f"q_wp_{i}_{safe_edge}"
                ok, q_next = cg.generate_via_edge(edge_name, q_current, label)
                if not ok or q_next is None:
                    raise RuntimeError(
                        "Failed generating config via edge '%s'" % edge_name
                    )
                q_next_frozen = self._freeze_vispa_joints(q_next, q_ref)
                cg.configs[label] = q_next_frozen
                q_current = q_next_frozen

            cg.configs["q_goal"] = self._freeze_vispa_joints(q_current, q_ref)
            print(f"    ✓ Goal state: {goal_state}")
            return cg.configs

        # 1. Project initial onto placement
        print("    1. Projecting onto 'placement' state...")
        cg.project_on_node("placement", q_init, "q_init")
        cg.configs["q_init"] = self._freeze_vispa_joints(cg.configs["q_init"], q_ref)
        
        # 2. Generate approach config
        print("    2. Generating 'approach-tool' config...")
        res, _ = cg.generate_via_edge(
            "approach-tool", cg.configs["q_init"], "q_approach"
        )
        if not res:
            return cg.configs
        cg.configs["q_approach"] = self._freeze_vispa_joints(
            cg.configs["q_approach"], q_ref
        )
            
        # 3. Project onto gripper-above-tool
        print("    3. Projecting onto 'gripper-above-tool' state...")
        cg.project_on_node(
            "gripper-above-tool", cg.configs["q_approach"], "q_above"
        )
        cg.configs["q_above"] = self._freeze_vispa_joints(cg.configs["q_above"], q_ref)
        
        # 4. Generate grasp config
        print("    4. Generating 'grasp-tool' config...")
        res, _ = cg.generate_via_edge(
            "grasp-tool", cg.configs["q_above"], "q_grasp"
        )
        if not res:
            return cg.configs
        cg.configs["q_grasp"] = self._freeze_vispa_joints(cg.configs["q_grasp"], q_ref)
            
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        cg.project_on_node(
            "grasp-placement", cg.configs["q_grasp"], "q_grasp_place"
        )
        cg.configs["q_grasp_place"] = self._freeze_vispa_joints(
            cg.configs["q_grasp_place"], q_ref
        )
        
        # 6. Generate lift config
        print("    6. Generating 'lift-tool' config...")
        res, _ = cg.generate_via_edge(
            "lift-tool", cg.configs["q_grasp_place"], "q_lifted"
        )
        if not res:
            return cg.configs
        cg.configs["q_lifted"] = self._freeze_vispa_joints(cg.configs["q_lifted"], q_ref)
            
        # 7. Project onto tool-in-air
        print("    7. Projecting onto 'tool-in-air' state...")
        cg.project_on_node("tool-in-air", cg.configs["q_lifted"], "q_tool_air")
        cg.configs["q_tool_air"] = self._freeze_vispa_joints(
            cg.configs["q_tool_air"], q_ref
        )

        # 8. Goal: use the lifted configuration as goal
        cg.configs["q_goal"] = self._freeze_vispa_joints(
            cg.configs["q_tool_air"], q_ref
        )
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
    preferred_configs = [
                    "q_above",
                    "q_grasp_place",
                    "q_tool_air",
                ]
    result = task.run(visualize=visualize, solve=solve,
                      preferred_configs=preferred_configs,
                      max_iterations=50000)
    
    # Visualize handles and grippers if viewer available
    if visualize and result.get('viewer'):
        print("\n" + "=" * 70)
        print("Visualizing Handles and Grippers")
        print("=" * 70)
        viewer = result['viewer']
        
        # Collect all handle names
        if task.use_factory and getattr(task.graph_builder, "factory", None):
            handle_names = task.graph_builder.factory.handles
        else:
            handle_names = [
                handle
                for handles in GraspFrameGripperConfig.HANDLES_PER_OBJECT
                for handle in handles
            ]
        
        # Visualize handles with approach arrows
        visualize_all_handles(
            viewer, handle_names,
            show_approach=False,
            frame_color=[0, 0.8, 0, 1],   # Green
            arrow_color=[0, 1, 1, 1],      # Cyan
            axis_length=0.05,
            arrow_length=0.1
        )
        
        # Visualize grippers with approach arrows
        gripper_names = GraspFrameGripperConfig.GRIPPERS
        visualize_all_grippers(
            viewer, gripper_names,
            show_approach=False,
            frame_color=[1, 0, 0, 1],      # Red
            arrow_color=[1, 0.5, 0, 1],    # Orange
            axis_length=0.05,
            arrow_length=0.1
        )
        viewer.client.gui.refresh()
    
    # Print summary
    print("\n" + "=" * 70)
    print("Results Summary")
    print("=" * 70)

    print("\nTo visualize specific configs:")
    print("  task.planner.visualize(result['configs']['q_init'])")

    if solve:
        print("\nTo replay path:")
        print("  task.planner.play_path(0)")

    # Visualize constraint graph
    print("\n📊 Generating constraint graph visualization...")
    # Pass state/edge dicts for PyHPP backend
    states_dict = getattr(task, 'pyhpp_states', None)
    edges_dict = getattr(task, 'pyhpp_edges', None)
    edge_topology = getattr(task, 'pyhpp_edge_topology', None)
    visualize_constraint_graph(
        task.graph_builder,
        output_path="constraint_graph",
        states_dict=states_dict,
        edges_dict=edges_dict,
        edge_topology=edge_topology
    )
    
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
