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

import numpy as np

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning.config import (
    bfs_edge_path,
)
from agimus_spacelab.visualization import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_all_handles,
    visualize_all_grippers,
)

# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))


def initialize_task_config() -> None:
    """Initialize GraspFrameGripperConfig with derived parameters."""
    from spacelab_config import TaskConfigurations  # noqa: E402
    GraspFrameGripperConfig = TaskConfigurations.GraspFrameGripper
    # Initialize poses
    GraspFrameGripperConfig.init_poses()
    return GraspFrameGripperConfig


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
        from spacelab_config import DEFAULT_PATHS, JointBounds  # noqa: PLC0415
        super().__init__(
            task_name="Spacelab Manipulation: UR10 Grasps Frame Gripper",
            backend=backend,
            FILE_PATHS=DEFAULT_PATHS,
            joint_bounds=JointBounds,
        )
        self.task_config = initialize_task_config()
        self.use_factory = use_factory
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> List[float]:
        """Build initial configuration from SpaceLab defaults."""
        from spacelab_config import InitialConfigurations  # noqa: PLC0415
        from agimus_spacelab.utils.transforms import xyzrpy_to_xyzquat
        q_robot = []
        for group in self.task_config.ROBOTS:
            if hasattr(InitialConfigurations, group):
                q_robot.extend(list(getattr(InitialConfigurations, group)))
        q_objects = []
        for obj_name in self.task_config.OBJECTS:
            obj_attr = obj_name.replace("-", "_").replace(" ", "_").upper()
            if hasattr(InitialConfigurations, obj_attr):
                pose = xyzrpy_to_xyzquat(getattr(InitialConfigurations, obj_attr))
                q_objects.extend(pose.tolist())
            else:
                q_objects.extend([0.0] * 7)
        return q_robot + q_objects

    def setup_collision_management(self) -> None:
        """Disable collision between tool and dispenser surface."""
        self.scene_builder.disable_collision_pair(
            "ground_demo/link_TD_0",  # Dispenser surface
            self.task_config.TOOL_CONTACT_JOINT,
            remove_collision=True,
            remove_distance=False
        )

        # Disable collisions between the EE subtree and the grasped object
        # subtree (SRDF grippers/handles live on fixed/fake child links whose
        # collision geometries would otherwise cause planning failures).
        self.scene_builder.disable_collisions_between_subtrees(
            "spacelab/ur10_link_7",
            self.task_config.TOOL_CONTACT_JOINT,
            remove_collision=True,
            remove_distance=False,
            verbose=True,
            max_pairs=200,
        )

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
        place_name = f"place_{self.task_config.OBJECTS[0]}"
        if place_name in getattr(self.graph_builder, "states", {}):
            return place_name
        # As a last resort, return any state name.
        states = list(getattr(self.graph_builder, "states", {}).keys())
        return states[0] if states else ""

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations."""
        cg = self.config_gen
        if q_init is None:
            raise ValueError("q_init must be provided")
        
        # Update max attempts
        cg.max_attempts = self.task_config.MAX_RANDOM_ATTEMPTS

        if self.use_factory:
            print("    Factory mode: generating a pick+hold goal")

            gripper = self.task_config.GRIPPERS[0]
            handle = self.task_config.HANDLES_PER_OBJECT[0][0]
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

            edge_path = bfs_edge_path(
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
                cg.configs[label] = q_next
                q_current = q_next

            cg.configs["q_goal"] = q_current
            print(f"    ✓ Goal state: {goal_state}")
            return cg.configs

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
            "grasp-tool", cg.configs["q_above"], "q_grasp_tool"
        )
        if not res:
            return cg.configs

        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        cg.project_on_node(
            "grasp-placement", cg.configs["q_grasp_tool"], "q_grasp_placement"
        )

        # 6. Generate lift config
        print("    6. Generating 'lift-tool' config...")
        res, _ = cg.generate_via_edge(
            "lift-tool", cg.configs["q_grasp_placement"], "q_lifted"
        )
        if not res:
            return cg.configs

        # 7. Project onto tool-in-air
        print("    7. Projecting onto 'tool-in-air' state...")
        cg.project_on_node("tool-in-air", cg.configs["q_lifted"], "q_tool_air")

        # 8. Generate move-tool-away config
        print("    8. Generating 'approach-dispenser' config...")
        res, _ = cg.generate_via_edge(
            "move-tool-away", cg.configs["q_tool_air"], "q_tool_away"
        )
        if not res:
            return cg.configs

        # 9. Project onto grasp
        print("    9. Projecting onto 'grasp' state...")
        cg.project_on_node("grasp", cg.configs["q_tool_away"], "q_grasp")

        # 10. Generate goal config
        cg.configs["q_goal"] = cg.configs["q_grasp"]
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
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=GraspFrameGripperTask.FREEZE_JOINT_SUBSTRINGS
    )

    # Print joint info if requested
    if show_joints:
        print_joint_info(task.robot)

    # Run task

    preferred_configs = (
        []
        if use_factory
        else [
            # "q_above",
            # "q_grasp_placement",
            # "q_tool_air",
            # "q_grasp",
        ]
    )
    result = task.run(
        visualize=visualize,
        solve=solve,
        preferred_configs=preferred_configs,
        max_iterations=5000,
    )

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
                for handles in task.task_config.HANDLES_PER_OBJECT
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
        gripper_names = task.task_config.GRIPPERS
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
