#!/usr/bin/env python3
"""Task: display and optionally reach a feasible factory state.

This is a minimal example task (modeled after
`script/spacelab/task_grasp_frame_gripper.py`) that uses
`TaskConfigurations.DisplayAllStates`.

Intent:
- Build a factory constraint graph over all default robots/objects.
- Enumerate feasible goal-state strings from the canonical VALID_PAIRS.
- Optionally project to a selected goal state (and solve if requested).
- Interactive mode for selecting pairs and visualizing configs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.tasks.grasp_sequence import (
    GraspSequencePlanner,
    InteractiveGraspSequenceBuilder,
)
from agimus_spacelab.visualization import (
    visualize_constraint_graph,
    visualize_constraint_graph_interactive,
)
from agimus_spacelab.utils.interactive import interactive_menu
from agimus_spacelab.cli import (
    add_common_arguments,
    add_advanced_arguments,
    add_grasp_sequence_arguments,
    parse_grasp_sequence,
    parse_goal_pairs,
)
from agimus_spacelab.cli.config_loader import (
    load_task_config,
    get_default_config_dir,
)
from agimus_spacelab.cli.interactive_pickers import (
    select_grasp_pairs,
    select_frozen_arms,
    browse_configurations,
)
from agimus_spacelab.planning.path_io import (
    load_paths_from_directory,
    replay_paths,
    get_path_files,
    get_num_paths,
    PathLoadError,
)


# =============================================================================
# Configuration
# =============================================================================


def initialize_task_config():
    """Load task configuration from config directory."""
    config_dir = get_default_config_dir(Path(__file__))
    return load_task_config(
        config_dir,
        "spacelab_config",
        "TaskConfigurations.DisplayAllStates",
    )


# =============================================================================
# Task Setup Utilities
# =============================================================================


def ensure_task_ready(
    task, cfg, freeze_joint_substrings, full_graph=True
) -> bool:
    """Ensure task is set up with appropriate level (lazy initialization).

    Args:
        task: DisplayStatesTask instance
        cfg: Task configuration
        freeze_joint_substrings: Joint substrings to freeze
        full_graph: If True, create full constraint graph.
                    If False, only do minimal setup (for grasp sequences).

    Returns:
        True if setup succeeded or was already done
    """
    # Check if already set up at this level
    if hasattr(task, "_setup_level"):
        if full_graph and task._setup_level == "full":
            return True
        if not full_graph and task._setup_level in ["minimal", "full"]:
            return True

    print("\n=== Initializing Task ===")
    if full_graph:
        print("Creating full constraint graph (this may take a moment)...")
    else:
        print("Minimal setup (scene + q_init, no graph)...")

    try:
        task.setup(
            validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
            projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
            freeze_joint_substrings=freeze_joint_substrings,
            skip_graph=not full_graph,
        )
        task._setup_level = "full" if full_graph else "minimal"

        # Verify critical components are initialized
        if not hasattr(task, "planner") or task.planner is None:
            raise RuntimeError("Planner not initialized")
        if not hasattr(task, "graph_builder") or task.graph_builder is None:
            raise RuntimeError("GraphBuilder not initialized")

        print("✓ Task initialized")
        return True
    except Exception as e:
        print(f"✗ Setup failed: {e}")
        import traceback

        traceback.print_exc()
        return False


def get_graph_status(task) -> str:
    """Get human-readable graph status for menu display."""
    if not hasattr(task, "_setup_level"):
        return "Not loaded"

    if task._setup_level == "minimal":
        return "Minimal (no graph)"

    # Full graph loaded - count states/edges
    try:
        states = task.graph_builder.get_states()
        edges = task.graph_builder.get_edges()
        return f"{len(states)} states, {len(edges)} edges"
    except Exception:
        return "Loaded"


# =============================================================================
# Interactive Functions
# =============================================================================


def interactive_replay_path(task, cfg, freeze_joint_substrings) -> None:
    """Interactively select a path id and replay it."""
    if not ensure_task_ready(
        task, cfg, freeze_joint_substrings, full_graph=False
    ):
        input("Press Enter to continue...")
        return

    num_paths = get_num_paths(task.planner)
    if num_paths <= 0:
        print("No paths available to replay (run Solve first).")
        input("Press Enter to continue...")
        return

    options = [f"Path {i}" for i in range(num_paths)] + ["[Exit]"]
    selected = interactive_menu(
        "Select path to replay:",
        options,
        multi_select=False,
    )

    if not selected or selected[0] >= num_paths:
        return

    path_id = int(selected[0])
    print(f"\n=== Replaying Path {path_id} ===")
    try:
        task.planner.play_path(path_id)
    except Exception as e:
        print(f"Failed to replay path {path_id}: {e}")
    input("Press Enter to continue...")


def interactive_grasp_sequence(task, cfg, freeze_joint_substrings) -> None:
    """Interactively select and plan a grasp sequence."""
    if not ensure_task_ready(
        task, cfg, freeze_joint_substrings, full_graph=False
    ):
        input("Press Enter to continue...")
        return

    builder = InteractiveGraspSequenceBuilder(
        task=task,
        task_config=cfg,
        freeze_joint_substrings=freeze_joint_substrings,
    )
    result = builder.run()

    input("\n\nPress Enter to continue...")


def interactive_load_and_replay_paths(
    task, cfg, freeze_joint_substrings
) -> None:
    """Load saved paths from directory and replay them."""
    print("\n=== Load and Replay Saved Paths ===")

    if not ensure_task_ready(
        task, cfg, freeze_joint_substrings, full_graph=False
    ):
        input("Press Enter to continue...")
        return

    # Ask for directory
    default_dir = "/tmp/grasp_sequence_paths"
    print(f"\nDefault directory: {default_dir}")
    custom_dir = input(
        "Enter directory path (or press Enter for default): "
    ).strip()

    load_dir = custom_dir if custom_dir else default_dir

    files = get_path_files(load_dir)
    print(f"\nFound files:")
    print(f"  - {len(files['native'])} native .path files")
    print(f"  - {len(files['json'])} portable .json files")

    if not files["native"] and not files["json"]:
        print(f"\nNo path files found in: {load_dir}")
        input("Press Enter to continue...")
        return

    # Ask which format to load
    if files["native"] and files["json"]:
        format_options = [
            "Load JSON waypoints (portable, works across sessions)",
            "Load native .path files (may fail if graph not loaded)",
            "Cancel",
        ]
        format_selected = interactive_menu(
            "Select format to load:",
            format_options,
            multi_select=False,
        )

        if not format_selected or format_selected[0] == 2:
            input("Press Enter to continue...")
            return

        prefer_json = format_selected[0] == 0
    else:
        prefer_json = len(files["json"]) > 0

    # Load paths
    try:
        indices = load_paths_from_directory(
            task.planner,
            load_dir,
            prefer_json=prefer_json,
        )
    except PathLoadError as e:
        if e.requires_graph:
            print(f"\n⚠ {e.message}")
            print("\nOptions:")
            print(
                "  1. Load paths in the same session where they were created"
            )
            print(
                "  2. Use JSON waypoint files instead (select portable format)"
            )
        else:
            print(f"\nError: {e.message}")
        input("\nPress Enter to continue...")
        return

    if not indices:
        print("No paths were loaded successfully.")
        input("Press Enter to continue...")
        return

    print(f"\n✓ Loaded {len(indices)} paths (indices: {indices})")

    # Offer replay
    replay_options = [
        "Replay all paths in sequence",
        "Select individual path to replay",
        "Cancel",
    ]

    selected = interactive_menu(
        "What would you like to do?",
        replay_options,
        multi_select=False,
    )

    if not selected or selected[0] == 2:
        input("Press Enter to continue...")
        return

    if selected[0] == 0:  # Replay all
        print("\nReplaying all loaded paths...")
        replay_paths(task.planner, indices)
    elif selected[0] == 1:  # Select individual
        path_options = [f"Path {idx}" for idx in indices] + ["Cancel"]
        while True:
            path_selected = interactive_menu(
                "Select path to replay:",
                path_options,
                multi_select=False,
            )

            if not path_selected or path_selected[0] >= len(indices):
                break

            idx = indices[path_selected[0]]
            print(f"\nReplaying path {idx}...")
            try:
                task.planner.play_path(idx)
            except Exception as e:
                print(f"Failed: {e}")

    input("\nPress Enter to continue...")


def interactive_main_menu(
    task, cfg, freeze_joint_substrings, configs: Dict[str, List[float]]
):
    """Interactive main menu with multiple options (lazy initialization)."""
    while True:
        graph_status = get_graph_status(task)

        options = [
            "[0] Load scene only (no graph)",
            "[1] Load full graph (required for configs/solving)",
            "[2] Browse configurations",
            "[3] Visualize constraint graph",
            "[4] Solve planning problem",
            "[5] Solve with TransitionPlanner",
            "[6] Plan grasp sequence (incremental)",
            "[7] Replay path (from ProblemSolver)",
            "[8] Load & replay saved paths (from files)",
            "[9] Exit",
        ]

        selected = interactive_menu(
            f"Select action: [Graph: {graph_status}]",
            options,
            multi_select=False,
        )

        if not selected or selected[0] == 9:  # Exit
            break

        if selected[0] == 0:  # Load scene only
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=False
            ):
                input("Press Enter to continue...")
                continue

            print("\n=== Displaying Scene ===")
            try:
                q_init = task.q_init
                if q_init:
                    task.planner.visualize(q_init)
                    print("✓ Scene displayed with initial configuration")
                else:
                    print("✗ Failed to get q_init")
            except Exception as e:
                print(f"✗ Visualization failed: {e}")
            input("Press Enter to continue...")

        elif selected[0] == 1:  # Load full graph
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=True
            ):
                input("Press Enter to continue...")
                continue

            if not configs:
                print("\n=== Generating Configurations ===")
                try:
                    q_init = task.q_init
                    if q_init:
                        configs.update(task.generate_configurations(q_init))
                        print(f"✓ Generated {len(configs)} configurations")
                    else:
                        print("✗ Failed to get q_init")
                except Exception as e:
                    print(f"✗ Configuration generation failed: {e}")
            else:
                print(
                    f"Graph already loaded with {len(configs)} configurations"
                )
            input("Press Enter to continue...")

        elif selected[0] == 2:  # Browse configurations
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=True
            ):
                input("Press Enter to continue...")
                continue

            if not configs:
                print("Generating configurations...")
                try:
                    q_init = task.q_init
                    if q_init:
                        configs.update(task.generate_configurations(q_init))
                except Exception as e:
                    print(f"Failed to generate configurations: {e}")
                    input("Press Enter to continue...")
                    continue
            browse_configurations(task, configs)

        elif selected[0] == 3:  # Visualize constraint graph
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=True
            ):
                input("Press Enter to continue...")
                continue

            print("\n=== Constraint Graph Visualization ===")
            mode_selection = interactive_menu(
                "Choose visualization mode:",
                ["Interactive window", "Static PNG", "Cancel"],
            )

            if not mode_selection or mode_selection[0] == 2:
                continue
            elif mode_selection[0] == 0:
                visualizer = visualize_constraint_graph_interactive(
                    task.graph_builder,
                    window_size=(1200, 900),
                    show_window=True,
                    blocking=True,
                )
            else:
                output_path = "/tmp/constraint_graph"
                visualize_constraint_graph(
                    task.graph_builder,
                    output_path=output_path,
                    show_png=True,
                )
                input("Press Enter to continue...")

        elif selected[0] == 4:  # Solve planning problem
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=True
            ):
                input("Press Enter to continue...")
                continue

            if not configs:
                print("Generating configurations...")
                try:
                    q_init = task.q_init
                    if q_init:
                        configs.update(task.generate_configurations(q_init))
                except Exception as e:
                    print(f"Failed to generate configurations: {e}")
                    input("Press Enter to continue...")
                    continue

            print("\n=== Solve Planning Problem ===")
            try:
                result = task.run(
                    visualize=True,
                    solve=True,
                    preferred_configs=[],
                    max_iterations=5000,
                    solve_mode="manipulation-planner",
                )
                if result.get("solved", False):
                    print("Planning succeeded!")
                else:
                    print("Planning completed (check output for details).")
            except Exception as e:
                print(f"Planning error: {e}")
            input("Press Enter to continue...")

        elif selected[0] == 5:  # Solve with TransitionPlanner
            if not ensure_task_ready(
                task, cfg, freeze_joint_substrings, full_graph=True
            ):
                input("Press Enter to continue...")
                continue

            if not configs:
                print("Generating configurations...")
                try:
                    q_init = task.q_init
                    if q_init:
                        configs.update(task.generate_configurations(q_init))
                except Exception as e:
                    print(f"Failed to generate configurations: {e}")
                    input("Press Enter to continue...")
                    continue

            print("\n=== Solve with TransitionPlanner ===")
            try:
                task.run(
                    visualize=True,
                    solve=True,
                    preferred_configs=[],
                    max_iterations=5000,
                    solve_mode="transition-planner",
                )
            except Exception as e:
                print(f"Planning error: {e}")
            input("Press Enter to continue...")

        elif selected[0] == 6:  # Plan grasp sequence
            interactive_grasp_sequence(task, cfg, freeze_joint_substrings)

        elif selected[0] == 7:  # Replay path from ProblemSolver
            interactive_replay_path(task, cfg, freeze_joint_substrings)

        elif selected[0] == 8:  # Load & replay saved paths from files
            interactive_load_and_replay_paths(
                task, cfg, freeze_joint_substrings
            )


# =============================================================================
# Task Class
# =============================================================================


class DisplayStatesTask(ManipulationTask):
    """Build a factory graph and project to a feasible goal state."""

    FREEZE_JOINT_SUBSTRINGS = []

    def __init__(self, backend: str = "corba"):
        _config_dir = get_default_config_dir(Path(__file__))
        if str(_config_dir) not in sys.path:
            sys.path.insert(0, str(_config_dir))
        from spacelab_config import DEFAULT_PATHS, JointBounds  # noqa: PLC0415
        super().__init__(
            task_name="Spacelab Manipulation: Display All Factory States",
            backend=backend,
            FILE_PATHS=DEFAULT_PATHS,
            joint_bounds=JointBounds,
        )
        self.task_config = initialize_task_config()
        self.use_factory = True
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

    def _factory_state_from_config(self, q: List[float]) -> str:
        """Best-effort detection of the current factory state name."""
        graph = self.graph

        if self.backend == "corba" and hasattr(graph, "getNode"):
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

        # Fallback: prefer a placement state if present.
        if getattr(self.task_config, "OBJECTS", None):
            place_name = f"place_{self.task_config.OBJECTS[0]}"
            if place_name in getattr(self.graph_builder, "states", {}):
                return place_name

        states = list(getattr(self.graph_builder, "states", {}).keys())
        return states[0] if states else ""

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate a configuration for each state in the factory graph."""
        cg = self.config_gen

        all_states = list(self.graph_builder.get_states().keys())
        print(f"    Graph has {len(all_states)} states:")
        for s in all_states:
            print(f"      - {s}")

        # Detect start state from initial config
        start_state = self._factory_state_from_config(q_init)
        if not start_state:
            placements = [s for s in all_states if s.startswith("place_")]
            start_state = placements[0] if placements else all_states[0]
        print(f"    Start state: {start_state}")

        # Project initial config onto start state
        cg.project_on_node(start_state, q_init, "q_init")
        q_current = cg.configs["q_init"]

        # Track which states we've generated configs for
        state_configs = {start_state: "q_init"}
        edge_topology = self.graph_builder.get_edge_topology()

        # Generate config for each state using BFS traversal
        visited = {start_state}
        queue = [start_state]

        while queue:
            current_state = queue.pop(0)
            current_q = cg.configs[state_configs[current_state]]

            for edge_name, (from_st, to_st) in edge_topology.items():
                if from_st != current_state:
                    continue
                if to_st in visited:
                    continue

                safe_name = to_st.replace(" ", "_").replace("/", "_")
                label = f"q_{safe_name}"

                try:
                    ok, q_next = cg.generate_via_edge(
                        edge_name, current_q, label
                    )
                    if ok and q_next is not None:
                        cg.configs[label] = q_next
                        state_configs[to_st] = label
                        visited.add(to_st)
                        queue.append(to_st)
                except Exception as e:
                    print(f"    ⚠ Failed {to_st}: {e}")

        # Try direct projection for unreachable states
        for state in all_states:
            if state in visited:
                continue
            safe_name = state.replace(" ", "_").replace("/", "_")
            label = f"q_{safe_name}"
            try:
                cg.project_on_node(state, q_current, label)
                state_configs[state] = label
                print(f"    ✓ {label} (direct projection)")
            except Exception as e:
                print(f"    ⚠ Could not project onto '{state}': {e}")

        # Select goal state
        goal_state = max(all_states, key=lambda s: s.count("grasps"))
        if goal_state is None:
            goal_state = list(state_configs.keys())[-1]
        cg.configs["q_goal"] = cg.configs[state_configs[goal_state]]
        print(f"    Selected goal state: {goal_state}")
        print(f"    Generated {len(cg.configs)} configurations")
        return cg.configs


# =============================================================================
# Main
# =============================================================================


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build a factory graph with DisplayAllStates and project to "
            "one feasible grasp state."
        )
    )

    # Add all argument groups
    add_common_arguments(parser)
    add_advanced_arguments(parser)
    add_grasp_sequence_arguments(parser)

    args = parser.parse_args(argv)

    cfg = initialize_task_config()

    # Convert --pair args to goal state strings
    goals_from_pairs = parse_goal_pairs(args.pair)
    all_goals = list(args.goal) + goals_from_pairs

    # Interactive mode: select pairs with arrow keys
    if args.interactive and not all_goals:
        print("\n=== Interactive Pair Selection ===")
        selected_goals = select_grasp_pairs(cfg)
        if selected_goals:
            all_goals = selected_goals

    # Apply grasp-goal filtering
    if all_goals:
        print(f"Filtering graph to {len(all_goals)} goal state(s):")
        for g in all_goals:
            print(f"  - {g}")
        cfg = cfg.with_grasp_goals(all_goals)
        print(f"  -> OBJECTS: {cfg.OBJECTS}")
        print(f"  -> GRIPPERS: {cfg.GRIPPERS}")
        print(f"  -> VALID_PAIRS: {cfg.VALID_PAIRS}")

    if args.print_goals:
        goals = list(cfg.feasible_grasp_goal_states())
        if args.limit and args.limit > 0:
            goals = goals[: args.limit]
        for s in goals:
            print(s)
        return 0

    freeze_joint_substrings = list(DisplayStatesTask.FREEZE_JOINT_SUBSTRINGS)
    if args.interactive:
        print("\n=== Interactive Locked-Arm Selection ===")
        freeze_joint_substrings = select_frozen_arms(freeze_joint_substrings)
        print(
            "Using freeze_joint_substrings: "
            f"{freeze_joint_substrings if freeze_joint_substrings else '[]'}"
        )

    task = DisplayStatesTask(backend=args.backend)
    task.task_config = cfg

    # Handle --load-paths mode
    if args.load_paths:
        print("\n=== Load and Replay Saved Paths ===")
        print(f"Loading from: {args.load_paths}")

        task.setup(
            validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
            projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
            freeze_joint_substrings=freeze_joint_substrings,
            skip_graph=True,
        )

        try:
            indices = load_paths_from_directory(
                task.planner,
                args.load_paths,
                prefer_json=True,
            )
        except PathLoadError as e:
            print(f"\n⚠ Error: {e.message}")
            return 1

        if not indices:
            print("No paths were loaded successfully.")
            return 1

        print(f"\n✓ Loaded {len(indices)} paths")
        print("Replaying all paths...")
        result = replay_paths(task.planner, indices)
        print("Replay complete.")
        return 0 if result["success"] else 1

    # Interactive mode
    if args.interactive:
        print("\n=== Interactive Mode ===")
        print("Graph will be loaded on-demand (faster startup)")
        configs = {}
        interactive_main_menu(task, cfg, freeze_joint_substrings, configs)
        return 0

    # Handle --grasp-sequence mode
    if args.grasp_sequence:
        print("\n=== Grasp Sequence Planning Mode ===")

        task.setup(
            validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
            projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
            freeze_joint_substrings=freeze_joint_substrings,
            skip_graph=True,
        )

        grasp_sequence = parse_grasp_sequence(args.grasp_sequence)
        if not grasp_sequence:
            print("Error: No valid grasps in sequence")
            return 1

        print(f"Sequence: {grasp_sequence}")

        q_init = task.q_init
        if not q_init:
            print("Error: Failed to build q_init")
            return 1

        if not args.no_viz:
            print("\nStarting visualization...")
            try:
                task.planner.visualize(q_init)
                print("   ✓ Initial configuration displayed")
            except Exception as e:
                print(f"   ⚠ Visualization failed: {e}")

        try:
            seq_planner = GraspSequencePlanner(
                graph_builder=task.graph_builder,
                config_gen=task.config_gen,
                planner=task.planner,
                task_config=task.task_config,
                backend=task.backend,
                pyhpp_constraints=getattr(task, "pyhpp_constraints", {}),
                graph_constraints=getattr(task, "_graph_constraints", None),
                auto_save_dir=getattr(args, "auto_save_dir", None),
            )

            seq_result = seq_planner.plan_sequence(
                grasp_sequence=grasp_sequence,
                q_init=q_init,
                verbose=True,
            )

            if seq_result["success"]:
                print(seq_planner.get_phase_summary())
                return 0
            else:
                print("Sequence planning failed")
                return 1

        except Exception as e:
            print(f"Sequence planning error: {e}")
            import traceback

            traceback.print_exc()
            return 1
    else:
        # Normal mode: setup with full graph creation
        task.setup(
            validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
            projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
            freeze_joint_substrings=freeze_joint_substrings,
            skip_graph=False,
        )

    result = task.run(
        visualize=not args.no_viz,
        solve=args.solve,
        preferred_configs=[],
        max_iterations=5000,
        solve_mode=args.solve_mode,
    )

    configs = result.get("configs", {})

    # Visualize constraint graph if requested
    if args.graph_viz and not args.interactive:
        print("\n=== Constraint Graph Visualization ===")
        output_path = args.graph_viz
        visualize_constraint_graph(
            task.graph_builder,
            output_path=output_path,
            show_png=True,
        )

    # Handle --show
    elif args.show:
        if "all" in args.show:
            print("\nAvailable configurations:")
            for name in sorted(configs.keys()):
                print(f"  - {name}")
            print("\nTo visualize, run with --show <config_name>")
        else:
            for name in args.show:
                if name in configs:
                    print(f"\nVisualizing '{name}'...")
                    task.planner.visualize(configs[name])
                    input(f"Press Enter to continue (showing '{name}')...")
                else:
                    print(
                        f"Warning: Config '{name}' not found. "
                        f"Available: {list(configs.keys())}"
                    )

    # Print summary
    print("\n" + "=" * 70)
    print("Generated Configurations")
    print("=" * 70)
    for name in sorted(configs.keys()):
        print(f"  {name}")
    print("\nTo visualize a config:")
    print("  task.planner.visualize(result['configs']['<name>'])")
    print("Or re-run with: --show <name> or --interactive")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
