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
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.visualization import visualize_constraint_graph


# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))


def initialize_task_config():
    from spacelab_config import TaskConfigurations  # noqa: E402

    cfg = TaskConfigurations.DisplayAllStates
    cfg.init_poses()
    return cfg


# =============================================================================
# Interactive Terminal Menu
# =============================================================================


def _clear_line():
    """Clear current line in terminal."""
    sys.stdout.write("\033[K")


def _move_cursor_up(n: int = 1):
    """Move cursor up n lines."""
    if n > 0:
        sys.stdout.write(f"\033[{n}A")


def _hide_cursor():
    sys.stdout.write("\033[?25l")
    sys.stdout.flush()


def _show_cursor():
    sys.stdout.write("\033[?25h")
    sys.stdout.flush()


def interactive_menu(
    title: str,
    options: List[str],
    multi_select: bool = False,
    selected: Optional[List[int]] = None,
) -> List[int]:
    """Simple arrow-key menu for terminal selection.

    Args:
        title: Menu title to display
        options: List of option strings
        multi_select: If True, allow multiple selections with Space
        selected: Initial selected indices (for multi_select)

    Returns:
        List of selected indices (single item list if not multi_select)
    """
    if not options:
        return []

    # Try to import getch for arrow key support
    try:
        import tty
        import termios

        def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                # Handle escape sequences (arrow keys)
                if ch == "\x1b":
                    ch += sys.stdin.read(2)
                return ch
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

        has_getch = True
    except ImportError:
        has_getch = False

    if not has_getch:
        # Fallback to numbered selection
        return _numbered_menu(title, options, multi_select, selected)

    cursor = 0
    checked = set(selected or [])

    _hide_cursor()
    try:
        while True:
            # Print menu
            print(f"\n{title}")
            print("Use ↑/↓ to navigate, ", end="")
            if multi_select:
                print("Space to toggle, Enter to confirm, q to quit")
            else:
                print("Enter to select, q to quit")

            for i, opt in enumerate(options):
                prefix = "→ " if i == cursor else "  "
                if multi_select:
                    check = "[x]" if i in checked else "[ ]"
                    print(f"{prefix}{check} {opt}")
                else:
                    print(f"{prefix}{opt}")

            # Get input
            key = getch()

            # Move cursor up to redraw
            lines_to_clear = len(options) + 3
            _move_cursor_up(lines_to_clear)

            if key == "\x1b[A":  # Up arrow
                cursor = (cursor - 1) % len(options)
            elif key == "\x1b[B":  # Down arrow
                cursor = (cursor + 1) % len(options)
            elif key == " " and multi_select:  # Space - toggle
                if cursor in checked:
                    checked.discard(cursor)
                else:
                    checked.add(cursor)
            elif key in ("\r", "\n"):  # Enter
                # Clear menu lines
                for _ in range(lines_to_clear):
                    _clear_line()
                    print()
                _move_cursor_up(lines_to_clear)

                if multi_select:
                    return sorted(checked)
                return [cursor]
            elif key in ("q", "Q", "\x03"):  # q or Ctrl+C
                for _ in range(lines_to_clear):
                    _clear_line()
                    print()
                _move_cursor_up(lines_to_clear)
                return []
    finally:
        _show_cursor()


def _numbered_menu(
    title: str,
    options: List[str],
    multi_select: bool = False,
    selected: Optional[List[int]] = None,
) -> List[int]:
    """Fallback numbered menu when arrow keys aren't available."""
    print(f"\n{title}")
    for i, opt in enumerate(options):
        print(f"  [{i}] {opt}")

    if multi_select:
        print("\nEnter numbers separated by commas (e.g., 0,2,3),")
        print("or 'q' to quit:")
    else:
        print("\nEnter number, or 'q' to quit:")

    try:
        choice = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        return []

    if choice.lower() == "q":
        return []

    try:
        if multi_select:
            indices = [int(x.strip()) for x in choice.split(",")]
            return [i for i in indices if 0 <= i < len(options)]
        else:
            idx = int(choice)
            if 0 <= idx < len(options):
                return [idx]
    except ValueError:
        pass

    print("Invalid selection")
    return []


def interactive_select_pairs(cfg) -> List[str]:
    """Interactively select gripper-handle pairs."""
    all_goals = list(cfg.feasible_grasp_goal_states())
    if not all_goals:
        print("No valid pairs available.")
        return []

    # Add "Select All" option at the top
    menu_options = ["[Select All]"] + all_goals

    selected = interactive_menu(
        "Select grasp pair(s) to include in graph:",
        menu_options,
        multi_select=True,
    )

    if not selected:
        print("No pairs selected, using all available pairs.")
        return []

    # Check if "Select All" (index 0) was selected
    if 0 in selected:
        print(f"Selected all {len(all_goals)} pairs.")
        return all_goals

    # Map back to actual goals (subtract 1 for the "Select All" option)
    return [all_goals[i - 1] for i in selected if i > 0]


def interactive_select_frozen_arms(
    default_substrings: List[str],
) -> List[str]:
    """Interactively select joint-name substrings to freeze during planning.

    Options are intentionally simple and map directly to substrings searched in
    joint names.

    Args:
        default_substrings: Substrings to use if the user quits.

    Returns:
        List of selected substrings. If the user quits without selecting,
        returns default_substrings.
    """
    options = ["vispa_", "vispa2", "ur10"]
    initial_selected = [
        i for i, opt in enumerate(options) if opt in set(default_substrings)
    ]

    selected = interactive_menu(
        "Select arm(s) to lock (freeze joints):",
        options,
        multi_select=True,
        selected=initial_selected,
    )

    if not selected:
        return list(default_substrings)

    return [options[i] for i in selected if 0 <= i < len(options)]


def interactive_visualize_configs(task, configs: Dict[str, List[float]]):
    """Interactively select and visualize configurations."""
    if not configs:
        print("No configurations available.")
        return

    config_names = sorted(configs.keys())

    while True:
        selected = interactive_menu(
            "Select configuration to visualize (q to quit):",
            config_names + ["[Exit]"],
            multi_select=False,
        )

        if not selected or selected[0] >= len(config_names):
            break

        name = config_names[selected[0]]
        print(f"\nVisualizing '{name}'...")
        task.planner.visualize(configs[name])
        input("Press Enter to continue...")


def _get_num_paths(task) -> int:
    """Best-effort number of available stored paths for replay."""
    planner = getattr(task, "planner", None)
    if planner is None:
        return 0

    stored = getattr(planner, "_stored_paths", None)
    if isinstance(stored, list):
        return len(stored)

    # CORBA backend exposes ProblemSolver via planner.ps
    ps = getattr(planner, "ps", None)
    if ps is not None and hasattr(ps, "numberPaths"):
        try:
            return int(ps.numberPaths())
        except Exception:
            return 0

    # PyHPP backend typically stores a single path in planner.path
    path = getattr(planner, "path", None)
    return 1 if path is not None else 0


def interactive_replay_path(task) -> None:
    """Interactively select a path id and replay it."""
    num_paths = _get_num_paths(task)
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


def interactive_grasp_sequence(task, cfg) -> None:
    """Interactively select and plan a grasp sequence."""
    print("\n=== Interactive Grasp Sequence Planning ===")

    # Get all possible grasps from config
    all_grasps = []
    valid_pairs = getattr(cfg, "VALID_PAIRS", {})
    for gripper, handles in valid_pairs.items():
        for handle in handles:
            all_grasps.append((gripper, handle))

    if not all_grasps:
        print("No valid grasps available in config.")
        input("Press Enter to continue...")
        return

    # Format for display
    grasp_options = [
        f"{gripper} → {handle}"
        for gripper, handle in all_grasps
    ] + ["[Done - Start Planning]"]

    selected_sequence = []

    while True:
        # Show current sequence
        if selected_sequence:
            print("\nCurrent sequence:")
            for i, (g, h) in enumerate(selected_sequence, 1):
                print(f"  {i}. {g} → {h}")
        else:
            print("\nSequence is empty. Select grasps to add.")

        # Select next grasp
        selected = interactive_menu(
            "Select next grasp to add (or Done to plan):",
            grasp_options,
            multi_select=False,
        )

        if not selected:
            break

        if selected[0] >= len(all_grasps):  # Done button
            break

        selected_sequence.append(all_grasps[selected[0]])

    if not selected_sequence:
        print("No grasps selected.")
        input("Press Enter to continue...")
        return

    print(f"\nPlanning sequence of {len(selected_sequence)} grasps...")

    # Select frozen arms mode
    frozen_mode_options = [
        "auto - Freeze all arms except active gripper's arm",
        "manual - Specify which arms to freeze per phase",
        "interactive - Prompt per phase which arms to freeze",
        "global - Use task's global locked joint constraints",
        "none - No locked joint constraints",
    ]

    print("\nSelect locked joint constraint mode:")
    mode_selected = interactive_menu(
        "Choose mode:",
        frozen_mode_options,
        multi_select=False,
    )

    if not mode_selected:
        print("Cancelled.")
        input("Press Enter to continue...")
        return

    mode_map = ["auto", "manual", "interactive", "global", "none"]
    frozen_arms_mode = mode_map[mode_selected[0]]
    per_phase_frozen_arms = None

    # Manual mode: collect per-phase specifications
    if frozen_arms_mode == "manual":
        per_phase_frozen_arms = {}
        arm_keywords = ["ur10", "vispa_", "vispa2"]

        for phase_idx, (gripper, handle) in enumerate(selected_sequence):
            print(f"\nPhase {phase_idx + 1}: {gripper} → {handle}")
            print("Select arm(s) to freeze:")

            selected_arms = interactive_menu(
                "Select arms:",
                arm_keywords + ["[None - No Locking]"],
                multi_select=True,
            )

            arm_count = len(arm_keywords)
            if selected_arms and selected_arms[0] < arm_count:
                frozen = [
                    arm_keywords[i] for i in selected_arms if i < arm_count
                ]
                per_phase_frozen_arms[phase_idx] = frozen
                print(f"  Freezing: {frozen}")
            else:
                print("  No locked joints for this phase")

    # Define interactive arm selector callback for interactive mode
    def interactive_arm_selector(phase_idx, gripper, arm_keywords):
        """Callback for interactive arm selection per phase."""
        print(f"\n  Select arms to freeze for Phase {phase_idx + 1}:")
        print(f"  Active gripper: {gripper}")

        selected = interactive_menu(
            "Select arm(s) to freeze:",
            arm_keywords + ["[None - No Locking]"],
            multi_select=True,
        )

        arm_count = len(arm_keywords)
        if selected and selected[0] < arm_count:
            return [arm_keywords[i] for i in selected if i < arm_count]
        return []

    try:
        # Create GraspSequencePlanner
        planner = GraspSequencePlanner(
            graph_builder=task.graph_builder,
            config_gen=task.config_gen,
            planner=task.planner,
            task_config=task.task_config,
            backend=task.backend,
            pyhpp_constraints=getattr(task, "pyhpp_constraints", {}),
            graph_constraints=getattr(task, "_graph_constraints", None),
        )

        # Set interactive callback if in interactive mode
        if frozen_arms_mode == "interactive":
            planner.interactive_arm_selector_callback = (
                interactive_arm_selector
            )

        # Get initial config
        q_init = task.config_gen.configs.get("q_init")
        if q_init is None:
            print("Error: q_init not found")
            input("Press Enter to continue...")
            return

        # Plan sequence
        result = planner.plan_sequence(
            grasp_sequence=selected_sequence,
            q_init=q_init,
            frozen_arms_mode=frozen_arms_mode,
            per_phase_frozen_arms=per_phase_frozen_arms,
            verbose=True,
        )

        if result["success"]:
            print("\n" + "=" * 70)
            print("Sequence planning succeeded!")
            print(planner.get_phase_summary())
            print("\nReplay sequence? (y/n)")
            if input("> ").lower() == "y":
                planner.replay_sequence()
        else:
            print("Sequence planning failed.")

    except Exception as e:
        print(f"\nSequence planning error: {e}")
        import traceback
        traceback.print_exc()

        # Check if sequence can be resumed
        if hasattr(planner, 'get_resumable_state'):
            resume_state = planner.get_resumable_state()
            if resume_state:
                print("\n" + "=" * 70)
                print("Planning Failed - Partial Progress Saved")
                print("=" * 70)
                print(f"Failed at: Phase {resume_state['phase_idx'] + 1}, "
                      f"Edge {resume_state['edge_idx'] + 1}")
                print(f"Completed phases: {resume_state['completed_phases']}")
                print(f"Completed edges in current phase: "
                      f"{resume_state['completed_edges_in_phase']} of "
                      f"{resume_state['total_edges_in_phase']}")
                print(f"Error: {resume_state['error']}")

                # Show partial results summary
                print(planner.get_phase_summary())

                # Offer resume options in a loop
                while True:
                    resume_state = planner.get_resumable_state()
                    if not resume_state:
                        # Planning succeeded or no more resumable state
                        break

                    print("\n" + "=" * 70)
                    print("Resume Options:")
                    print("=" * 70)
                    print("[R] Replay completed paths")
                    print("[1] Retry from failed edge")
                    print("[2] Retry from start of failed phase")
                    print("[3] Retry with increased timeout (2x)")
                    print("[4] Retry with increased max iterations")
                    print("[Q] Quit to menu")

                    choice = input("\nSelect option: ").strip().upper()

                    if choice == "Q":
                        break
                    elif choice == "R":
                        print("\nReplaying completed paths...")
                        planner.replay_sequence()
                    elif choice in ["1", "2", "3", "4"]:
                        retry_edge = -1 if choice == "1" else 0
                        timeout_mult = 2.0 if choice == "3" else 1.0

                        edge_or_phase = (
                            "failed edge" if choice == "1" else "failed phase"
                        )
                        print(f"\nRetrying {edge_or_phase}...")

                        timeout = None
                        max_iters = None

                        if choice == "3":
                            print("  Using 2x timeout")
                            timeout = 60.0 * timeout_mult
                        elif choice == "4":
                            print("\nIncrease max iterations:")
                            print("[1] 2x (double current)")
                            print("[2] Custom value")
                            iter_choice = input("Select: ").strip()

                            if iter_choice == "1":
                                max_iters = 5000 * 2
                                print(
                                    f"  Using 2x max iterations: {max_iters}"
                                )
                            elif iter_choice == "2":
                                try:
                                    custom_val = input(
                                        "Enter max iterations: "
                                    ).strip()
                                    max_iters = int(custom_val)
                                    print(
                                        f"  Using max iterations: {max_iters}"
                                    )
                                except ValueError:
                                    print("  Invalid input, using default")
                                    max_iters = None

                        try:
                            result = planner.resume_sequence(
                                retry_from_edge=retry_edge,
                                timeout_per_edge=timeout,
                                max_iterations_per_edge=max_iters,
                                frozen_arms_mode=frozen_arms_mode,
                                per_phase_frozen_arms=per_phase_frozen_arms,
                                verbose=True,
                            )

                            if result["success"]:
                                print("\n" + "=" * 70)
                                print("Resume succeeded!")
                                print(planner.get_phase_summary())
                                print("\nReplay full sequence? (y/n)")
                                if input("> ").lower() == "y":
                                    planner.replay_sequence()
                                # Exit the resume loop on success
                                break
                        except Exception as e2:
                            print(f"\nResume failed: {e2}")
                            # Show updated failure info
                            resume_state_new = planner.get_resumable_state()
                            if resume_state_new:
                                print("\n" + "=" * 70)
                                print("Updated Failure Status")
                                print("=" * 70)
                                print(f"Failed at: Phase {resume_state_new['phase_idx'] + 1}, "
                                      f"Edge {resume_state_new['edge_idx'] + 1}")
                                print(f"Error: {resume_state_new['error']}")
                                print(planner.get_phase_summary())
                                print("\nYou can retry again with different options.")
                            else:
                                print("\nNo resumable state available.")
                                break

    input("\nPress Enter to continue...")


def interactive_main_menu(task, configs: Dict[str, List[float]]):
    """Interactive main menu with multiple options."""
    while True:
        options = [
            "Browse configurations",
            "Visualize constraint graph",
            "Solve planning problem",
            "Solve with TransitionPlanner",
            "Plan grasp sequence (incremental)",
            "Replay path",
            "[Exit]",
        ]

        selected = interactive_menu(
            "Select action:",
            options,
            multi_select=False,
        )

        if not selected or selected[0] == 6:  # Exit
            break

        if selected[0] == 0:  # Browse configurations
            interactive_visualize_configs(task, configs)

        elif selected[0] == 1:  # Visualize constraint graph
            print("\n=== Constraint Graph Visualization ===")
            output_path = "/tmp/constraint_graph"
            visualize_constraint_graph(
                task.graph_builder,
                output_path=output_path,
                show_png=True,
            )
            input("Press Enter to continue...")

        elif selected[0] == 2:  # Solve planning problem
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

        elif selected[0] == 3:  # Solve with TransitionPlanner
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

        elif selected[0] == 4:  # Plan grasp sequence
            interactive_grasp_sequence(task, task.task_config)

        elif selected[0] == 5:  # Replay path
            interactive_replay_path(task)


class DisplayStatesTask(ManipulationTask):
    """Build a factory graph and project to a feasible goal state."""
    # VISPA arms are not used in this task; keep them fixed during planning.
    FREEZE_JOINT_SUBSTRINGS = []

    def __init__(self, backend: str = "corba"):
        super().__init__(
            task_name="Spacelab Manipulation: Display All Factory States",
            backend=backend,
        )
        self.task_config = initialize_task_config()
        self.use_factory = True
        self.pyhpp_constraints = {}

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
            # Fallback: use first placement state
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

            # Find all edges from current state
            for edge_name, (from_st, to_st) in edge_topology.items():
                if from_st != current_state:
                    continue
                if to_st in visited:
                    continue

                # Try to generate config via this edge
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

        # Also try direct projection for any states we couldn't reach
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

        goal_state = None

        # Select state with most "grasps" occurrences
        goal_state = max(all_states, key=lambda s: s.count("grasps"))
        if goal_state is None:
            # Use last generated config
            goal_state = list(state_configs.keys())[-1]
        cg.configs["q_goal"] = cg.configs[state_configs[goal_state]]
        print(f"    Selected goal state: {goal_state}")
        print(f"    Generated {len(cg.configs)} configurations")
        return cg.configs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build a factory graph with DisplayAllStates and project to "
            "one feasible grasp state."
        )
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization",
    )
    parser.add_argument(
        "--solve",
        action="store_true",
        help="Attempt to solve planning problem",
    )
    parser.add_argument(
        "--solve-mode",
        type=str,
        default="manipulation-planner",
        choices=["manipulation-planner", "transition-planner"],
        help="Planning algorithm to use when --solve is set",
    )
    parser.add_argument(
        "--goal",
        action="append",
        default=[],
        metavar="STATE",
        help=(
            "Target grasp state(s) to include in graph, e.g. "
            "'spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool'. "
            "Reduces graph size by filtering VALID_PAIRS. Repeatable."
        ),
    )
    parser.add_argument(
        "--pair",
        action="append",
        default=[],
        metavar="GRIPPER:HANDLE",
        help=(
            "Add a gripper-handle pair to VALID_PAIRS, e.g. "
            "'spacelab/g_ur10_tool:frame_gripper/h_FG_tool'. Repeatable. "
            "If specified, only these pairs are included in the graph."
        ),
    )
    parser.add_argument(
        "--show",
        action="append",
        default=[],
        metavar="CONFIG",
        help=(
            "After running, visualize these configs interactively, e.g. "
            "'q_init', 'q_goal'. Repeatable. Use 'all' to list available."
        ),
    )
    parser.add_argument(
        "--print-goals",
        action="store_true",
        help="Print feasible goal-state strings and exit",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=50,
        help="Limit printed goal states (with --print-goals)",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help=(
            "Interactive mode: use arrow-key menus to select pairs and "
            "browse configurations"
        ),
    )
    parser.add_argument(
        "--graph-viz",
        nargs="?",
        const="/tmp/constraint_graph",
        metavar="PATH",
        help=(
            "Visualize constraint graph as PNG. Optionally specify output "
            "path (default: /tmp/constraint_graph). Requires matplotlib."
        ),
    )
    parser.add_argument(
        "--grasp-sequence",
        type=str,
        metavar="SEQUENCE",
        help=(
            "Plan a sequence of grasps using incremental phase graphs. "
            "Format: GRIPPER1:HANDLE1,GRIPPER2:HANDLE2,... "
            "Example: 'spacelab/g_ur10_tool:frame_gripper/h_FG_tool,"
            "spacelab/g_vispa:panel/h_panel'. Uses TransitionPlanner for "
            "each phase with minimal graph regeneration."
        ),
    )

    args = parser.parse_args(argv)

    cfg = initialize_task_config()

    # Convert --pair args to goal state strings
    goals_from_pairs = []
    for pair in args.pair:
        if ":" in pair:
            gripper, handle = pair.split(":", 1)
            goals_from_pairs.append(
                f"{gripper.strip()} grasps {handle.strip()}"
            )
        else:
            print(
                f"Warning: Invalid --pair format '{pair}', "
                "expected 'GRIPPER:HANDLE'"
            )

    # Combine --goal and --pair into one list
    all_goals = list(args.goal) + goals_from_pairs

    # Interactive mode: select pairs with arrow keys
    if args.interactive and not all_goals:
        print("\n=== Interactive Pair Selection ===")
        selected_goals = interactive_select_pairs(cfg)
        if selected_goals:
            all_goals = selected_goals

    # Apply grasp-goal filtering (most impactful for graph size)
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
        freeze_joint_substrings = interactive_select_frozen_arms(
            freeze_joint_substrings
        )
        print(
            "Using freeze_joint_substrings: "
            f"{freeze_joint_substrings if freeze_joint_substrings else '[]'}"
        )

    task = DisplayStatesTask(backend=args.backend)
    task.task_config = cfg  # Use filtered config

    # Handle --grasp-sequence mode
    if args.grasp_sequence:
        print("\n=== Grasp Sequence Planning Mode ===")

        # Setup without creating full graph (skip_graph=True)
        # GraspSequencePlanner will build minimal phase graphs
        task.setup(
            validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
            projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
            freeze_joint_substrings=freeze_joint_substrings,
            skip_graph=True,
        )

        # Parse sequence: "gripper1:handle1,gripper2:handle2,..."
        grasp_sequence = []
        for pair_str in args.grasp_sequence.split(","):
            pair_str = pair_str.strip()
            if ":" not in pair_str:
                print(
                    f"Warning: Invalid pair format '{pair_str}', "
                    "expected 'GRIPPER:HANDLE'"
                )
                continue
            gripper, handle = pair_str.split(":", 1)
            grasp_sequence.append((gripper.strip(), handle.strip()))

        if not grasp_sequence:
            print("Error: No valid grasps in sequence")
            return 1

        print(f"Sequence: {grasp_sequence}")

        # Get q_init from task (no full graph needed)
        q_init = task.q_init
        if not q_init:
            print("Error: Failed to build q_init")
            return 1

        # Optional visualization
        if not args.no_viz:
            print("\nStarting visualization...")
            try:
                task.planner.visualize(q_init)
                print("   ✓ Initial configuration displayed")
            except Exception as e:
                print(f"   ⚠ Visualization failed: {e}")

        # Create and run sequence planner
        try:
            seq_planner = GraspSequencePlanner(
                graph_builder=task.graph_builder,
                config_gen=task.config_gen,
                planner=task.planner,
                task_config=task.task_config,
                backend=task.backend,
                pyhpp_constraints=getattr(task, "pyhpp_constraints", {}),
                graph_constraints=getattr(task, "_graph_constraints", None),
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

    # Visualize constraint graph if requested (non-interactive)
    if args.graph_viz and not args.interactive:
        print("\n=== Constraint Graph Visualization ===")
        output_path = args.graph_viz
        visualize_constraint_graph(
            task.graph_builder,
            output_path=output_path,
            show_png=True,
        )

    # Interactive mode: main menu with multiple options
    if args.interactive:
        print("\n=== Interactive Mode ===")
        interactive_main_menu(task, configs)

    # Handle --show: display specific configurations
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

    # Print summary of available configs
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
