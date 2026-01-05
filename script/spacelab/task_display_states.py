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
                    return sorted(checked) if checked else [cursor]
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


class DisplayStatesTask(ManipulationTask):
    """Build a factory graph and project to a feasible goal state."""

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

        # Set q_goal to a grasp state if available
        desired = list(self.task_config.feasible_grasp_goal_states())
        feasible = [s for s in desired if s in state_configs]
        if feasible:
            cg.configs["q_goal"] = cg.configs[state_configs[feasible[0]]]
        elif state_configs:
            # Use last generated config
            last_label = list(state_configs.values())[-1]
            cg.configs["q_goal"] = cg.configs[last_label]

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

    task = DisplayStatesTask(backend=args.backend)
    task.task_config = cfg  # Use filtered config
    task.setup(
        validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
        projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
    )

    result = task.run(
        visualize=not args.no_viz,
        solve=args.solve,
        preferred_configs=[],
        max_iterations=5000,
    )

    configs = result.get("configs", {})

    # Interactive mode: browse configurations with arrow keys
    if args.interactive and configs:
        print("\n=== Interactive Configuration Viewer ===")
        interactive_visualize_configs(task, configs)

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
