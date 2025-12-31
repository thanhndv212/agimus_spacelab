#!/usr/bin/env python3
"""Task: display and optionally reach a feasible factory state.

This is a minimal example task (modeled after
`script/spacelab/task_grasp_frame_gripper.py`) that uses
`TaskConfigurations.DisplayAllStates`.

Intent:
- Build a factory constraint graph over all default robots/objects.
- Enumerate feasible goal-state strings from the canonical VALID_PAIRS.
- Optionally project to a selected goal state (and solve if requested).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

from agimus_spacelab.planning.config import bfs_edge_path
from agimus_spacelab.tasks import ManipulationTask


# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))


def initialize_task_config():
    from spacelab_config import TaskConfigurations  # noqa: E402

    cfg = TaskConfigurations.DisplayAllStates
    cfg.init_poses()
    return cfg


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
        cg = self.config_gen

        # In this task we just need one feasible goal to demonstrate
        # state naming.
        all_states = list(self.graph_builder.get_states().keys())

        goal_state = ""
        desired = list(self.task_config.feasible_grasp_goal_states())
        feasible = [s for s in desired if s in all_states]
        if feasible:
            goal_state = feasible[0]
        else:
            grasps = [s for s in all_states if "grasps" in s]
            if grasps:
                goal_state = grasps[0]

        if not goal_state:
            raise RuntimeError(
                "Could not find any factory state containing 'grasps'. "
                "This likely means the factory graph has no grasp states "
                "for the loaded models."
            )

        start_state = self._factory_state_from_config(q_init)
        if not start_state:
            raise RuntimeError("Could not determine start state")

        cg.project_on_node(start_state, q_init, "q_init")

        edge_path = bfs_edge_path(
            start_state,
            goal_state,
            self.graph_builder.get_edge_topology(),
        )
        if not edge_path:
            q_current = cg.configs["q_init"]
            cg.project_on_node(goal_state, q_current, "q_goal")
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

    args = parser.parse_args(argv)

    cfg = initialize_task_config()
    if args.print_goals:
        goals = list(cfg.feasible_grasp_goal_states())
        if args.limit and args.limit > 0:
            goals = goals[: args.limit]
        for s in goals:
            print(s)
        return 0

    task = DisplayStatesTask(backend=args.backend)
    task.setup(
        validation_step=getattr(cfg, "PATH_VALIDATION_STEP", 0.01),
        projector_step=getattr(cfg, "PATH_PROJECTOR_STEP", 0.1),
    )

    task.run(
        visualize=not args.no_viz,
        solve=args.solve,
        preferred_configs=[],
        max_iterations=5000,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
