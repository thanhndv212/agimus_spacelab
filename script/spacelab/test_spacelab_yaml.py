#!/usr/bin/env python3
"""SpaceLab grasp-sequence test — driven by YAML config.

This script is the YAML-based successor to ``test_pyhpp_planning.py``.
All robot/object paths, initial poses, joint bounds, gripper names, and
valid pairs come from ``script/config/spacelab_config.yaml``.  No Python
file in the framework imports SpaceLab-specific configuration.

Architecture:
    spacelab_config.yaml
        └─ YamlTaskLoader ──► file_paths       → SceneBuilder
                           ──► joint_bounds_class
                           ──► task_config      → ManipulationTask
                           ──► build_initial_config()

Usage::

    python script/spacelab/test_spacelab_yaml.py [--backend pyhpp|corba]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List

from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.config.yaml_loader import YamlTaskLoader

# ---------------------------------------------------------------------------
# Load YAML config (module-level singleton — cheap / idempotent)
# ---------------------------------------------------------------------------

_YAML_PATH = Path(__file__).parent.parent / "config" / "spacelab_config.yaml"
_loader = YamlTaskLoader(_YAML_PATH)


# ---------------------------------------------------------------------------
# Task class
# ---------------------------------------------------------------------------

class SpaceLabYamlTask(ManipulationTask):
    """SpaceLab grasp-sequence task driven entirely by YAML configuration.

    Demonstrates:
    * Framework-agnostic file_paths / joint_bounds loaded from YAML.
    * task_config built by YamlTaskLoader — zero SpaceLab-specific imports.
    * build_initial_config() reads initial poses from YAML (no InitialConfigurations).
    * Factory-mode constraint graph for ``g_ur10_tool`` → ``frame_gripper/h_FG_tool``.
    * VISPA joints are frozen via FREEZE_JOINT_SUBSTRINGS (same as original test).
    """

    FREEZE_JOINT_SUBSTRINGS: List[str] = ["vispa_", "vispa2"]

    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name="SpaceLab YAML Grasp Sequence",
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        # Filter to a single grasp so the factory graph stays minimal.
        self.task_config = _loader.task_config.with_grasp_goals([
            "spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool"
        ])
        self.use_factory = True
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> List[float]:
        """Build initial config from YAML for the active object subset."""
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


# ---------------------------------------------------------------------------
# Run logic (extracted from main to allow re-use in tests)
# ---------------------------------------------------------------------------

def run_task(backend: str = "pyhpp") -> bool:
    """Initialise and run the SpaceLab YAML grasp-sequence test.

    Returns True on planning success, False otherwise.
    """
    task = SpaceLabYamlTask(backend=backend)

    print("\n" + "=" * 70)
    print("SpaceLab YAML Grasp Sequence Test")
    print("=" * 70)
    print(f"  Backend   : {backend}")
    print(f"  YAML cfg  : {_YAML_PATH.name}")
    print(f"  Grasp pair: spacelab/g_ur10_tool → frame_gripper/h_FG_tool")
    print(f"  Frozen    : {task.FREEZE_JOINT_SUBSTRINGS}")
    print("=" * 70 + "\n")

    # ------------------------------------------------------------------
    # 1. Setup scene + frozen joints + factory graph
    # ------------------------------------------------------------------
    print("Initialising task...")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,   # GraspSequencePlanner builds phase graphs
        )
    except Exception as e:
        print(f"✗ Setup failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    # ------------------------------------------------------------------
    # 2. Disable known noisy collision pairs (mirrors test_pyhpp_planning)
    # ------------------------------------------------------------------
    print("\nDisabling problematic collision pairs...")
    collision_pairs = [
        ("ground_demo/link_NYX_0", "spacelab/ur10_link_4"),
        ("ground_demo/link_NYX_0", "spacelab/vispa_link_4"),
        ("ground_demo/link_NYX_0", "spacelab/vispa2_link_4"),
        ("ground_demo/link_NYX_0", "spacelab/ur10_link_3"),
        ("ground_demo/link_NYX_0", "spacelab/vispa_link_3"),
        ("ground_demo/link_NYX_0", "spacelab/vispa2_link_3"),
    ]
    removed = 0
    for obstacle, joint in collision_pairs:
        try:
            task.scene_builder.disable_collision_pair(
                obstacle_name=obstacle, joint_name=joint
            )
            removed += 1
        except Exception as e:
            print(f"   Could not disable {obstacle} <-> {joint}: {e}")
    print(f"✓ Disabled {removed}/{len(collision_pairs)} collision pair(s)")

    # ------------------------------------------------------------------
    # 3. Retrieve initial config
    # ------------------------------------------------------------------
    q_init = task.q_init
    if not q_init:
        print("✗ Failed to get initial configuration")
        return False
    print(f"\n✓ Initial config: {len(q_init)} DOF")

    # Optionally display.
    try:
        task.planner.visualize(q_init)
        print("✓ Initial scene displayed")
    except Exception as e:
        print(f"⚠ Visualization: {e}")

    # ------------------------------------------------------------------
    # 4. Build GraspSequencePlanner and run
    # ------------------------------------------------------------------
    print("\nCreating GraspSequencePlanner...")
    seq_planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        pyhpp_constraints=getattr(task, "pyhpp_constraints", {}),
        graph_constraints=getattr(task, "_graph_constraints", None),
        auto_save_dir=None,
        run_logger=getattr(task, "run_logger", None),
    )

    grasp_sequence = [("spacelab/g_ur10_tool", "frame_gripper/h_FG_tool")]
    print(f"\nPlanning grasp sequence: {grasp_sequence}")

    try:
        result = seq_planner.plan_sequence(
            grasp_sequence=grasp_sequence,
            q_init=q_init,
            verbose=True,
        )
    except Exception as e:
        print(f"\n✗ Planning error: {e}")
        import traceback
        traceback.print_exc()
        return False

    if not result["success"]:
        print("\n" + "=" * 70)
        print("✗ PLANNING FAILED")
        print("=" * 70)
        print(f"Reason: {result.get('error', 'Unknown')}")
        return False

    # ------------------------------------------------------------------
    # 5. Success — interactive replay
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("✓ PLANNING SUCCEEDED")
    print("=" * 70)
    print(seq_planner.get_phase_summary())

    all_paths = [
        p
        for phase in seq_planner.phase_results
        for p in phase.get("paths", [])
        if p is not None
    ]
    num_paths = len(all_paths)
    if num_paths > 0:
        print(f"\n✓ {num_paths} path(s) generated")
        _interactive_replay(task, seq_planner)
    return True


# ---------------------------------------------------------------------------
# Interactive replay (identical logic to test_pyhpp_planning.py)
# ---------------------------------------------------------------------------

def _interactive_replay(
    task: SpaceLabYamlTask,
    seq_planner: "GraspSequencePlanner",
) -> None:
    """Persistent replay menu for paths produced by the sequence planner."""
    path_items = []
    for phase in seq_planner.phase_results:
        for idx, path_obj in enumerate(phase.get("paths", [])):
            if path_obj is not None:
                edge_names = phase.get("edges", [])
                label = (
                    edge_names[idx]
                    if idx < len(edge_names)
                    else f"phase {phase['phase']} path {idx}"
                )
                path_items.append((label, path_obj))

    print("\n" + "-" * 50)
    print("Replay menu")
    print(f"  {len(path_items)} path(s) available")
    for i, (label, _) in enumerate(path_items):
        print(f"    [{i}]  {label}")
    print("  [a]  replay all in sequence")
    print("  [q]  quit")
    print("-" * 50)

    while True:
        try:
            raw = input("replay> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if raw in ("q", "quit", "exit", ""):
            break

        if raw in ("a", "all"):
            print("\nReplaying full sequence...")
            try:
                seq_planner.replay_sequence(speed=1.0)
                print("✓ Done")
            except Exception as e:
                print(f"  ⚠ replay_sequence failed: {e}")
                _replay_fallback(task, path_items)
            continue

        try:
            idx = int(raw)
        except ValueError:
            print(
                f"  Unknown command '{raw}'. "
                "Enter a path index, 'a' for all, or 'q' to quit."
            )
            continue

        if idx < 0 or idx >= len(path_items):
            print(f"  Index {idx} out of range (0 – {len(path_items) - 1})")
            continue

        label, path_obj = path_items[idx]
        print(f"\nReplaying [{idx}] {label} ...")
        try:
            if isinstance(path_obj, int):
                task.planner.play_path(path_obj)
            else:
                task.planner.play_path_vector(path_obj)
            print("✓ Done")
        except Exception as e:
            print(f"  ⚠ Failed: {e}")


def _replay_fallback(
    task: SpaceLabYamlTask,
    path_items: List,
) -> None:
    """Replay all items directly when replay_sequence() throws."""
    for label, path_obj in path_items:
        print(f"  Playing: {label}")
        try:
            if isinstance(path_obj, int):
                task.planner.play_path(path_obj)
            else:
                task.planner.play_path_vector(path_obj)
        except Exception as ex:
            print(f"    ⚠ Failed: {ex}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="SpaceLab YAML grasp-sequence test"
    )
    parser.add_argument(
        "--backend",
        default="pyhpp",
        choices=["pyhpp", "corba"],
        help="Planning backend (default: pyhpp)",
    )
    args = parser.parse_args()

    success = run_task(backend=args.backend)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
