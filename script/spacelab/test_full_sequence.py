#!/usr/bin/env python3
"""Test: full 13-phase SpaceLab assembly sequence (non-stop mode).

Replicates the interactive_planning.py session with:

  Sequence (13 grasps, auto-release inserted automatically by planner):
    1.  spacelab/g_ur10_tool     -> frame_gripper/h_FG_tool
    2.  frame_gripper/g_FG_part  -> RS1/h_RS1_FG
    3.  spacelab/g_vispa2_wb1    -> RS1/h_RS1_WB
    4.  frame_gripper/g_FG_part  -> RS6/h_RS6_FG
    5.  spacelab/g_vispa2_wb6    -> RS6/h_RS6_WB
    6.  frame_gripper/g_FG_part  -> RS5/h_RS5_FG
    7.  spacelab/g_vispa2_wb5    -> RS5/h_RS5_WB
    8.  frame_gripper/g_FG_part  -> RS2/h_RS2_FG
    9.  spacelab/g_vispa2_wb2    -> RS2/h_RS2_WB
    10. frame_gripper/g_FG_part  -> RS3/h_RS3_FG
    11. spacelab/g_vispa2_wb3    -> RS3/h_RS3_WB
    12. frame_gripper/g_FG_part  -> RS4/h_RS4_FG
    13. spacelab/g_vispa2_wb4    -> RS4/h_RS4_WB

  Frozen arms per phase (manual mode):
    Phase 0  (g_ur10_tool -> h_FG_tool):   ['vispa_', 'vispa2']
    Phase 1  (g_FG_part -> RS1/h_RS1_FG):  ['vispa_', 'vispa2']
    Phases 2-12:                           ['vispa_']

  Non-stop mode: auto-resume on failure until all phases complete or Ctrl+C.
  TransitionPlanner: timeout=60.0 s, max_iterations=10000
"""

import sys
from pathlib import Path
from typing import Dict, List

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.cli.config_loader import (
    load_task_config,
    get_default_config_dir,
)

# ---------------------------------------------------------------------------
# Sequence configuration
# ---------------------------------------------------------------------------

GRASP_SEQUENCE = [
    ("spacelab/g_ur10_tool", "frame_gripper/h_FG_tool"),
    ("frame_gripper/g_FG_part", "RS1/h_RS1_FG"),
    ("spacelab/g_vispa2_wb1", "RS1/h_RS1_WB"),
    ("frame_gripper/g_FG_part", "RS6/h_RS6_FG"),
    ("spacelab/g_vispa2_wb6", "RS6/h_RS6_WB"),
    ("frame_gripper/g_FG_part", "RS5/h_RS5_FG"),
    ("spacelab/g_vispa2_wb5", "RS5/h_RS5_WB"),
    ("frame_gripper/g_FG_part", "RS2/h_RS2_FG"),
    ("spacelab/g_vispa2_wb2", "RS2/h_RS2_WB"),
    ("frame_gripper/g_FG_part", "RS3/h_RS3_FG"),
    ("spacelab/g_vispa2_wb3", "RS3/h_RS3_WB"),
    ("frame_gripper/g_FG_part", "RS4/h_RS4_FG"),
    ("spacelab/g_vispa2_wb4", "RS4/h_RS4_WB"),
]

# Per-phase arm-freeze keywords.  Keys are 0-based phase indices matching
# GRASP_SEQUENCE above.
#
# For phases where ur10 carries the frame_gripper but is not actively moving
# (vispa2 is the active arm), also freeze ur10 to prevent it from drifting
# into self-colliding configurations between phases.
PER_PHASE_FROZEN_ARMS: Dict[int, List[str]] = {
    0: ["vispa_", "vispa2"],  # ur10 grabs frame tool — freeze both vispas
    1: ["vispa_", "vispa2"],  # FG grabs RS1 — freeze both vispas
    2: ["vispa_"],  # vispa2_wb1 grabs RS1
    3: ["vispa_"],  # FG_part grabs RS6 — ur10 active
    4: ["vispa_"],  # vispa2_wb6 grabs RS6
    5: ["vispa_"],  # FG_part grabs RS5 — ur10 active
    6: ["vispa_"],  # vispa2_wb5 grabs RS5
    7: ["vispa_"],  # FG_part grabs RS2 — ur10 active
    8: ["vispa_"],  # vispa2_wb2 grabs RS2
    9: ["vispa_"],  # FG_part grabs RS3 — ur10 active
    10: ["vispa_"],  # vispa2_wb3 grabs RS3
    11: ["vispa_"],  # FG_part grabs RS4 — ur10 active
    12: ["vispa_"],  # vispa2_wb4 grabs RS4
}

TIMEOUT_PER_EDGE = 60.0
MAX_ITERATIONS_PER_EDGE = 10000

# All 13 goal-state strings used to build the factory graph
GRASP_GOALS = [
    "spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool",
    "spacelab/g_vispa2_wb1 grasps RS1/h_RS1_WB",
    "spacelab/g_vispa2_wb2 grasps RS2/h_RS2_WB",
    "spacelab/g_vispa2_wb3 grasps RS3/h_RS3_WB",
    "spacelab/g_vispa2_wb4 grasps RS4/h_RS4_WB",
    "spacelab/g_vispa2_wb5 grasps RS5/h_RS5_WB",
    "spacelab/g_vispa2_wb6 grasps RS6/h_RS6_WB",
    "frame_gripper/g_FG_part grasps RS1/h_RS1_FG",
    "frame_gripper/g_FG_part grasps RS2/h_RS2_FG",
    "frame_gripper/g_FG_part grasps RS3/h_RS3_FG",
    "frame_gripper/g_FG_part grasps RS4/h_RS4_FG",
    "frame_gripper/g_FG_part grasps RS5/h_RS5_FG",
    "frame_gripper/g_FG_part grasps RS6/h_RS6_FG",
]

# ---------------------------------------------------------------------------
# Task class
# ---------------------------------------------------------------------------


class FullSequenceTask(ManipulationTask):
    """Full 13-phase SpaceLab assembly sequence test task."""

    # No global arm freeze — controlled per-phase via per_phase_frozen_arms
    FREEZE_JOINT_SUBSTRINGS: List[str] = []

    def __init__(self, backend: str = "pyhpp"):
        config_dir = get_default_config_dir(Path(__file__))
        if str(config_dir) not in sys.path:
            sys.path.insert(0, str(config_dir))
        from spacelab_config import DEFAULT_PATHS, JointBounds  # noqa: PLC0415

        super().__init__(
            task_name="SpaceLab Full Assembly Sequence (13 phases)",
            backend=backend,
            FILE_PATHS=DEFAULT_PATHS,
            joint_bounds=JointBounds,
        )
        cfg = load_task_config(
            config_dir,
            "spacelab_config",
            "TaskConfigurations.DisplayAllStates",
        )
        self.task_config = cfg.with_grasp_goals(GRASP_GOALS)
        self.use_factory = True
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> List[float]:
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
                pose = xyzrpy_to_xyzquat(
                    getattr(InitialConfigurations, obj_attr)
                )
                q_objects.extend(pose.tolist())
            else:
                q_objects.extend([0.0] * 7)
        return q_robot + q_objects

    def run(self, *, no_viz: bool = False, auto_save_dir: str = None) -> bool:
        """Run the full 13-phase sequence with non-stop auto-resume.

        Args:
            no_viz: Skip gepetto-viewer visualisation of the initial config.
            auto_save_dir: Directory to auto-save path files after each
                           successful phase.  None = no auto-save.

        Returns:
            True when all phases completed successfully.
        """
        print("\n" + "=" * 70)
        print("SpaceLab Full Assembly Sequence — 13 phases")
        print("=" * 70)
        _print_sequence()
        print("=" * 70 + "\n")

        # Task setup (scene + constraints, graph built per-phase by planner)
        print("Setting up task...")
        self.setup(
            validation_step=0.01,
            projector_step=0.1,
            freeze_joint_substrings=self.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,
        )
        print("✓ Task ready\n")

        q_init = self.q_init
        if not q_init:
            print("✗ Failed to obtain q_init")
            return False

        if not no_viz:
            try:
                self.planner.visualize(q_init)
                print("✓ Initial configuration displayed")
            except Exception as e:
                print(f"⚠ Visualization skipped: {e}")

        seq_planner = GraspSequencePlanner(
            graph_builder=self.graph_builder,
            config_gen=self.config_gen,
            planner=self.planner,
            task_config=self.task_config,
            backend=self.backend,
            pyhpp_constraints=getattr(self, "pyhpp_constraints", {}),
            graph_constraints=getattr(self, "_graph_constraints", None),
            auto_save_dir=auto_save_dir,
            run_logger=self.run_logger,
        )

        print(
            "Non-stop mode: will auto-resume after each failure "
            "(Ctrl+C to stop)\n"
        )
        print(
            f"TransitionPlanner: timeout={TIMEOUT_PER_EDGE}s, "
            f"max_iterations={MAX_ITERATIONS_PER_EDGE}\n"
        )

        # First planning attempt
        try:
            result = seq_planner.plan_sequence(
                grasp_sequence=GRASP_SEQUENCE,
                q_init=q_init,
                frozen_arms_mode="manual",
                per_phase_frozen_arms=PER_PHASE_FROZEN_ARMS,
                timeout_per_edge=TIMEOUT_PER_EDGE,
                max_iterations_per_edge=MAX_ITERATIONS_PER_EDGE,
                verbose=True,
            )
        except KeyboardInterrupt:
            print("\nStopped by user.")
            _print_summary(seq_planner)
            return False
        except Exception as e:
            print(f"\nplan_sequence raised: {e}")
            result = {"success": False}

        if result.get("success"):
            return _on_success(seq_planner, self.planner)

        # Non-stop auto-resume loop
        print("\nNon-stop mode: auto-resuming ...  (Ctrl+C to stop)\n")
        while True:
            resume_state = seq_planner.get_resumable_state()
            if resume_state is None:
                # All phases done (no remaining failures)
                return _on_success(seq_planner, self.planner)

            print(
                f"Failed at Phase {resume_state['phase_idx'] + 1}, "
                f"Edge {resume_state['edge_idx'] + 1}: "
                f"{resume_state['edge_name']}"
            )
            print(f"  Completed phases: {resume_state['completed_phases']}")
            print(f"  Error: {resume_state['error']}")

            try:
                result = seq_planner.resume_sequence(
                    retry_from_edge=-1,  # retry only the failed edge
                    frozen_arms_mode="manual",
                    per_phase_frozen_arms=PER_PHASE_FROZEN_ARMS,
                    timeout_per_edge=TIMEOUT_PER_EDGE,
                    max_iterations_per_edge=MAX_ITERATIONS_PER_EDGE,
                    verbose=True,
                )
            except KeyboardInterrupt:
                print("\nStopped by user.")
                _print_summary(seq_planner)
                return False
            except Exception as e:
                print(f"\nresume_sequence raised: {e}")
                result = {"success": False}

            if result.get("success"):
                return _on_success(seq_planner, self.planner)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _print_sequence() -> None:
    print("Sequence:")
    for i, (gripper, handle) in enumerate(GRASP_SEQUENCE):
        freeze = PER_PHASE_FROZEN_ARMS.get(i, [])
        print(f"  {i + 1:2}. {gripper} -> {handle}   [freeze {freeze}]")


def _on_success(seq_planner: GraspSequencePlanner, planner) -> bool:
    print("\n" + "=" * 70)
    print("✓ ALL 13 PHASES COMPLETE")
    print("=" * 70)
    print(seq_planner.get_phase_summary())
    all_paths = [
        p
        for phase in seq_planner.phase_results
        for p in phase.get("paths", [])
        if p is not None
    ]
    print(f"\n✓ Generated {len(all_paths)} path(s)")
    _interactive_replay(seq_planner, planner)
    return True


def _print_summary(seq_planner: GraspSequencePlanner) -> None:
    try:
        print("\n" + seq_planner.get_phase_summary())
    except Exception:
        pass


def _interactive_replay(seq_planner: GraspSequencePlanner, planner) -> None:
    """Simple replay menu matching test_pyhpp_planning.py."""
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

    if not path_items:
        return

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
            print("Replaying full sequence...")
            try:
                seq_planner.replay_sequence(speed=1.0)
                print("✓ Done")
            except Exception as e:
                print(f"  ⚠ replay_sequence failed: {e}")
                for _, path_obj in path_items:
                    try:
                        if isinstance(path_obj, int):
                            planner.play_path(path_obj)
                        else:
                            planner.play_path_vector(path_obj)
                    except Exception:
                        pass
            continue

        try:
            idx = int(raw)
        except ValueError:
            print(f"  Unknown command '{raw}'. Enter index, 'a', or 'q'.")
            continue

        if idx < 0 or idx >= len(path_items):
            print(f"  Index {idx} out of range (0 - {len(path_items) - 1})")
            continue

        label, path_obj = path_items[idx]
        print(f"\nReplaying [{idx}] {label} ...")
        try:
            if isinstance(path_obj, int):
                planner.play_path(path_obj)
            else:
                planner.play_path_vector(path_obj)
            print("✓ Done")
        except Exception as e:
            print(f"  ⚠ Failed: {e}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Full 13-phase SpaceLab assembly sequence test"
    )
    parser.add_argument(
        "--backend",
        default="pyhpp",
        choices=["pyhpp", "corba"],
        help="Planning backend (default: pyhpp)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip gepetto-viewer visualisation of initial config",
    )
    parser.add_argument(
        "--auto-save-dir",
        default=None,
        metavar="DIR",
        help="Directory to auto-save paths after each phase (default: no save)",
    )
    args = parser.parse_args()

    task = FullSequenceTask(backend=args.backend)
    success = task.run(no_viz=args.no_viz, auto_save_dir=args.auto_save_dir)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
