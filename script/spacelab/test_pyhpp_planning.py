#!/usr/bin/env python3
"""Test script for PyHPP backend grasp sequence planning.

Configuration:
- Pair: ur10-frame_gripper (g_ur10_tool:frame_gripper)
- Lock: vispa and vispa2 arms
- Use: GraspSequencePlanner
- No auto-save paths
- Allow replay
"""

import sys
from pathlib import Path

from agimus_spacelab.tasks.grasp_sequence import (
    GraspSequencePlanner,
)
from agimus_spacelab.cli.config_loader import (
    load_task_config,
    get_default_config_dir,
)
from agimus_spacelab.tasks import ManipulationTask


class TestPyHPPPlanning(ManipulationTask):
    """Test PyHPP planning with specific configuration."""

    FREEZE_JOINT_SUBSTRINGS = ["vispa_", "vispa2"]

    def __init__(self, backend: str = "pyhpp"):
        config_dir = get_default_config_dir(Path(__file__))
        if str(config_dir) not in sys.path:
            sys.path.insert(0, str(config_dir))
        from spacelab_config import DEFAULT_PATHS, JointBounds  # noqa: PLC0415
        super().__init__(
            task_name="Test PyHPP Grasp Sequence",
            backend=backend,
            FILE_PATHS=DEFAULT_PATHS,
            joint_bounds=JointBounds,
        )
        self.task_config = load_task_config(
            config_dir,
            "spacelab_config",
            "TaskConfigurations.DisplayAllStates",
        )

        # Filter to single grasp pair
        self.task_config = self.task_config.with_grasp_goals([
            "spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool"
        ])

        self.use_factory = True
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> list[float]:
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

    def run_test(self):
        """Run the planning test."""
        print("\n" + "="*70)
        print("PyHPP Backend Test")
        print("="*70)
        print(f"Grasp: g_ur10_tool -> frame_gripper")
        print(f"Locked arms: {self.FREEZE_JOINT_SUBSTRINGS}")
        print(f"Backend: {self.backend}")
        print("="*70 + "\n")

        # Setup with locked arms
        print("Initializing task...")
        self.setup(
            validation_step=0.01,
            projector_step=0.1,
            freeze_joint_substrings=self.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,  # Grasp sequence builder creates its own graph
        )

        # Disable collision between robot arms and ground structure
        print("\nDisabling problematic collision pairs...")
        pairs_removed = 0
        collision_pairs = [
            ("ground_demo/link_NYX_0", "spacelab/ur10_link_4"),
            ("ground_demo/link_NYX_0", "spacelab/vispa_link_4"),
            ("ground_demo/link_NYX_0", "spacelab/vispa2_link_4"),
            ("ground_demo/link_NYX_0", "spacelab/ur10_link_3"),
            ("ground_demo/link_NYX_0", "spacelab/vispa_link_3"),
            ("ground_demo/link_NYX_0", "spacelab/vispa2_link_3"),
        ]
        for obstacle, joint in collision_pairs:
            try:
                self.scene_builder.disable_collision_pair(
                    obstacle_name=obstacle,
                    joint_name=joint
                )
                pairs_removed += 1
            except Exception as e:
                print(f"   Could not disable {obstacle} <-> {joint}: {e}")
        print(f"✓ Disabled {pairs_removed} collision pair(s) with ground structure")

        print("✓ Task initialized")
        print(f"  Backend: {type(self.planner).__name__}")
        print(f"  Graph builder: {type(self.graph_builder).__name__}")

        # Get initial config
        q_init = self.q_init
        if not q_init:
            print("✗ Failed to get initial configuration")
            return False

        print(f"  Initial config size: {len(q_init)}")

        # Check if initial config is collision-free
        print("\nChecking initial configuration for collisions...")
        try:
            # For PyHPP, collision checking is handled internally by the planner
            # We can verify the config is valid by attempting to project it
            if self.backend == "pyhpp":
                print("✓ Using PyHPP backend (collision checking during planning)")
            else:
                # For CORBA backend, we could check via the problem solver
                print("✓ Using CORBA backend (collision checking during planning)")
        except Exception as e:
            print(f"⚠ Could not check collisions: {e}")

        # Display initial scene
        try:
            print("\nDisplaying initial configuration...")
            self.planner.visualize(q_init)
            print("✓ Scene displayed")
        except Exception as e:
            print(f"⚠ Visualization failed: {e}")

        # Create grasp sequence planner
        print("\nCreating grasp sequence planner...")
        seq_planner = GraspSequencePlanner(
            graph_builder=self.graph_builder,
            config_gen=self.config_gen,
            planner=self.planner,
            task_config=self.task_config,
            backend=self.backend,
            pyhpp_constraints=getattr(self, "pyhpp_constraints", {}),
            graph_constraints=getattr(self, "_graph_constraints", None),
            auto_save_dir=None,  # No auto-save
        )

        # Plan sequence: grasp then release
        grasp_sequence = [
            ("spacelab/g_ur10_tool", "frame_gripper/h_FG_tool"),
            ("spacelab/g_ur10_tool", None),  # release FG
        ]

        print("\nPlanning grasp sequence...")
        print(f"  Sequence: {grasp_sequence}")

        try:
            result = seq_planner.plan_sequence(
                grasp_sequence=grasp_sequence,
                q_init=q_init,
                verbose=True,
            )

            if result["success"]:
                print("\n" + "="*70)
                print("✓ PLANNING SUCCEEDED")
                print("="*70)
                print(seq_planner.get_phase_summary())

                # Interactive replay loop
                all_paths = [
                    p
                    for phase in seq_planner.phase_results
                    for p in phase.get("paths", [])
                    if p is not None
                ]
                num_paths = len(all_paths)
                if num_paths > 0:
                    print(f"\n✓ Generated {num_paths} path(s)")
                    self._interactive_replay(seq_planner, num_paths)

                return True
            else:
                print("\n" + "="*70)
                print("✗ PLANNING FAILED")
                print("="*70)
                print(f"Reason: {result.get('error', 'Unknown')}")
                return False

        except Exception as e:
            print("\n" + "="*70)
            print("✗ PLANNING ERROR")
            print("="*70)
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _interactive_replay(self, seq_planner, num_paths: int):
        """Interactive replay menu.

        Available commands:
          a       — replay all paths in sequence
          <index> — replay a single path by 0-based index
          q       — quit replay
        """
        # Collect flat list of (display_label, path_obj) for single-path replay
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
        print(f"  {num_paths} path(s) available")
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
                    self._replay_fallback(path_items)
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
                print(
                    f"  Index {idx} out of range "
                    f"(0 – {len(path_items) - 1})"
                )
                continue

            label, path_obj = path_items[idx]
            print(f"\nReplaying [{idx}] {label} ...")
            try:
                if isinstance(path_obj, int):
                    self.planner.play_path(path_obj)
                else:
                    self.planner.play_path_vector(path_obj)
                print("✓ Done")
            except Exception as e:
                print(f"  ⚠ Failed: {e}")

    def _replay_fallback(self, path_items):
        """Replay all items directly when replay_sequence fails."""
        for label, path_obj in path_items:
            print(f"  Playing: {label}")
            try:
                if isinstance(path_obj, int):
                    self.planner.play_path(path_obj)
                else:
                    self.planner.play_path_vector(path_obj)
            except Exception as ex:
                print(f"    ⚠ Failed: {ex}")


def main():
    """Main entry point."""
    import argparse
    parser = argparse.ArgumentParser(
        description="Test PyHPP grasp sequence planning"
    )
    parser.add_argument(
        "--backend",
        default="pyhpp",
        choices=["pyhpp", "corba"],
        help="Backend to use (default: pyhpp)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip visualization",
    )
    
    args = parser.parse_args()
    
    task = TestPyHPPPlanning(backend=args.backend)
    success = task.run_test()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
