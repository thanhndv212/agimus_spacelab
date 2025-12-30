#!/usr/bin/env python3
"""
SpaceLab Scene Visualization Script

Loads and visualizes the complete SpaceLab scene with:
- UR10 and VISPA robots
- RS modules (RS1-RS6)
- Tools (frame_gripper, screw_driver, cleat_gripper)
- Environment (ground_demo)

Usage:
    python visualize_scene.py                    # Load all objects
    python visualize_scene.py --objects RS1 RS2  # Load specific objects
    python visualize_scene.py --minimal          # Robots + environment only
"""

import argparse
from typing import List, Optional

from agimus_spacelab.config.spacelab_config import (
    InitialConfigurations,
    ManipulationConfig,
)
from agimus_spacelab.planning import SceneBuilder
from agimus_spacelab.utils import xyzrpy_to_xyzquat

try:
    from hpp.gepetto.manipulation import ViewerFactory

    HAS_GEPETTO = True
except ImportError:
    HAS_GEPETTO = False
    print("⚠ Warning: gepetto-viewer not available. Visualization disabled.")


class SceneVisualizer:
    """Handles scene visualization with gepetto-viewer."""

    def __init__(self):
        """Initialize the visualizer."""
        self.planner = None
        self.viewer_factory = None
        self.viewer = None
        self.loaded_objects: List[str] = []

    def setup_scene(self, objects: Optional[List[str]] = None) -> List[str]:
        """
        Set up the scene with robots, environment, and objects.
        
        Args:
            objects: List of object names to load. If None, loads all.
        """
        print("\n" + "=" * 70)
        print("SPACELAB SCENE SETUP")
        print("=" * 70)

        # Default objects if none specified.
        # Prefer canonical OBJECTS (has poses/handles), then include any extra
        # URDF-only objects from MODEL_PATHS.
        if objects is None:
            objects = list(ManipulationConfig.OBJECTS.keys())
            extra = [
                name
                for name in ManipulationConfig.MODEL_PATHS.get("objects", {})
                if name not in objects
            ]
            objects.extend(extra)

        # Build scene using fluent API, with canonical file paths.
        builder = SceneBuilder(FILE_PATHS=ManipulationConfig.MODEL_PATHS)

        print("\n1. Loading robots and environment...")
        robot_names = list(getattr(ManipulationConfig, "ROBOT_NAMES", []))
        composite_names = list(robot_names)
        environment_names = list(
            getattr(ManipulationConfig, "ENVIRONMENT_NAMES", [])
        )

        builder.load_robot(
            composite_names=composite_names,
            robot_names=robot_names,
        )
        builder.load_environment(environment_names=environment_names)

        print(f"\n2. Loading objects: {', '.join(objects)}")
        builder.load_objects(objects)

        print("\n3. Setting joint bounds...")
        builder.set_joint_bounds()

        self.planner = builder.planner
        self.loaded_objects = list(objects)
        print("\n✓ Scene setup complete!")

        return self.loaded_objects

    def create_initial_configuration(self, objects: List[str]) -> List[float]:
        """
        Create initial configuration with all robots and objects.
        
        Args:
            objects: List of loaded objects
            
        Returns:
            Complete configuration vector
        """
        print("\n" + "=" * 70)
        print("INITIAL CONFIGURATION")
        print("=" * 70)

        q = []

        # Robot joint groups (order matters, matches composite robot model).
        robot_groups = ["UR10", "VISPA_BASE", "VISPA_ARM"]
        for group in robot_groups:
            values = list(getattr(InitialConfigurations, group, []))
            if not values:
                continue
            print(f"\n{group} ({len(values)} DOF):")
            q.extend(values)
            print(f"  {values}")

        # Object poses (7 DOF each: xyz + quaternion)
        print("\nObject poses (XYZQUAT):")

        for obj_name in objects:
            xyzrpy = None

            if obj_name in ManipulationConfig.OBJECTS:
                xyzrpy = ManipulationConfig.OBJECTS[obj_name].get(
                    "default_pose_xyzrpy"
                )

            if xyzrpy is None:
                # Fallback to InitialConfigurations constants if present.
                attr = obj_name.upper().replace("-", "_")
                xyzrpy = getattr(InitialConfigurations, attr, None)

            if xyzrpy is None:
                continue

            xyzquat = xyzrpy_to_xyzquat(xyzrpy)
            q.extend(xyzquat)
            print(
                f"  {obj_name}: xyz=("
                f"{xyzquat[0]:.3f}, {xyzquat[1]:.3f}, {xyzquat[2]:.3f})"
            )

        print(f"\n✓ Configuration size: {len(q)} DOF")
        return q

    def initialize_viewer(self) -> None:
        """Initialize gepetto viewer."""
        if not HAS_GEPETTO:
            print("\n⚠ Gepetto viewer not available")
            return

        print("\n" + "=" * 70)
        print("VIEWER INITIALIZATION")
        print("=" * 70)

        try:
            print("\nCreating viewer factory...")
            # Prefer the backend's ViewerFactory (it has already loaded
            # environment + objects via planner.load_environment/load_object).
            self.viewer_factory = getattr(self.planner, "vf", None)
            if self.viewer_factory is None:
                self.viewer_factory = ViewerFactory(self.planner.ps)

            print("Launching gepetto-viewer...")
            self.viewer = self.viewer_factory.createViewer()

            print("\n✓ Viewer ready!")
            print("\n  A gepetto-viewer window should now be open.")
            print("  Use mouse to navigate:")
            print("    - Left click + drag: Rotate")
            print("    - Right click + drag: Pan")
            print("    - Scroll: Zoom")

        except Exception as e:
            print(f"\n✗ Failed to initialize viewer: {e}")
            print("  Make sure gepetto-gui is installed and running.")

    def display_configuration(self, q: List[float]) -> None:
        """
        Display a configuration in the viewer.
        
        Args:
            q: Configuration vector
        """
        if not self.viewer:
            print("⚠ No viewer available")
            return

        try:
            # Use the viewer callable to display configuration
            self.viewer(q)
            print("\n✓ Displaying configuration in viewer")
        except Exception as e:
            print(f"✗ Failed to display configuration: {e}")

    def run_interactive(self, q: List[float]) -> None:
        """
        Run interactive mode with configuration display.
        
        Args:
            q: Initial configuration
        """
        if not self.viewer:
            print("\n⚠ Interactive mode requires gepetto viewer")
            return

        print("\n" + "=" * 70)
        print("INTERACTIVE MODE")
        print("=" * 70)

        self.display_configuration(q)

        print("\nAvailable commands:")
        print("  q / quit / exit - Exit")
        print("  h / help        - Show this help")
        print("  refresh / r     - Refresh display")
        print("  info / i        - Show scene info")

        while True:
            try:
                cmd = input("\nviewer> ").strip().lower()

                if cmd in ['q', 'quit', 'exit']:
                    print("Exiting...")
                    break

                elif cmd in ['h', 'help']:
                    print("\nAvailable commands:")
                    print("  q / quit / exit - Exit")
                    print("  h / help        - Show this help")
                    print("  refresh / r     - Refresh display")
                    print("  info / i        - Show scene info")

                elif cmd in ['r', 'refresh']:
                    self.display_configuration(q)
                    print("✓ Display refreshed")

                elif cmd in ['i', 'info']:
                    print("\nScene information:")
                    print(f"  Configuration DOF: {len(q)}")
                    print("  Robots: UR10, VISPA")
                    if self.loaded_objects:
                        print(f"  Objects: {', '.join(self.loaded_objects)}")

                else:
                    if cmd:
                        print(f"Unknown command: '{cmd}'. Type 'h' for help.")

            except (EOFError, KeyboardInterrupt):
                print("\nExiting...")
                break


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Visualize SpaceLab scene with robots and objects",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Load all objects
  %(prog)s --objects RS1 RS2         # Load only RS1 and RS2
  %(prog)s --minimal                 # Robots + environment only
  %(prog)s --no-interactive          # Setup and exit (no interactive mode)
        """
    )
    
    parser.add_argument(
        '--objects', '-o',
        nargs='+',
        help='Objects to load (e.g., RS1 RS2 frame_gripper)'
    )
    
    parser.add_argument(
        '--minimal', '-m',
        action='store_true',
        help='Load minimal scene (robots + environment only, no objects)'
    )
    
    parser.add_argument(
        '--no-interactive', '-n',
        action='store_true',
        help='Do not enter interactive mode after setup'
    )
    
    return parser.parse_args()


def main():
    """Main visualization script."""
    args = parse_args()

    # Determine which objects to load
    if args.minimal:
        objects = []
    elif args.objects:
        objects = args.objects
    else:
        # Default: let `setup_scene()` pick canonical objects.
        objects = None

    # Create visualizer
    visualizer = SceneVisualizer()

    # Setup scene
    loaded_objects = visualizer.setup_scene(objects)

    # Create initial configuration
    q_init = visualizer.create_initial_configuration(loaded_objects)

    # Initialize viewer
    visualizer.initialize_viewer()

    # Display configuration
    if visualizer.viewer:
        visualizer.display_configuration(q_init)

    # Interactive mode
    if not args.no_interactive and HAS_GEPETTO:
        try:
            visualizer.run_interactive(q_init)
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
    else:
        print("\n" + "=" * 70)
        print("Scene loaded successfully!")
        print("=" * 70)
        if not HAS_GEPETTO:
            print("\nNote: Install gepetto-viewer for visualization:")
            print(
                "  Viewer would show: robots, environment, "
                "and all loaded objects"
            )

    print("\nDone.")


if __name__ == "__main__":
    main()
