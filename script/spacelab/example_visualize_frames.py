#!/usr/bin/env python3
"""
Example: Using visualize_frames utility with SpaceLab scene.

Demonstrates how to visualize handle and gripper frames with approaching
directions for any robot/object configuration.
"""

import sys
from pathlib import Path

# Add paths
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "config"))

from visualize_frames import (
    displayHandle,
    displayGripper,
    displayHandleApproach,
    displayGripperApproach,
    visualize_all_handles,
    visualize_all_grippers,
    print_handle_info,
    print_gripper_info,
    clear_all_visualizations,
)

from spacelab_tools import SpaceLabSceneBuilder
from spacelab_config import InitialConfigurations, ManipulationConfig
from agimus_spacelab.utils import xyzrpy_to_xyzquat

try:
    from hpp.gepetto.manipulation import ViewerFactory
    HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False
    print("Warning: gepetto-viewer not available")


def main():
    """Run visualization example."""
    
    if not HAS_VIEWER:
        print("Error: This example requires gepetto-viewer")
        return
    
    print("=" * 70)
    print("SPACELAB FRAME VISUALIZATION EXAMPLE")
    print("=" * 70)
    
    # Setup scene
    print("\n1. Setting up scene...")
    builder = SpaceLabSceneBuilder()
    builder.load_robot()
    builder.load_environment()
    builder.load_objects(["frame_gripper", "RS1"])
    
    planner = builder.planner
    robot = planner.get_robot()
    ps = planner.get_problem_solver()
    
    # Build initial configuration
    print("\n2. Building initial configuration...")
    q_robot = (InitialConfigurations.UR10 + 
               InitialConfigurations.VISPA_BASE + 
               InitialConfigurations.VISPA_ARM)
    
    # Add object poses
    q_objects = []
    for pose_xyzrpy in [InitialConfigurations.FRAME_GRIPPER,
                         InitialConfigurations.RS1]:
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_objects.extend(pose_xyzquat.tolist())
    
    q_init = q_robot + q_objects
    
    # Create viewer
    print("\n3. Creating viewer...")
    vf = ViewerFactory(ps)
    viewer = vf.createViewer()
    viewer(q_init)  # Display initial config
    
    # Get actually defined handles from robot
    print("\n4. Querying available handles and grippers...")
    
    # For handles, we need to check which ones are actually defined
    # Try to get handles from the loaded objects
    potential_handles = ["frame_gripper/h_FG_tool"]
    
    # Test which handles actually exist
    print("\nHandles:")
    all_handles = []
    for handle in potential_handles:
        try:
            robot.getHandlePositionInJoint(handle)
            all_handles.append(handle)
            print(f"  ✓ {handle}")
        except Exception:
            print(f"  ✗ {handle} (not defined in SRDF)")
    
    # Get grippers
    print("\nGrippers:")
    potential_grippers = [
        "spacelab/g_ur10_tool",
    ]
    grippers = []
    for gripper in potential_grippers:
        try:
            robot.getGripperPositionInJoint(gripper)
            grippers.append(gripper)
            print(f"  ✓ {gripper}")
        except Exception:
            print(f"  ✗ {gripper} (not defined in SRDF)")
    
    # Print detailed information for available handles
    if all_handles:
        print("\n5. Detailed handle information:")
        for handle in all_handles[:2]:  # Show first 2
            print_handle_info(viewer, handle)
    else:
        print("\n5. No handles available to display")
    
    # Demonstrate basic display functions (frames only, no arrows)
    print("\n6. Displaying basic frames (attached to robot)...")
    if all_handles:
        print(f"   - Handle: {all_handles[0]}")
        displayHandle(viewer, all_handles[0])
    if grippers:
        print(f"   - Gripper: {grippers[0]}")
        displayGripper(viewer, grippers[0])
    
    # Visualize all handles
    if all_handles:
        print("\n7. Visualizing handles with approach arrows...")
        visualize_all_handles(
            viewer, all_handles,
            show_approach=True,
            frame_color=[0, 0.8, 0, 1],  # Green
            arrow_color=[0, 1, 1, 1],     # Cyan
            axis_length=0.05,
            arrow_length=0.1
        )
    else:
        print("\n7. No handles to visualize")
    
    # Visualize grippers
    if grippers:
        print("\n8. Visualizing grippers with approach arrows...")
        visualize_all_grippers(
            viewer, grippers,
            show_approach=True,
            frame_color=[1, 0, 0, 1],     # Red
            arrow_color=[1, 0.5, 0, 1],   # Orange
            axis_length=0.05,
            arrow_length=0.1
        )
    else:
        print("\n8. No grippers to visualize")
    
    print("\n" + "=" * 70)
    print("VISUALIZATION COMPLETE")
    print("=" * 70)
    
    if all_handles or grippers:
        print("\nLegend:")
        if all_handles:
            print("  - Green frames + cyan arrows: Handles")
        if grippers:
            print("  - Red frames + orange arrows: Grippers")
        print("\nUseful commands:")
        print("  viewer(q_init) - Redisplay configuration")
        print("  displayHandle(viewer, 'object/handle') - Add handle frame")
        print("  displayHandleApproach(viewer, 'object/handle') - Add approach arrow")
        print("  displayGripper(viewer, 'robot/gripper') - Add gripper frame")
        print("  displayGripperApproach(viewer, 'robot/gripper') - Add approach arrow")
        print("  clear_all_visualizations(viewer) - Remove all frames")
    else:
        print("\nNote: No handles or grippers were visualized.")
        print("This may be because they are not defined in the SRDF files.")
    
    print("=" * 70)
    
    # Keep session alive
    import code
    code.interact(local=locals())


if __name__ == "__main__":
    main()
