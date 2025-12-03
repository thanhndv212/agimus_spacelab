#!/usr/bin/env python3
"""
Utility for visualizing handle and gripper frames with approaching directions.

This module provides functions to display frames and arrows in gepetto-viewer
for any robot handles and grippers, showing their poses and approach vectors.

Based on hpp.gepetto.viewer displayHandle and displayGripper functions.
"""

import numpy as np
from typing import List, Optional
from pinocchio import SE3, Quaternion
from agimus_spacelab.utils import xyzquat_to_se3

def pose_to_SE3(pose: List[float]) -> SE3:
    """
    Convert pose list to SE3 transform.
    
    Args:
        pose: [x, y, z, qw, qx, qy, qz]
        
    Returns:
        SE3 transformation matrix
    """
    trans = np.array(pose[:3])
    quat = Quaternion(pose[6], pose[3], pose[4], pose[5])  # w, x, y, z
    return SE3(quat.matrix(), trans)


def compute_arrow_orientation(direction: np.ndarray) -> Quaternion:
    """
    Compute quaternion to orient arrow along given direction.
    
    Arrow default orientation is along X-axis.
    
    Args:
        direction: 3D direction vector (will be normalized)
        
    Returns:
        Quaternion for arrow orientation
    """
    x_axis = direction / np.linalg.norm(direction)
    z_axis = np.array([0, 0, 1])
    y_axis = np.cross(z_axis, x_axis)
    
    # Handle parallel case
    if np.linalg.norm(y_axis) < 1e-6:
        y_axis = np.array([0, 1, 0])
    
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    
    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
    return Quaternion(rot_matrix)


def displayHandle(
    viewer,
    handle_name: str,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015
) -> bool:
    """
    Display handle frame in gepetto-gui (wrapper around hpp.gepetto.viewer method).
    
    Retrieves the joint and pose information of the handle in the robot model
    and displays a frame. The frame will be attached to the robot link, so it
    moves with the robot configuration.
    
    Args:
        viewer: Gepetto viewer instance
        handle_name: Full handle name (e.g., "box/handle2")
        frame_color: RGBA color for frame [r, g, b, a] (default: [0, 1, 0, 1] green)
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        
    Returns:
        True if successful
    """
    if frame_color is None:
        frame_color = [0, 1, 0, 1]  # Green
    
    try:
        robot = viewer.robot
        joint, pose = robot.getHandlePositionInJoint(handle_name)
        hname = "handle__" + handle_name.replace("/", "_")
        viewer.client.gui.addXYZaxis(hname, frame_color, axis_radius, axis_length)
        
        if joint != "universe":
            link = robot.getLinkNames(joint)[0]
            viewer.client.gui.addToGroup(hname, robot.name + "/" + link)
        else:
            viewer.client.gui.addToGroup(hname, robot.name)
        
        viewer.client.gui.applyConfiguration(hname, pose)
        return True
    except Exception as e:
        print(f"  Warning: Could not display handle {handle_name}: {e}")
        return False



def displayGripper(
    viewer,
    gripper_name: str,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015
) -> bool:
    """
    Display gripper frame in gepetto-gui (wrapper around hpp.gepetto.viewer method).
    
    Retrieves the joint and pose information of the gripper in the robot model
    and displays a frame. The frame will be attached to the robot link, so it
    moves with the robot configuration.
    
    Args:
        viewer: Gepetto viewer instance
        gripper_name: Full gripper name (e.g., "pr2/l_gripper")
        frame_color: RGBA color for frame [r, g, b, a] (default: [0, 1, 0, 1] green)
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        
    Returns:
        True if successful
    """
    if frame_color is None:
        frame_color = [0, 1, 0, 1]  # Green
    
    try:
        robot = viewer.robot
        joint, pose = robot.getGripperPositionInJoint(gripper_name)
        gname = "gripper__" + gripper_name.replace("/", "_")
        viewer.client.gui.addXYZaxis(gname, frame_color, axis_radius, axis_length)
        
        if joint != "universe":
            link = robot.getLinkNames(joint)[0]
            viewer.client.gui.addToGroup(gname, robot.name + "/" + link)
        else:
            viewer.client.gui.addToGroup(gname, robot.name)
        
        viewer.client.gui.applyConfiguration(gname, pose)
        return True
    except Exception as e:
        print(f"  Warning: Could not display gripper {gripper_name}: {e}")
        return False


def displayHandleApproach(
    viewer,
    handle_name: str,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None
) -> bool:
    """
    Display approach direction arrow for a handle.
    
    The arrow will be attached to the same link as the handle frame,
    so it moves with the robot configuration.
    
    Args:
        viewer: Gepetto viewer instance
        handle_name: Full handle name (e.g., "box/handle2")
        arrow_color: RGBA color for arrow [r, g, b, a] (default: [0, 1, 1, 1] cyan)
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow
        
    Returns:
        True if successful
    """
    if arrow_color is None:
        arrow_color = [0, 1, 1, 1]  # Cyan
    
    try:
        robot = viewer.robot
        joint, pose = robot.getHandlePositionInJoint(handle_name)
        
        # Create arrow in handle frame
        arrow_name = "handle__" + handle_name.replace("/", "_") + "_approach"
        viewer.client.gui.addArrow(arrow_name, arrow_radius, arrow_length, arrow_color)
        # Transform approach direction relative to handle frame
        handle_T = xyzquat_to_se3(pose)
        approach_dir = np.array(approach_direction) if approach_direction is not None else robot.getHandleApproachingDirection(handle_name)
        approach_world = handle_T.rotation @ approach_dir
        arrow_quat = compute_arrow_orientation(approach_world)
        
        # Arrow starts at handle position with computed orientation
        arrow_pose = [pose[0], pose[1], pose[2],
                      arrow_quat.w, arrow_quat.x, arrow_quat.y, arrow_quat.z]
        viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)

        # Attach arrow to same parent as frame
        if joint != "universe":
            link = robot.getLinkNames(joint)[0]
            viewer.client.gui.addToGroup(arrow_name, robot.name + "/" + link)
        else:
            viewer.client.gui.addToGroup(arrow_name, robot.name)
        
        viewer.client.gui.applyConfiguration(arrow_name, pose)
        return True
    except Exception as e:
        print(f"  Warning: Could not display approach arrow for {handle_name}: {e}")
        return False


def displayGripperApproach(
    viewer,
    gripper_name: str,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None
) -> bool:
    """
    Display approach direction arrow for a gripper.
    
    The arrow will be attached to the same link as the gripper frame,
    so it moves with the robot configuration.
    
    Args:
        viewer: Gepetto viewer instance
        gripper_name: Full gripper name (e.g., "pr2/l_gripper")
        arrow_color: RGBA color for arrow [r, g, b, a] (default: [1, 0.5, 0, 1] orange)
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow
        approach_direction: Approach direction in gripper frame (default: [1, 0, 0])
        
    Returns:
        True if successful
    """
    if arrow_color is None:
        arrow_color = [1, 0.5, 0, 1]  # Orange
    
    try:
        robot = viewer.robot
        joint, pose = robot.getGripperPositionInJoint(gripper_name)
        
        # Create arrow in gripper frame
        arrow_name = "gripper__" + gripper_name.replace("/", "_") + "_approach"
        viewer.client.gui.addArrow(arrow_name, arrow_radius, arrow_length, arrow_color)
        
        # Transform approach direction relative to gripper frame
        gripper_T = xyzquat_to_se3(pose)
        approach_vec = np.array(approach_direction) if approach_direction is not None else robot.getGripperApproachingDirection(gripper_name)
        approach_world = gripper_T.rotation @ approach_vec
        arrow_quat = compute_arrow_orientation(approach_world)
        
        # Arrow starts at gripper position with computed orientation
        arrow_pose = [pose[0], pose[1], pose[2],
                      arrow_quat.w, arrow_quat.x, arrow_quat.y, arrow_quat.z]
        
        viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)
        
        # Attach arrow to same parent as frame
        if joint != "universe":
            link = robot.getLinkNames(joint)[0]
            viewer.client.gui.addToGroup(arrow_name, robot.name + "/" + link)
        else:
            viewer.client.gui.addToGroup(arrow_name, robot.name)
        
        return True
    except Exception as e:
        print(f"  Warning: Could not display approach arrow for {gripper_name}: {e}")
        return False


def visualize_all_handles(
    viewer,
    handle_names: List[str],
    show_approach: bool = True,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008
) -> int:
    """
    Display multiple handles at once.
    
    Args:
        viewer: Gepetto viewer instance
        handle_names: List of handle names
        show_approach: Whether to display approach arrows
        frame_color: RGBA color for frames
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        arrow_color: RGBA color for arrows
        arrow_length: Length of approach arrows
        arrow_radius: Radius of approach arrows
        
    Returns:
        Number of successfully visualized handles
    """
    print(f"\nDisplaying {len(handle_names)} handles...")
    success_count = 0
    
    for handle_name in handle_names:
        print(f"  {handle_name}")
        frame_ok = displayHandle(
            viewer, handle_name,
            frame_color=frame_color,
            axis_radius=axis_radius,
            axis_length=axis_length
        )
        arrow_ok = True
        if show_approach:
            arrow_ok = displayHandleApproach(
                viewer, handle_name,
                arrow_color=arrow_color,
                arrow_length=arrow_length,
                arrow_radius=arrow_radius,
            )
        
        if frame_ok and arrow_ok:
            success_count += 1
            status = "frame and arrow" if show_approach else "frame"
            print(f"    ✓ {status} added")
        elif frame_ok:
            print("    ✓ Frame added (arrow failed)")
        else:
            print("    ✗ Failed")
    
    viewer.client.gui.refresh()
    print(f"\nSuccessfully displayed {success_count}/{len(handle_names)} handles")
    return success_count


def visualize_all_grippers(
    viewer,
    gripper_names: List[str],
    show_approach: bool = True,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None
) -> int:
    """
    Display multiple grippers at once.
    
    Args:
        viewer: Gepetto viewer instance
        gripper_names: List of gripper names
        show_approach: Whether to display approach arrows
        frame_color: RGBA color for frames
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        arrow_color: RGBA color for arrows
        arrow_length: Length of approach arrows
        arrow_radius: Radius of approach arrows
        approach_direction: Approach direction in gripper frame
        
    Returns:
        Number of successfully visualized grippers
    """
    print(f"\nDisplaying {len(gripper_names)} grippers...")
    success_count = 0
    
    for gripper_name in gripper_names:
        print(f"  {gripper_name}")
        frame_ok = displayGripper(
            viewer, gripper_name,
            frame_color=frame_color,
            axis_radius=axis_radius,
            axis_length=axis_length
        )
        arrow_ok = True
        if show_approach:
            arrow_ok = displayGripperApproach(
                viewer, gripper_name,
                arrow_color=arrow_color,
                arrow_length=arrow_length,
                arrow_radius=arrow_radius,
                approach_direction=approach_direction
            )
        
        if frame_ok and arrow_ok:
            success_count += 1
            status = "frame and arrow" if show_approach else "frame"
            print(f"    ✓ {status} added")
        elif frame_ok:
            print("    ✓ Frame added (arrow failed)")
        else:
            print("    ✗ Failed")
    
    viewer.client.gui.refresh()
    print(f"\nSuccessfully displayed {success_count}/{len(gripper_names)} grippers")
    return success_count


def print_handle_info(viewer, handle_name: str) -> None:
    """
    Print detailed information about a handle.
    
    Args:
        viewer: Gepetto viewer instance
        handle_name: Full handle name
    """
    robot = viewer.robot
    handle_info = robot.getHandlePositionInJoint(handle_name)
    approach_dir = list(robot.getHandleApproachingDirection(handle_name))
    
    print(f"\nHandle: {handle_name}")
    print(f"  Joint: {handle_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {handle_info[1]}")
    print(f"  Approaching direction: {approach_dir}")


def print_gripper_info(viewer, gripper_name: str) -> None:
    """
    Print detailed information about a gripper.
    
    Args:
        viewer: Gepetto viewer instance
        gripper_name: Full gripper name
    """
    robot = viewer.robot
    gripper_info = robot.getGripperPositionInJoint(gripper_name)
    
    print(f"\nGripper: {gripper_name}")
    print(f"  Joint: {gripper_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {gripper_info[1]}")


def remove_visualization(viewer, name: str) -> bool:
    """
    Remove a visualization element from viewer.
    
    Args:
        viewer: Gepetto viewer instance
        name: Name of element to remove (e.g., "handle__box_handle2")
        
    Returns:
        True if successful
    """
    try:
        viewer.client.gui.deleteNode(name, True)
        return True
    except Exception:
        return False


def clear_handle_visualizations(viewer) -> int:
    """
    Clear all handle visualization elements (handle__ prefix).
    
    Args:
        viewer: Gepetto viewer instance
        
    Returns:
        Number of elements removed
    """
    count = 0
    try:
        nodes = viewer.client.gui.getNodeList()
        for node in nodes:
            if node.startswith("handle__"):
                if remove_visualization(viewer, node):
                    count += 1
        viewer.client.gui.refresh()
    except Exception as e:
        print(f"Warning: Could not clear handle visualizations: {e}")
    
    return count


def clear_gripper_visualizations(viewer) -> int:
    """
    Clear all gripper visualization elements (gripper__ prefix).
    
    Args:
        viewer: Gepetto viewer instance
        
    Returns:
        Number of elements removed
    """
    count = 0
    try:
        nodes = viewer.client.gui.getNodeList()
        for node in nodes:
            if node.startswith("gripper__"):
                if remove_visualization(viewer, node):
                    count += 1
        viewer.client.gui.refresh()
    except Exception as e:
        print(f"Warning: Could not clear gripper visualizations: {e}")
    
    return count


def clear_all_visualizations(viewer) -> int:
    """
    Clear all handle and gripper visualization elements.
    
    Args:
        viewer: Gepetto viewer instance
        
    Returns:
        Number of elements removed
    """
    handle_count = clear_handle_visualizations(viewer)
    gripper_count = clear_gripper_visualizations(viewer)
    return handle_count + gripper_count


# ============================================================================
# Example usage
# ============================================================================

if __name__ == "__main__":
    print("""
Visualize Frames Utility
========================

This module provides functions to visualize handles and grippers using
hpp.gepetto.viewer methods for frame display.

Example usage:

    from visualize_frames import (
        displayHandle, displayGripper,
        displayHandleApproach, displayGripperApproach
    )
    
    # Display handle frame only (attached to robot)
    displayHandle(viewer, "box/handle2")
    
    # Display gripper frame only (attached to robot)
    displayGripper(viewer, "pr2/l_gripper")
    
    # Display handle approach arrow
    displayHandleApproach(viewer, "box/handle2")
    
    # Display gripper approach arrow
    displayGripperApproach(viewer, "pr2/l_gripper")
    
    # Display all handles
    handles = ["box/handle", "box/handle2"]
    visualize_all_handles(viewer, handles, show_approach=True)
    
    # Display all grippers
    grippers = ["pr2/l_gripper", "pr2/r_gripper"]
    visualize_all_grippers(viewer, grippers, show_approach=True)
    
    # Print handle information
    print_handle_info(viewer, "box/handle2")
    
    # Clear visualizations
    clear_all_visualizations(viewer)

Available functions:
- displayHandle(): Display handle frame attached to robot (no config needed)
- displayGripper(): Display gripper frame attached to robot (no config needed)
- displayHandleApproach(): Display approach arrow for handle
- displayGripperApproach(): Display approach arrow for gripper
- visualize_all_handles(): Display multiple handles
- visualize_all_grippers(): Display multiple grippers
- print_handle_info(): Print handle details
- print_gripper_info(): Print gripper details
- clear_handle_visualizations(): Remove all handle__ elements
- clear_gripper_visualizations(): Remove all gripper__ elements
- clear_all_visualizations(): Remove all visualization elements

Note: Frames are attached to robot links, so they automatically move with
      robot configuration updates. No need to pass configuration vectors.
""")
