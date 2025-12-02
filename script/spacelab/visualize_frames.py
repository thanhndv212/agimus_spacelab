#!/usr/bin/env python3
"""
Utility for visualizing handle and gripper frames with approaching directions.

This module provides functions to display frames and arrows in gepetto-viewer
for any robot handles and grippers, showing their poses and approach vectors.
"""

import numpy as np
from typing import List, Optional, Tuple
from pinocchio import SE3, Quaternion


def transform_to_list(T: SE3) -> List[float]:
    """
    Convert SE3 transform to pose list.
    
    Args:
        T: SE3 transformation matrix
        
    Returns:
        [x, y, z, qw, qx, qy, qz]
    """
    pos = T.translation
    quat = Quaternion(T.rotation)
    return [pos[0], pos[1], pos[2], quat.w, quat.x, quat.y, quat.z]


def pose_to_SE3(pose: List[float]) -> SE3:
    """
    Convert pose list to SE3 transform.
    
    Args:
        pose: [x, y, z, qw, qx, qy, qz]
        
    Returns:
        SE3 transformation matrix
    """
    trans = np.array(pose[:3])
    quat = Quaternion(pose[3], pose[4], pose[5], pose[6])  # w, x, y, z
    return SE3(quat.matrix(), trans)


def get_joint_transform(robot, q: List[float], joint_name: str) -> SE3:
    """
    Get world transform of a joint in configuration q.
    
    Args:
        robot: Robot instance
        q: Configuration vector
        joint_name: Name of the joint
        
    Returns:
        SE3 world transform
    """
    robot.setCurrentConfig(q)
    if joint_name == "universe":
        return SE3.Identity()
    T = robot.getJointPosition(joint_name)
    return pose_to_SE3(T)


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


def visualize_handle(
    viewer,
    robot,
    q: List[float],
    handle_name: str,
    frame_color: Optional[List[float]] = None,
    arrow_color: Optional[List[float]] = None,
    frame_scale: float = 0.1,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008
) -> Tuple[bool, bool]:
    """
    Visualize a handle frame and its approaching direction.
    
    Args:
        viewer: Gepetto viewer instance
        robot: Robot instance
        q: Configuration vector
        handle_name: Full handle name (e.g., "box/handle2")
        frame_color: RGBA color for frame [r, g, b, a] (default: green)
        arrow_color: RGBA color for arrow [r, g, b, a] (default: cyan)
        frame_scale: Size of XYZ frame axes
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow
        
    Returns:
        (frame_success, arrow_success): Success flags for frame and arrow
    """
    if frame_color is None:
        frame_color = [0, 0.8, 0, 1]  # Green
    if arrow_color is None:
        arrow_color = [0, 1, 1, 1]  # Cyan
    
    # Get handle information
    handle_info = robot.getHandlePositionInJoint(handle_name)
    joint_name = handle_info[0]
    local_pose = handle_info[1]
    approach_dir = np.array(list(robot.getHandleApproachingDirection(handle_name)))
    
    # Compute world transform
    joint_T = get_joint_transform(robot, q, joint_name)
    handle_local_T = pose_to_SE3(local_pose)
    handle_world_T = joint_T * handle_local_T
    handle_world_pose = transform_to_list(handle_world_T)
    
    # Transform approach direction to world frame
    approach_world = handle_world_T.rotation @ approach_dir
    
    # Create safe GUI names
    safe_name = handle_name.replace('/', '_')
    frame_name = f"hpp-gui/{safe_name}_frame"
    arrow_name = f"hpp-gui/{safe_name}_approach"
    
    # Add frame
    frame_success = False
    try:
        viewer.client.gui.addXYZaxis(frame_name, frame_color, arrow_radius, frame_scale)
        viewer.client.gui.applyConfiguration(frame_name, handle_world_pose)
        frame_success = True
    except Exception as e:
        print(f"  Warning: Could not add frame {frame_name}: {e}")
    
    # Add approach arrow
    arrow_success = False
    try:
        viewer.client.gui.addArrow(arrow_name, arrow_radius, arrow_length, arrow_color)
        
        # Compute arrow pose
        arrow_quat = compute_arrow_orientation(approach_world)
        start_pt = handle_world_T.translation
        arrow_pose = [start_pt[0], start_pt[1], start_pt[2],
                      arrow_quat.w, arrow_quat.x, arrow_quat.y, arrow_quat.z]
        
        viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)
        arrow_success = True
    except Exception as e:
        print(f"  Warning: Could not add arrow {arrow_name}: {e}")
    
    return frame_success, arrow_success


def visualize_gripper(
    viewer,
    robot,
    q: List[float],
    gripper_name: str,
    frame_color: Optional[List[float]] = None,
    arrow_color: Optional[List[float]] = None,
    frame_scale: float = 0.1,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None
) -> Tuple[bool, bool]:
    """
    Visualize a gripper frame and its approaching direction.
    
    Args:
        viewer: Gepetto viewer instance
        robot: Robot instance
        q: Configuration vector
        gripper_name: Full gripper name (e.g., "pr2/l_gripper")
        frame_color: RGBA color for frame [r, g, b, a] (default: red)
        arrow_color: RGBA color for arrow [r, g, b, a] (default: orange)
        frame_scale: Size of XYZ frame axes
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow
        approach_direction: Approach direction in gripper frame (default: [1, 0, 0])
        
    Returns:
        (frame_success, arrow_success): Success flags for frame and arrow
    """
    if frame_color is None:
        frame_color = [1, 0, 0, 1]  # Red
    if arrow_color is None:
        arrow_color = [1, 0.5, 0, 1]  # Orange
    if approach_direction is None:
        approach_direction = [1, 0, 0]  # X-axis
    
    # Get gripper information
    gripper_info = robot.getGripperPositionInJoint(gripper_name)
    gripper_joint = gripper_info[0]
    gripper_local_pose = gripper_info[1]
    
    # Compute world transform
    gripper_joint_T = get_joint_transform(robot, q, gripper_joint)
    gripper_local_T = pose_to_SE3(gripper_local_pose)
    gripper_world_T = gripper_joint_T * gripper_local_T
    gripper_world_pose = transform_to_list(gripper_world_T)
    
    # Transform approach direction to world frame
    approach_vec = np.array(approach_direction)
    approach_world = gripper_world_T.rotation @ approach_vec
    
    # Create safe GUI names
    safe_name = gripper_name.replace('/', '_')
    frame_name = f"hpp-gui/{safe_name}_frame"
    arrow_name = f"hpp-gui/{safe_name}_approach"
    
    # Add frame
    frame_success = False
    try:
        viewer.client.gui.addXYZaxis(frame_name, frame_color, arrow_radius, frame_scale)
        viewer.client.gui.applyConfiguration(frame_name, gripper_world_pose)
        frame_success = True
    except Exception as e:
        print(f"  Warning: Could not add frame {frame_name}: {e}")
    
    # Add approach arrow
    arrow_success = False
    try:
        viewer.client.gui.addArrow(arrow_name, arrow_radius, arrow_length, arrow_color)
        
        # Compute arrow pose
        arrow_quat = compute_arrow_orientation(approach_world)
        start_pt = gripper_world_T.translation
        arrow_pose = [start_pt[0], start_pt[1], start_pt[2],
                      arrow_quat.w, arrow_quat.x, arrow_quat.y, arrow_quat.z]
        
        viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)
        arrow_success = True
    except Exception as e:
        print(f"  Warning: Could not add arrow {arrow_name}: {e}")
    
    return frame_success, arrow_success


def visualize_all_handles(
    viewer,
    robot,
    q: List[float],
    handle_names: List[str],
    **kwargs
) -> int:
    """
    Visualize multiple handles at once.
    
    Args:
        viewer: Gepetto viewer instance
        robot: Robot instance
        q: Configuration vector
        handle_names: List of handle names
        **kwargs: Additional arguments passed to visualize_handle
        
    Returns:
        Number of successfully visualized handles
    """
    print(f"\nVisualizing {len(handle_names)} handles...")
    success_count = 0
    
    for handle_name in handle_names:
        print(f"  {handle_name}")
        frame_ok, arrow_ok = visualize_handle(viewer, robot, q, handle_name, **kwargs)
        if frame_ok and arrow_ok:
            success_count += 1
            print(f"    ✓ Frame and arrow added")
        elif frame_ok:
            print(f"    ✓ Frame added (arrow failed)")
        elif arrow_ok:
            print(f"    ✓ Arrow added (frame failed)")
        else:
            print(f"    ✗ Failed")
    
    viewer.client.gui.refresh()
    print(f"\nSuccessfully visualized {success_count}/{len(handle_names)} handles")
    return success_count


def visualize_all_grippers(
    viewer,
    robot,
    q: List[float],
    gripper_names: List[str],
    **kwargs
) -> int:
    """
    Visualize multiple grippers at once.
    
    Args:
        viewer: Gepetto viewer instance
        robot: Robot instance
        q: Configuration vector
        gripper_names: List of gripper names
        **kwargs: Additional arguments passed to visualize_gripper
        
    Returns:
        Number of successfully visualized grippers
    """
    print(f"\nVisualizing {len(gripper_names)} grippers...")
    success_count = 0
    
    for gripper_name in gripper_names:
        print(f"  {gripper_name}")
        frame_ok, arrow_ok = visualize_gripper(viewer, robot, q, gripper_name, **kwargs)
        if frame_ok and arrow_ok:
            success_count += 1
            print(f"    ✓ Frame and arrow added")
        elif frame_ok:
            print(f"    ✓ Frame added (arrow failed)")
        elif arrow_ok:
            print(f"    ✓ Arrow added (frame failed)")
        else:
            print(f"    ✗ Failed")
    
    viewer.client.gui.refresh()
    print(f"\nSuccessfully visualized {success_count}/{len(gripper_names)} grippers")
    return success_count


def print_handle_info(robot, handle_name: str) -> None:
    """
    Print detailed information about a handle.
    
    Args:
        robot: Robot instance
        handle_name: Full handle name
    """
    handle_info = robot.getHandlePositionInJoint(handle_name)
    approach_dir = list(robot.getHandleApproachingDirection(handle_name))
    
    print(f"\nHandle: {handle_name}")
    print(f"  Joint: {handle_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {handle_info[1]}")
    print(f"  Approaching direction: {approach_dir}")


def print_gripper_info(robot, gripper_name: str) -> None:
    """
    Print detailed information about a gripper.
    
    Args:
        robot: Robot instance
        gripper_name: Full gripper name
    """
    gripper_info = robot.getGripperPositionInJoint(gripper_name)
    
    print(f"\nGripper: {gripper_name}")
    print(f"  Joint: {gripper_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {gripper_info[1]}")


def remove_visualization(viewer, name: str) -> bool:
    """
    Remove a visualization element from viewer.
    
    Args:
        viewer: Gepetto viewer instance
        name: Name of element to remove
        
    Returns:
        True if successful
    """
    try:
        viewer.client.gui.deleteNode(name, True)
        return True
    except Exception:
        return False


def clear_all_visualizations(viewer, prefix: str = "hpp-gui/") -> int:
    """
    Clear all visualization elements with given prefix.
    
    Args:
        viewer: Gepetto viewer instance
        prefix: Prefix of elements to remove
        
    Returns:
        Number of elements removed
    """
    count = 0
    try:
        nodes = viewer.client.gui.getNodeList()
        for node in nodes:
            if node.startswith(prefix):
                if remove_visualization(viewer, node):
                    count += 1
        viewer.client.gui.refresh()
    except Exception as e:
        print(f"Warning: Could not clear visualizations: {e}")
    
    return count


# ============================================================================
# Example usage
# ============================================================================

if __name__ == "__main__":
    print("""
Visualize Frames Utility
========================

This module provides functions to visualize handles and grippers.

Example usage:

    from visualize_frames import visualize_handle, visualize_gripper
    
    # Visualize a handle
    visualize_handle(viewer, robot, q_init, "box/handle2")
    
    # Visualize a gripper
    visualize_gripper(viewer, robot, q_init, "pr2/l_gripper")
    
    # Visualize all handles
    handles = ["box/handle", "box/handle2"]
    visualize_all_handles(viewer, robot, q_init, handles)
    
    # Print handle information
    print_handle_info(robot, "box/handle2")
    
    # Clear visualizations
    clear_all_visualizations(viewer)

Available functions:
- visualize_handle(): Display single handle frame and approach arrow
- visualize_gripper(): Display single gripper frame and approach arrow
- visualize_all_handles(): Display multiple handles
- visualize_all_grippers(): Display multiple grippers
- print_handle_info(): Print handle details
- print_gripper_info(): Print gripper details
- clear_all_visualizations(): Remove all visualization elements
""")
