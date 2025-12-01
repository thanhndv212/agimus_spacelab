#!/usr/bin/env python3
"""
Example: Multi-arm collaborative assembly of RS modules.

Scenario:
  1. UR10 picks up frame_gripper from dispenser
  2. VISPA picks up RS1 from dispenser  
  3. UR10 (with frame_gripper) grasps RS2
  4. UR10 transports RS2 to assembly zone
  5. VISPA moves RS1 to assembly zone
  6. UR10 and VISPA collaboratively assemble RS1+RS2

This demonstrates:
- Parallel task execution (UR10 and VISPA work simultaneously)
- Resource management (no conflicts)
- Dependencies (assembly requires both parts ready)
- Synchronization points (collaborative assembly)
"""

import sys
from pathlib import Path

# Add script directory
sys.path.insert(0, str(Path(__file__).parent))

from task_orchestration import (
    TaskOrchestrator,
    TaskBuilder,
    AtomicTask,
)


# ============================================================================
# Mock Execution Functions (Replace with real planning)
# ============================================================================

def mock_ur10_grasp_frame_gripper() -> bool:
    """UR10 grasps frame_gripper from dispenser."""
    print("    [UR10] Planning path to grasp frame_gripper...")
    print("    [UR10] Executing grasp motion...")
    # Here: call actual manipulation planner
    # planner.solve(initial=q_init, goal=q_grasp)
    return True


def mock_vispa_grasp_rs1() -> bool:
    """VISPA grasps RS1 from dispenser."""
    print("    [VISPA] Planning path to grasp RS1...")
    print("    [VISPA] Executing grasp motion...")
    return True


def mock_ur10_grasp_rs2_with_tool() -> bool:
    """UR10 (holding frame_gripper) grasps RS2."""
    print("    [UR10+FG] Planning path to grasp RS2...")
    print("    [UR10+FG] Executing grasp motion...")
    return True


def mock_ur10_transport_rs2() -> bool:
    """UR10 transports RS2 to assembly zone."""
    print("    [UR10+FG+RS2] Planning path to assembly zone...")
    print("    [UR10+FG+RS2] Executing transport motion...")
    return True


def mock_vispa_transport_rs1() -> bool:
    """VISPA transports RS1 to assembly zone."""
    print("    [VISPA+RS1] Planning path to assembly zone...")
    print("    [VISPA+RS1] Executing transport motion...")
    return True


def mock_collaborative_assembly() -> bool:
    """UR10 and VISPA collaboratively assemble RS1+RS2."""
    print("    [UR10+VISPA] Synchronizing positions...")
    print("    [UR10] Aligning RS2 with RS1...")
    print("    [VISPA] Holding RS1 stable...")
    print("    [UR10+VISPA] Executing assembly motion...")
    return True


# ============================================================================
# Task Definitions
# ============================================================================

def create_assembly_tasks() -> list[AtomicTask]:
    """Create task graph for RS1+RS2 assembly."""
    
    tasks = []
    
    # Task 1: UR10 grasps frame_gripper
    task_ur10_grasp_fg = (
        TaskBuilder("t1_ur10_grasp_fg", "UR10 grasp frame_gripper")
        .with_description("UR10 picks up frame_gripper from dispenser")
        .requires_arm("UR10")
        .requires_object("frame_gripper")
        .requires_workspace("dispenser_zone")
        .with_execution(mock_ur10_grasp_frame_gripper)
        .with_postcondition(
            "frame_gripper grasped",
            lambda: True  # In reality: check gripper sensors
        )
        .with_timeout(60.0)
        .build()
    )
    tasks.append(task_ur10_grasp_fg)
    
    # Task 2: VISPA grasps RS1 (can run in parallel with Task 1)
    task_vispa_grasp_rs1 = (
        TaskBuilder("t2_vispa_grasp_rs1", "VISPA grasp RS1")
        .with_description("VISPA picks up RS1 from dispenser")
        .requires_arm("VISPA")
        .requires_object("RS1")
        .requires_workspace("dispenser_zone")
        .with_execution(mock_vispa_grasp_rs1)
        .with_postcondition("RS1 grasped", lambda: True)
        .with_timeout(60.0)
        .build()
    )
    tasks.append(task_vispa_grasp_rs1)
    
    # Task 3: UR10 (with frame_gripper) grasps RS2
    # Depends on Task 1 completing
    task_ur10_grasp_rs2 = (
        TaskBuilder("t3_ur10_grasp_rs2", "UR10+FG grasp RS2")
        .with_description("UR10 (holding frame_gripper) grasps RS2")
        .depends_on("t1_ur10_grasp_fg")
        .requires_arm("UR10")
        .requires_object("RS2")
        .requires_workspace("dispenser_zone")
        .with_precondition(
            "UR10 has frame_gripper",
            lambda: True  # Check UR10 gripper state
        )
        .with_execution(mock_ur10_grasp_rs2_with_tool)
        .with_postcondition("RS2 grasped by frame_gripper", lambda: True)
        .with_timeout(60.0)
        .build()
    )
    tasks.append(task_ur10_grasp_rs2)
    
    # Task 4: UR10 transports RS2 to assembly zone
    # Depends on Task 3
    task_ur10_transport = (
        TaskBuilder("t4_ur10_transport", "UR10 transport RS2")
        .with_description("UR10 moves RS2 to assembly zone")
        .depends_on("t3_ur10_grasp_rs2")
        .requires_arm("UR10")
        .requires_workspace("assembly_zone")
        .with_execution(mock_ur10_transport_rs2)
        .with_postcondition("RS2 in assembly zone", lambda: True)
        .with_timeout(60.0)
        .build()
    )
    tasks.append(task_ur10_transport)
    
    # Task 5: VISPA transports RS1 to assembly zone
    # Depends on Task 2
    # NOTE: Can run in parallel with Task 3-4 if resources don't conflict
    task_vispa_transport = (
        TaskBuilder("t5_vispa_transport", "VISPA transport RS1")
        .with_description("VISPA moves RS1 to assembly zone")
        .depends_on("t2_vispa_grasp_rs1")
        .requires_arm("VISPA")
        .requires_workspace("assembly_zone")
        .with_execution(mock_vispa_transport_rs1)
        .with_postcondition("RS1 in assembly zone", lambda: True)
        .with_timeout(60.0)
        .build()
    )
    tasks.append(task_vispa_transport)
    
    # Task 6: Collaborative assembly
    # Depends on both Task 4 and Task 5
    # Requires both arms simultaneously
    task_assembly = (
        TaskBuilder("t6_assembly", "Assemble RS1+RS2")
        .with_description("UR10 and VISPA collaboratively assemble RS1 to RS2")
        .depends_on("t4_ur10_transport", "t5_vispa_transport")
        .requires_arm("UR10")
        .requires_arm("VISPA")
        .requires_workspace("assembly_zone")
        .with_precondition("Both parts in assembly zone", lambda: True)
        .with_execution(mock_collaborative_assembly)
        .with_postcondition("RS1+RS2 assembled", lambda: True)
        .with_timeout(120.0)
        .build()
    )
    tasks.append(task_assembly)
    
    return tasks


# ============================================================================
# Main Execution
# ============================================================================

def main(dry_run: bool = False):
    """
    Run multi-arm collaborative assembly.
    
    Args:
        dry_run: If True, show execution plan without running
    """
    print("=" * 70)
    print("MULTI-ARM COLLABORATIVE ASSEMBLY")
    print("Scenario: UR10 + VISPA assemble RS1 + RS2")
    print("=" * 70)
    
    # Create orchestrator
    orchestrator = TaskOrchestrator(max_concurrent_tasks=2)
    
    # Setup resources
    orchestrator.setup_resources(
        arms=["UR10", "VISPA"],
        objects=["frame_gripper", "RS1", "RS2"],
        workspaces=["dispenser_zone", "assembly_zone"]
    )
    
    # Add tasks
    tasks = create_assembly_tasks()
    for task in tasks:
        orchestrator.add_task(task)
        
    # Show task graph
    print("\nTask Graph:")
    for task in tasks:
        deps_str = f" (depends on: {', '.join(task.depends_on)})" if task.depends_on else ""
        resources_str = ", ".join(
            f"{r.resource_type.value}:{r.name}" 
            for r in task.required_resources
        )
        print(f"  {task.task_id}: {task.name}")
        print(f"    Resources: {resources_str}")
        print(f"    {deps_str}")
        
    # Run orchestration
    success = orchestrator.run(dry_run=dry_run)
    
    if success:
        print("\n✓ Assembly workflow completed successfully!")
    else:
        print("\n✗ Assembly workflow failed")
        
    return orchestrator


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Multi-arm collaborative assembly example"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show execution plan without running tasks"
    )
    
    args = parser.parse_args()
    
    orchestrator = main(dry_run=args.dry_run)
    
    print("\n" + "=" * 70)
    print("Example complete!")
    print("=" * 70)
    print("\nThe orchestrator object is available for inspection.")
    print("Try: orchestrator.dependency_graph.get_execution_summary()")
    print("     orchestrator.resource_manager.get_allocation_summary()")
