#!/usr/bin/env python3
"""
Shared utilities and base classes for Spacelab manipulation tasks.

DEPRECATED: This module location is deprecated.
Please use the new module structure:
  - from agimus_spacelab.planning import SceneBuilder, GraphBuilder, ...
  - from agimus_spacelab.tasks import TaskOrchestrator, ManipulationTask, ...
  - from agimus_spacelab.visualization import visualize_constraint_graph, ...

This module provides reusable components for building manipulation planning:
- SpaceLabSceneBuilder: Set up robots, objects, and environment
- GraphBuilder: Create constraint graphs (manual or factory-based)
- ConstraintBuilder: Create transformation constraints
- ConfigurationGenerator: Generate and validate configurations
- ManipulationTask: Base class for task implementation
- Visualization utilities: Graph visualization and debug tools
"""

import warnings

# Show deprecation warning
warnings.warn(
    "Importing from script.spacelab.spacelab_tools is deprecated. "
    "Use the new module structure:\n"
    "  - from agimus_spacelab.planning import SceneBuilder, GraphBuilder\n"
    "  - from agimus_spacelab.tasks import TaskOrchestrator, ManipulationTask\n"
    "  - from agimus_spacelab.visualization import visualize_constraint_graph",
    DeprecationWarning,
    stacklevel=2
)

# Import from new module locations
try:
    from agimus_spacelab.planning import (
        SceneBuilder as SpaceLabSceneBuilder,
        GraphBuilder,
        ConstraintBuilder,
        ConfigGenerator as ConfigurationGenerator,
    )
    from agimus_spacelab.tasks import (
        ManipulationTask,
        TaskOrchestrator,
        TaskBuilder,
        PlanningBridge as ManipulationPlannerBridge,
        create_grasp_task,
        create_place_task,
    )
    from agimus_spacelab.tasks.orchestration import (
        TaskStatus,
        ResourceType,
        Resource,
        AtomicTask,
        ResourceManager,
        TaskDependencyGraph,
    )
    from agimus_spacelab.tasks.bridge import PlanningContext
    from agimus_spacelab.visualization import (
        print_joint_info,
        visualize_handle_frames,
        visualize_constraint_graph,
    )
except ImportError as e:
    raise ImportError(
        f"Failed to import from agimus_spacelab: {e}\n"
        "The modules have been reorganized. Please ensure the package is installed.\n"
        "New locations:\n"
        "  - agimus_spacelab.planning (SceneBuilder, GraphBuilder, etc.)\n"
        "  - agimus_spacelab.tasks (TaskOrchestrator, ManipulationTask, etc.)\n"
        "  - agimus_spacelab.visualization (print_joint_info, visualize_*, etc.)"
    )


__all__ = [
    "SpaceLabSceneBuilder",
    "GraphBuilder",
    "ConstraintBuilder",
    "ConfigurationGenerator",
    "ManipulationTask",
    "print_joint_info",
    "visualize_handle_frames",
    "visualize_constraint_graph",
    # Orchestration
    "TaskStatus",
    "ResourceType",
    "Resource",
    "AtomicTask",
    "ResourceManager",
    "TaskDependencyGraph",
    "TaskOrchestrator",
    "TaskBuilder",
    # Bridge
    "PlanningContext",
    "ManipulationPlannerBridge",
    "create_grasp_task",
    "create_place_task",
]
