# SpaceLab Manipulation Planning

Multi-arm collaborative manipulation framework for SpaceLab assembly tasks using HPP (Humanoid Path Planner).

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Components](#components)
- [Multi-Arm Collaboration](#multi-arm-collaboration)
- [Getting Started](#getting-started)
- [Creating New Tasks](#creating-new-tasks)
- [Development Roadmap](#development-roadmap)

---

## Overview

This framework enables UR10 and VISPA robots to collaboratively assemble RS modules through:

1. **Modular Design**: Separation of configuration, tools, and task logic
2. **Task Orchestration**: Dependency-based scheduling with resource management
3. **Parallel Execution**: Concurrent task execution across multiple arms
4. **Behavior Trees**: Dynamic task composition and failure recovery (planned)

### Key Features

✅ Reusable manipulation planning utilities  
✅ Fluent APIs for scene setup and constraint creation  
✅ Task orchestration with resource conflict resolution  
✅ Multi-arm coordination framework  
✅ Scene visualization tools  

---

## Architecture

### File Structure

```
script/
├── config/
│   └── spacelab_config.py           # Configuration data
├── spacelab/
│   ├── spacelab_tools.py            # Shared utilities
│   ├── task_orchestration.py        # Multi-arm orchestration
│   ├── task_grasp_frame_gripper.py  # Example task
│   ├── example_collaborative_assembly.py  # Multi-arm demo
│   ├── visualize_scene.py           # Scene visualization
│   └── README.md                    # This file
```

### System Layers

```
┌─────────────────────────────────────────────────────────────┐
│                   Assembly Mission                           │
│              "Assemble RS1-RS6 structure"                    │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                 Behavior Tree Layer                          │
│   - Decompose mission into sub-goals                         │
│   - Select task sequences dynamically                        │
│   - Handle failures and replanning                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              Task Orchestration Layer                        │
│   - Manage task dependencies                                 │
│   - Allocate resources (arms, objects, workspace)            │
│   - Execute concurrent tasks                                 │
│   - Synchronize multi-arm actions                            │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│               Atomic Task Layer                              │
│   - Individual manipulation primitives                       │
│   - Grasp, Place, Transport, Assemble, Handover             │
│   - Pre/postconditions                                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│            Motion Planning Layer                             │
│   - HPP manipulation planning (existing)                     │
│   - Collision-free paths                                     │
│   - Constraint satisfaction                                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Components

### 1. Configuration (`spacelab_config.py`)

Centralized configuration for all manipulation tasks.

**Classes:**
- `RobotJoints`: Joint names for UR10, VISPA, and objects
- `InitialConfigurations`: Initial joint angles and object poses (XYZRPY)
- `JointBounds`: Joint limits and freeflyer bounds
- `ManipulationConfig`: Grippers, handles, and valid grasp pairs

**Usage:**
```python
from spacelab_config import InitialConfigurations, ManipulationConfig

# Get robot initial config
q_ur10 = InitialConfigurations.UR10

# Get valid gripper-handle pairs
pairs = ManipulationConfig.VALID_PAIRS["ur10_gripper"]
```

---

### 2. Shared Tools (`spacelab_tools.py`)

Reusable components for building manipulation tasks.

#### **SpaceLabSceneBuilder**

Fluent API for scene setup:

```python
from spacelab_tools import SpaceLabSceneBuilder

# Quick setup
scene_builder = SpaceLabSceneBuilder()
planner, robot, ps = scene_builder.build(
    objects=["frame_gripper", "RS1"],
    validation_step=0.01
)

# Step-by-step setup
scene_builder = (SpaceLabSceneBuilder()
    .load_robot()
    .load_environment()
    .load_objects(["frame_gripper", "RS1"])
    .set_joint_bounds()
    .setup_collision_pairs()
    .configure_path_validation(0.01, 0.1))

planner, robot, ps = scene_builder.get_instances()
```

#### **ConstraintBuilder**

Helper for creating manipulation constraints:

```python
from spacelab_tools import ConstraintBuilder

# Grasp constraint
ConstraintBuilder.create_grasp_constraint(
    ps, "grasp", gripper_name, tool_name, transform, mask
)

# Placement constraint
ConstraintBuilder.create_placement_constraint(
    ps, "placement", tool_name, world_pose, mask
)

# Complement constraint (for security margins)
ConstraintBuilder.create_complement_constraint(
    ps, "complement", tool_name, world_pose, complement_mask
)
```

#### **ConfigurationGenerator**

Manages configuration generation and projection:

```python
from spacelab_tools import ConfigurationGenerator

config_gen = ConfigurationGenerator(robot, graph, ps, max_attempts=1000)

# Project onto constraint graph node
config_gen.project_on_node("placement", q_init, config_label="q_init")

# Generate configuration via edge
success, q_target = config_gen.generate_via_edge(
    "approach-tool", q_from, config_label="q_approach"
)

# Modify object pose
q_new = config_gen.modify_object_pose(
    q, object_index=0, translation_delta=[0.2, 0.0, 0.0]
)
```

#### **ManipulationTask** (Base Class)

Abstract base for task implementation:

```python
from spacelab_tools import ManipulationTask

class MyTask(ManipulationTask):
    def get_objects(self) -> List[str]:
        return ["frame_gripper"]
        
    def create_constraints(self) -> None:
        # Create constraints using ConstraintBuilder
        pass
        
    def create_graph(self) -> ConstraintGraph:
        # Build constraint graph
        pass
        
    def generate_configurations(self, q_init) -> Dict:
        # Generate waypoint configs using ConfigurationGenerator
        pass
```

---

### 3. Task Orchestration (`task_orchestration.py`)

Multi-arm task coordination framework.

#### **Key Classes:**

**AtomicTask**: Indivisible manipulation action
```python
from task_orchestration import AtomicTask, Resource, ResourceType


task = AtomicTask(
    task_id="t1",
    name="Grasp RS1",
    execute=lambda: grasp_rs1(),
    preconditions={"RS1_at_dispenser"},
    postconditions={"RS1_grasped"},
    required_resources={
        Resource(ResourceType.ARM, "UR10"),
        Resource(ResourceType.OBJECT, "RS1")
    }
)
```

**TaskOrchestrator**: Executes tasks respecting dependencies
```python
from task_orchestration import TaskOrchestrator, TaskBuilder

orchestrator = TaskOrchestrator(max_concurrent_tasks=2)
orchestrator.setup_resources(
    arms=["UR10", "VISPA"],
    objects=["RS1", "RS2"]
)

# Add tasks using builder
t1 = TaskBuilder("t1", "UR10 grasp RS1").requires_arm("UR10").build()
t2 = TaskBuilder("t2", "VISPA grasp RS2").requires_arm("VISPA").build()

orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.run()  # Executes in parallel
```

---

### 4. Scene Visualization (`visualize_scene.py`)

Interactive scene visualization with gepetto-viewer.

**Usage:**
```bash
# Load complete scene
python visualize_scene.py

# Load specific objects
python visualize_scene.py --objects RS1 RS2 frame_gripper

# Minimal scene (robots + environment only)
python visualize_scene.py --minimal

# Setup without interactive mode
python visualize_scene.py --no-interactive
```

**Interactive Commands:**
- `refresh` - Refresh display
- `info` - Show scene information
- `help` - Show available commands
- `quit` - Exit

---

## Multi-Arm Collaboration

### Implementation Status

#### ✅ Completed

1. **Core Infrastructure** (`task_orchestration.py`)
   - AtomicTask: Indivisible manipulation actions
   - ResourceManager: Arm/object/workspace allocation
   - TaskDependencyGraph: Dependency tracking
   - TaskOrchestrator: Execution engine with concurrency

2. **Example** (`example_collaborative_assembly.py`)
   - Multi-arm assembly scenario (UR10 + VISPA)
   - Parallel task execution
   - Resource conflict resolution
   - Dependency-based scheduling

3. **Integration Points**
   - Hooks for real motion planning
   - Precondition/postcondition framework
   - Timeout and retry logic

#### 🔄 Development Roadmap

**Phase 1: Integration with Existing Planners** (HIGH PRIORITY)
- Bridge `ManipulationTask` to `AtomicTask`
- Replace mock execution functions
- Connect to real HPP planning

**Phase 2: Behavior Tree Framework** (HIGH PRIORITY)
- Implement Sequence, Selector, Parallel nodes
- Enable dynamic task composition
- Add failure recovery mechanisms

**Phase 3: Synchronization Primitives** (MEDIUM PRIORITY)
- Synchronization barriers for multi-arm coordination
- Handover primitives between arms
- Temporal alignment for collaborative actions

**Phase 4: Failure Recovery** (MEDIUM PRIORITY)
- Retry strategies
- Alternative path planning
- Resource reassignment

**Phase 5: Learning & Optimization** (LOW PRIORITY)
- Learn optimal task orderings
- Execution time prediction
- Adaptive scheduling

---

## Getting Started

### Prerequisites

```bash
# HPP stack
sudo apt install robotpkg-hpp-manipulation-corba
sudo apt install robotpkg-gepetto-viewer

# Python dependencies
pip install numpy pinocchio
```

### Quick Start

#### 1. Visualize the Scene

```bash
cd script/spacelab
python visualize_scene.py
```

#### 2. Run Example Task

```bash
# Grasp frame_gripper task (visualization only)
python task_grasp_frame_gripper.py

# With path planning
python task_grasp_frame_gripper.py --solve

# Without visualization
python task_grasp_frame_gripper.py --no-viz --solve
```

#### 3. Run Multi-Arm Example

```bash
# Dry run (show execution plan)
python example_collaborative_assembly.py --dry-run

# Execute with mock functions
python example_collaborative_assembly.py
```

---

## Creating New Tasks

### Pattern 1: Single-Arm Task

```python
from spacelab_tools import ManipulationTask, SpaceLabSceneBuilder
from spacelab_tools import ConstraintBuilder, ConfigurationGenerator

class GraspRS1Task(ManipulationTask):
    def __init__(self):
        super().__init__("Grasp RS1")
        
    def get_objects(self):
        return ["RS1"]
        
    def create_constraints(self):
        # Use ConstraintBuilder
        ConstraintBuilder.create_grasp_constraint(
            self.ps, "grasp", "spacelab/ur10_joint_6_7", 
            "RS1/root_joint", self.grasp_transform, self.mask
        )
        
    def create_graph(self):
        graph = ConstraintGraph(self.robot, "graph")
        graph.createNode(["free", "grasp"])
        graph.createEdge("free", "grasp", "pick", weight=1)
        return graph
        
    def generate_configurations(self, q_init):
        self.config_gen.project_on_node("free", q_init, "q_init")
        success, q_grasp = self.config_gen.generate_via_edge(
            "pick", self.config_gen.configs["q_init"], "q_grasp"
        )
        return self.config_gen.configs

# Run task
if __name__ == "__main__":
    task = GraspRS1Task()
    task.setup()
    task.run(visualize=True, solve=False)
```

### Pattern 2: Multi-Arm Collaboration

```python
from task_orchestration import TaskOrchestrator, TaskBuilder

# Setup orchestrator
orchestrator = TaskOrchestrator(max_concurrent_tasks=2)
orchestrator.setup_resources(
    arms=["UR10", "VISPA"],
    objects=["RS1", "RS2"]
)

# Define parallel tasks
t1 = (TaskBuilder("t1", "UR10 grasp RS1")
    .requires_arm("UR10")
    .requires_object("RS1")
    .execute_with(ur10_grasp_rs1_fn)
    .build())

t2 = (TaskBuilder("t2", "VISPA grasp RS2")
    .requires_arm("VISPA")
    .requires_object("RS2")
    .execute_with(vispa_grasp_rs2_fn)
    .build())

# Define collaborative task (depends on both)
t3 = (TaskBuilder("t3", "Assemble RS1+RS2")
    .depends_on("t1", "t2")
    .requires_arm("UR10")
    .requires_arm("VISPA")
    .execute_with(assembly_fn)
    .build())

# Execute
orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.add_task(t3)
orchestrator.run()  # t1 and t2 run in parallel, then t3
```

---

## Usage Patterns

### Simple Sequential Assembly

```python
orchestrator = TaskOrchestrator()
orchestrator.setup_resources(arms=["UR10"], objects=["RS1", "RS2"])

t1 = TaskBuilder("t1", "Grasp RS1").requires_arm("UR10").build()
t2 = TaskBuilder("t2", "Grasp RS2").depends_on("t1").requires_arm("UR10").build()

orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.run()
```

### Parallel Execution

```python
# Both run concurrently (different arms, no conflicts)
t1 = TaskBuilder("t1", "UR10 work").requires_arm("UR10").build()
t2 = TaskBuilder("t2", "VISPA work").requires_arm("VISPA").build()

orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.run()
```

### Collaborative Task

```python
# Both arms needed simultaneously
t_prep1 = TaskBuilder("prep1", "UR10 prep").requires_arm("UR10").build()
t_prep2 = TaskBuilder("prep2", "VISPA prep").requires_arm("VISPA").build()

t_collab = (TaskBuilder("collab", "Assembly")
    .depends_on("prep1", "prep2")
    .requires_arm("UR10")
    .requires_arm("VISPA")
    .build())

# prep1 and prep2 run in parallel, then collab uses both arms
```

---

## Comparison: Before vs After Refactoring

### Before
- **800+ lines** of monolithic code per task
- Scene setup mixed with task logic
- Constraint creation scattered
- Configuration generation in main function
- Hard to reuse for new tasks

### After
- **500 lines** in task file
- **700 lines** of reusable tools
- Clear separation of concerns
- Task focuses on "what" not "how"
- Easy to create new tasks (~200 lines)

### Benefits

✅ **Reusability**: ~700 lines of shared utilities  
✅ **Maintainability**: Centralized bug fixes  
✅ **Readability**: High-level task logic clear  
✅ **Extensibility**: Easy to add features  
✅ **Collaboration**: Multi-arm orchestration  

---

## Testing

### Unit Tests (Planned)
```python
# test_orchestration.py
def test_parallel_execution():
    """Test independent tasks run concurrently."""
    
def test_dependency_blocking():
    """Test dependent task waits for prerequisite."""
    
def test_resource_conflicts():
    """Test conflicting resources are serialized."""
```

### Integration Tests
```bash
# Run example tasks
python task_grasp_frame_gripper.py --solve
python example_collaborative_assembly.py
```

---

## Performance Considerations

### Current Implementation
- **Synchronous execution**: Sequential task execution in orchestrator
- **Polling-based**: 0.1s sleep between scheduling cycles
- **No prediction**: Greedy scheduling based on current state

### Future Optimizations
1. **Async execution**: Use asyncio for true concurrency
2. **Event-driven**: Trigger scheduling on task completion
3. **Predictive scheduling**: Estimate task durations, optimize globally
4. **Resource reservation**: Pre-allocate resources for future tasks

---

## Known Limitations

1. ❌ **No real motion planning integration** (uses mocks)
2. ❌ **No behavior tree framework** (linear execution only)
3. ❌ **No failure recovery** beyond retries
4. ❌ **No learning/adaptation** from execution history
5. ❌ **No real-time constraints** (assumes task durations known)

---

## Contributing

### Adding a New Task

1. Create task configuration class
2. Extend `ManipulationTask` base class
3. Implement required methods
4. Test with visualization
5. Add to task library

### Code Style

- Use type hints
- Document classes and methods
- Follow PEP 8 conventions
- Add unit tests for utilities

---

## Example Output

```
======================================================================
TASK ORCHESTRATION
======================================================================

Total tasks: 6
Max concurrent: 2

----------------------------------------------------------------------
EXECUTION
----------------------------------------------------------------------
[ORCHESTRATOR] Executing: UR10 grasp frame_gripper (ID: t1_ur10_grasp_fg)
[ORCHESTRATOR] Executing: VISPA grasp RS1 (ID: t2_vispa_grasp_rs1)
    [UR10] Planning path to grasp frame_gripper...
    [VISPA] Planning path to grasp RS1...
  ✓ UR10 grasp frame_gripper completed (45.23s)
  ✓ VISPA grasp RS1 completed (42.18s)
[ORCHESTRATOR] Executing: UR10+FG grasp RS2 (ID: t3_ur10_grasp_rs2)
    [UR10+FG] Planning path to grasp RS2...
  ✓ UR10+FG grasp RS2 completed (38.92s)
[ORCHESTRATOR] Executing: UR10 transport RS2 (ID: t4_ur10_transport)
[ORCHESTRATOR] Executing: VISPA transport RS1 (ID: t5_vispa_transport)
  ✓ UR10 transport RS2 completed (28.45s)
  ✓ VISPA transport RS1 completed (31.22s)
[ORCHESTRATOR] Executing: Assemble RS1+RS2 (ID: t6_assembly)
    [UR10+VISPA] Synchronizing positions...
  ✓ Assemble RS1+RS2 completed (65.78s)

----------------------------------------------------------------------
SUMMARY
----------------------------------------------------------------------
✓ All tasks completed successfully!
Total time: 251.84s
```

---

## Resources

- [HPP Documentation](https://humanoid-path-planner.github.io/hpp-doc/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- [Gepetto Viewer](https://github.com/Gepetto/gepetto-viewer)

---

## License

See LICENSE file in repository root.

---

## Contact

For questions or contributions, please open an issue in the repository.
