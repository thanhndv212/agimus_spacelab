#!/usr/bin/env python3
"""
Task: UR10 grasps frame_gripper from dispenser.

This task demonstrates a complete pick-and-place sequence:
1. Tool rests on dispenser (placement)
2. Gripper approaches tool (gripper-above-tool)
3. Gripper grasps tool on dispenser (grasp-placement)
4. Tool is lifted from dispenser (tool-in-air)
5. Free motion with grasped tool (grasp)

Task features:
- Surface contact modeling (tool on dispenser)
- Complement constraints for sliding motion
- Security margins for expected contacts
- Multi-waypoint configuration generation
"""

import sys
from pathlib import Path
from typing import List, Dict

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import GraphBuilder, ConstraintBuilder
from agimus_spacelab.visualization import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_all_handles,
    visualize_all_grippers,
)

# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from spacelab_config import TaskConfigurations


# ============================================================================
# Task Configuration
# ============================================================================
GraspFrameGripperConfig = TaskConfigurations.GraspFrameGripper
# Initialize poses
GraspFrameGripperConfig.init_poses()


# ============================================================================
# Task Implementation
# ============================================================================

class GraspFrameGripperTask(ManipulationTask):
    """UR10 grasps frame_gripper from dispenser."""
    
    def __init__(self, use_factory: bool = False, backend: str = "corba"):
        """
        Initialize task.
        
        Args:
            use_factory: If True, use ConstraintGraphFactory for automatic
                        graph generation. If False, build graph manually.
            backend: "corba" or "pyhpp" - which backend to use
        """
        super().__init__(
            task_name="Spacelab Manipulation: UR10 Grasps Frame Gripper",
            backend=backend
        )
        self.config = GraspFrameGripperConfig
        self.use_factory = use_factory
        
    def get_robot_names(self) -> List[str]:
        return self.config.ROBOT_NAMES
    
    def get_composite_names(self) -> List[str]:
        return self.config.ROBOT_NAMES
    
    def get_object_names(self) -> List[str]:
        return self.config.OBJECTS
    
    def get_environment_names(self) -> List[str]:
        return self.config.ENVIRONMENT_NAMES
        
    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return self.config.ROBOTS

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return self.config.OBJECTS
        
    def setup_collision_management(self) -> None:
        """Disable collision between tool and dispenser surface."""
        self.scene_builder.disable_collision_pair(
            "ground_demo/link_TD_0",  # Dispenser surface
            self.config.TOOL_CONTACT_JOINT,
            remove_collision=True,
            remove_distance=False
        )
    
    def create_constraints(self) -> None:
        """Create all transformation constraints."""
        # Factory mode creates constraints automatically
        if self.use_factory:
            print("    (Factory mode: constraints created automatically)")
            return
            
        cb = ConstraintBuilder
        cfg = self.config
        
        # 1. Grasp: gripper holds tool
        cb.create_grasp_constraint(
            self.ps, "grasp",
            cfg.GRIPPER_NAME, cfg.TOOL_NAME,
            cfg.TOOL_IN_GRIPPER, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 2. Placement: tool on dispenser (Z and rotations fixed)
        cb.create_placement_constraint(
            self.ps, "placement",
            cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 3. Placement complement: X, Y free (sliding on surface)
        cb.create_complement_constraint(
            self.ps, "placement",
            cfg.TOOL_NAME, cfg.TOOL_ON_DISPENSER,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 4. Gripper-tool aligned: gripper above tool
        cb.create_grasp_constraint(
            self.ps, "gripper_tool_aligned",
            cfg.GRIPPER_NAME, cfg.TOOL_NAME,
            cfg.GRIPPER_ABOVE_TOOL, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 5. Tool in air: lifted position
        cb.create_placement_constraint(
            self.ps, "tool_in_air",
            cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 6. Tool in air complement
        cb.create_complement_constraint(
            self.ps, "tool_in_air",
            cfg.TOOL_NAME, cfg.TOOL_IN_AIR,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
    def create_graph(self):
        """Create and configure constraint graph."""
        if self.use_factory:
        # """Create graph using ConstraintGraphFactory (automatic)."""
            cfg = self.config
            
            # Initialize GraphBuilder
            self.graph_builder = GraphBuilder(
                self.planner, self.robot, self.ps, backend=self.backend
            )
            
            # Create factory graph
            graph = self.graph_builder.create_factory_graph(
                grippers=cfg.GRIPPERS,
                objects=cfg.OBJECTS,
                handles_per_object=cfg.HANDLES_PER_OBJECT,
                environment_contacts=["ground_demo/tools_dispenser_surface"],
                valid_pairs=cfg.VALID_PAIRS
            )
            if self.backend == "pyhpp":
                problem = self.planner.get_problem()
                problem.constraintGraph(graph)
                self.pyhpp_states = self.graph_builder.get_states()
                self.pyhpp_edges = self.graph_builder.get_edges()
                self.pyhpp_edge_topology = self.graph_builder.get_edge_topology()
                
            return graph

        else:
            if self.backend == "corba":
                return self._create_corba_graph_manual()
            if self.backend == "pyhpp":
                return self._create_pyhpp_graph_manual()

            
    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations."""
        # Factory mode creates different node/edge names
        # For now, just return initial config for factory mode        
        cg = self.config_gen
        cfg = self.config

                    
        # Update max attempts
        cg.max_attempts = cfg.MAX_RANDOM_ATTEMPTS
        
        if self.use_factory:
            print("    Factory mode: Using initial configuration only")
            q_goal_modified = cg.modify_object_pose(
                q_init,
                object_index=0,  # frame_gripper is first object
                translation_delta=[0.2, 0.0, 0.0]
            )
            return {"q_init": q_init,
                    "q_goal": q_goal_modified}
                
        # 1. Project initial onto placement
        print("    1. Projecting onto 'placement' state...")
        cg.project_on_node("placement", q_init, "q_init")
        
        # 2. Generate approach config
        print("    2. Generating 'approach-tool' config...")
        res, _ = cg.generate_via_edge(
            "approach-tool", cg.configs["q_init"], "q_approach"
        )
        if not res:
            return cg.configs
            
        # 3. Project onto gripper-above-tool
        print("    3. Projecting onto 'gripper-above-tool' state...")
        cg.project_on_node(
            "gripper-above-tool", cg.configs["q_approach"], "q_above"
        )
        
        # 4. Generate grasp config
        print("    4. Generating 'grasp-tool' config...")
        res, _ = cg.generate_via_edge(
            "grasp-tool", cg.configs["q_above"], "q_grasp"
        )
        if not res:
            return cg.configs
            
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        cg.project_on_node(
            "grasp-placement", cg.configs["q_grasp"], "q_grasp_place"
        )
        
        # 6. Generate lift config
        print("    6. Generating 'lift-tool' config...")
        res, _ = cg.generate_via_edge(
            "lift-tool", cg.configs["q_grasp_place"], "q_lifted"
        )
        if not res:
            return cg.configs
            
        # 7. Project onto tool-in-air
        print("    7. Projecting onto 'tool-in-air' state...")
        cg.project_on_node("tool-in-air", cg.configs["q_lifted"], "q_tool_air")
        
        # 8. Generate goal (move tool 20cm in x)
        print("    8. Generating goal configuration...")
        q_goal_modified = cg.modify_object_pose(
            cg.configs["q_init"],
            object_index=0,  # frame_gripper is first object
            translation_delta=[0.2, 0.0, 0.0]
        )
        cg.project_on_node("placement", q_goal_modified, "q_goal")
        return cg.configs


# ============================================================================
# Main Execution
# ============================================================================

def main(
    visualize: bool = True,
    solve: bool = False,
    show_joints: bool = False,
    use_factory: bool = False,
    backend: str = "corba"
):
    """
    Run the grasp frame_gripper task.
    
    Args:
        visualize: Show configurations in viewer
        solve: Attempt to solve planning problem
        show_joints: Print joint information
        use_factory: Use ConstraintGraphFactory (automatic) instead of
            manual graph
        backend: "corba" or "pyhpp" - which backend to use
    """ 
    print(f"Using backend: {backend.upper()}")
    
    # Create and setup task
    task = GraspFrameGripperTask(use_factory=use_factory, backend=backend)
    task.setup(
        validation_step=GraspFrameGripperConfig.PATH_VALIDATION_STEP,
        projector_step=GraspFrameGripperConfig.PATH_PROJECTOR_STEP
    )
    
    # Print joint info if requested
    if show_joints:
        print_joint_info(task.robot)
        
    # Run task
    result = task.run(visualize=visualize, solve=solve)
    
    # Visualize handles and grippers if viewer available
    if visualize and result.get('viewer'):
        print("\n" + "=" * 70)
        print("Visualizing Handles and Grippers")
        print("=" * 70)
        viewer = result['viewer']
        
        # Collect all handle names
        handle_names = task.graph_builder.factory.handles
        
        # Visualize handles with approach arrows
        visualize_all_handles(
            viewer, handle_names,
            show_approach=True,
            frame_color=[0, 0.8, 0, 1],   # Green
            arrow_color=[0, 1, 1, 1],      # Cyan
            axis_length=0.05,
            arrow_length=0.1
        )
        
        # Visualize grippers with approach arrows
        gripper_names = GraspFrameGripperConfig.GRIPPERS
        visualize_all_grippers(
            viewer, gripper_names,
            show_approach=True,
            frame_color=[1, 0, 0, 1],      # Red
            arrow_color=[1, 0.5, 0, 1],    # Orange
            axis_length=0.05,
            arrow_length=0.1
        )
        viewer.client.gui.refresh()
    
    # Print summary
    print("\n" + "=" * 70)
    print("Results Summary")
    print("=" * 70)

    print("\nTo visualize specific configs:")
    print("  task.planner.visualize(result['configs']['q_init'])")

    if solve:
        print("\nTo replay path:")
        print("  task.planner.play_path(0)")


    # Visualize constraint graph
    print("\n📊 Generating constraint graph visualization...")
    # Pass state/edge dicts for PyHPP backend
    states_dict = getattr(task, 'pyhpp_states', None)
    edges_dict = getattr(task, 'pyhpp_edges', None)
    edge_topology = getattr(task, 'pyhpp_edge_topology', None)
    viz_path = visualize_constraint_graph(
        task.graph_builder,
        output_path="constraint_graph",
        states_dict=states_dict,
        edges_dict=edges_dict,
        edge_topology=edge_topology
    )
    
    return task, result


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="UR10 grasps frame_gripper from dispenser"
    )
    parser.add_argument(
        "--no-viz", action="store_true", help="Disable visualization"
    )
    parser.add_argument(
        "--solve", action="store_true", help="Solve planning problem"
    )
    parser.add_argument(
        "--show-joints", action="store_true", help="Print joint info"
    )
    parser.add_argument(
        "--factory", action="store_true",
        help="Use ConstraintGraphFactory (automatic graph generation)"
    )
    parser.add_argument(
        "--backend", type=str, default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use: corba (default) or pyhpp"
    )
    
    args = parser.parse_args()
    
    task, result = main(
        visualize=not args.no_viz,
        solve=args.solve,
        show_joints=args.show_joints,
        use_factory=args.factory,
        backend=args.backend
    )