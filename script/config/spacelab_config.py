"""
Configuration classes for manipulation tasks.
"""
from agimus_spacelab.config import InitialConfigurations
from agimus_spacelab.utils import xyzrpy_to_xyzquat

class TaskConfigurations:
    """Task-specific configurations for common manipulation tasks."""
    
    # Grasp Frame Gripper Task
    class GraspFrameGripper:
        """Configuration for UR10 grasping frame_gripper from dispenser."""
        
        # Robot name
        ROBOT_NAMES = ["spacelab"]
        ENVIRONMENT_NAMES = ["ground_demo"]
        OBJECTS = ["frame_gripper",
                #    "screw_driver",
                #    "RS1",
                #    "RS2",
                #    "RS3",
                #    "RS4",
                #    "RS5",
                #    "RS6"
                   ]
        # Joint groups
        ROBOTS = ["UR10", "VISPA_BASE", "VISPA_ARM"]

        # Transform configurations (in xyzquat format)
        TOOL_IN_GRIPPER = [0.0, 0.0, 0.1,
                           0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
        GRIPPER_ABOVE_TOOL = [0.0, 0.0, 0.2,
                              0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
        
        # Constraint masks (6 DOF: [x, y, z, rx, ry, rz])
        GRASP_MASK = [True, True, True, True, True, True]  # All DOF fixed
        PLACEMENT_MASK = [False, False, True, True, True, True]  # Z + rotations fixed
        PLACEMENT_COMPLEMENT_MASK = [True, True, False, False, False, False]  # X, Y free
        
        # Graph structure
        GRAPH_NODES = [
            "grasp",
            "tool-in-air",
            "grasp-placement",
            "gripper-above-tool",
            "placement",
        ]
        
        # Collision parameters
        TOOL_CONTACT_JOINT = "frame_gripper/root_joint"
        DISPENSER_CONTACT_JOINT = "universe"
        CONTACT_MARGIN = -0.02  # Allow 2cm penetration
        
        # Edges requiring surface contact security margins
        PLACEMENT_EDGES = [
            "transit", "approach-tool", "move-gripper-away",
            "grasp-tool", "release-tool", "lift-tool", "lower-tool",
        ]
        
        # Planning parameters
        PATH_VALIDATION_STEP = 0.01
        PATH_PROJECTOR_STEP = 0.1
        MAX_RANDOM_ATTEMPTS = 1000
        LIFT_HEIGHT = 0.15  # Lift tool 15cm from dispenser


        # Gripper and object names
        GRIPPER_NAME = "spacelab/ur10_joint_6_7"
        TOOL_NAME = "frame_gripper/root_joint"
        
        # Grippers (for factory mode)
        GRIPPERS = ["spacelab/g_ur10_tool"]
        
        # Handles per object (for factory mode)
        HANDLES_PER_OBJECT = [
            ["frame_gripper/h_FG_tool"],  # frame_gripper
            # ["screw_driver/h_SD_tool"],   # screw_driver
            # ["RS1/h_RS1_FG"],              # RS1
            # ["RS2/h_RS2_FG"],              # RS2
            # ["RS3/h_RS3_FG"],              # RS3
            # ["RS4/h_RS4_FG"],              # RS4
            # ["RS5/h_RS5_FG"],              # RS5
            # ["RS6/h_RS6_FG"],              # RS6
        ]
        VALID_PAIRS = {
        "spacelab/g_ur10_tool": [
            "frame_gripper/h_FG_tool",
            # "screw_driver/h_SD_tool",
        ],}
        # Tool poses (computed from initial config)
        TOOL_ON_DISPENSER = None
        TOOL_IN_AIR = None
        
        @classmethod
        def init_poses(cls):
            """Initialize tool poses from configuration."""
            tool_pose_xyzrpy = InitialConfigurations.FRAME_GRIPPER
            tool_pose_quat = xyzrpy_to_xyzquat(tool_pose_xyzrpy)
            
            cls.TOOL_ON_DISPENSER = tool_pose_quat.tolist()
            
            # Lifted position
            cls.TOOL_IN_AIR = tool_pose_quat.copy()
            cls.TOOL_IN_AIR[2] += cls.LIFT_HEIGHT  # Use inherited constant
            cls.TOOL_IN_AIR = cls.TOOL_IN_AIR.tolist()


__all__ = [
    "TaskConfigurations",
]
