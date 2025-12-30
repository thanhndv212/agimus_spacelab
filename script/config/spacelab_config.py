"""
Configuration classes for manipulation tasks.
"""

from agimus_spacelab.config import ManipulationConfig
from agimus_spacelab.utils import xyzrpy_to_xyzquat


class SpacelabTaskDefaults(ManipulationConfig):
    """Task config base class.

    Inherits SpaceLab-wide defaults from `ManipulationConfig` and provides
    stable aliases to avoid name collisions with task-local fields.
    """

    GRIPPERS_INFO = ManipulationConfig.GRIPPERS
    OBJECTS_INFO = ManipulationConfig.OBJECTS
    VALID_PAIRS_INFO = ManipulationConfig.VALID_PAIRS


class TaskConfigurations:
    """Task-specific configurations for common manipulation tasks."""

    # Grasp Frame Gripper Task
    class GraspFrameGripper(SpacelabTaskDefaults):
        """Configuration for UR10 grasping frame_gripper from dispenser."""

        ROBOTS = ["UR10", "VISPA_BASE", "VISPA_ARM"]
        OBJECTS = ["frame_gripper"]

        _GRIPPER_FRAME_TO_JOINT = SpacelabTaskDefaults.GRIPPERS_INFO[
            "ur10"
        ]
        GRIPPER_NAME, GRIPPER_JOINT = next(
            iter(_GRIPPER_FRAME_TO_JOINT.items())
        )
        GRIPPERS = [GRIPPER_NAME]

        TOOL_NAME = SpacelabTaskDefaults.OBJECTS_INFO["frame_gripper"][
            "handles"
        ][0]
        TOOL_JOINT = SpacelabTaskDefaults.OBJECTS_INFO["frame_gripper"][
            "root_joint"
        ]

        # Handles per object
        HANDLES_PER_OBJECT = [
            SpacelabTaskDefaults.OBJECTS_INFO[OBJECTS[0]]["handles"],
        ]

        # Contact surfaces per object
        CONTACT_SURFACES_PER_OBJECT = [
            SpacelabTaskDefaults.OBJECTS_INFO[OBJECTS[0]]["contact_surfaces"],
        ]

        # Environment contact surfaces
        ENVIRONMENT_CONTACTS = SpacelabTaskDefaults.ENVIRONMENT_CONTACTS

        # Rules for valid grasps
        RULES = None

        # Valid gripper-object pairs
        VALID_PAIRS = {GRIPPER_NAME: [TOOL_NAME]}

        # Transform configurations (in xyzquat format)
        TOOL_IN_GRIPPER = [0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 1.0]
        GRIPPER_ABOVE_TOOL = [0.0, 0.0, 0.03,
                              0.0, 0.0, 0.0, 1.0]

        TOOL_ON_DISPENSER = None

        TOOL_IN_AIR = None

        # Constraint masks (6 DOF: [x, y, z, rx, ry, rz])
        GRASP_MASK = [True, True, True, True, True, True]  # All DOF fixed
        PLACEMENT_MASK = [
            True,
            False,
            True,
            True,
            True,
            True,
        ]  # X, Z + rotations fixed
        PLACEMENT_COMPLEMENT_MASK = [
            False,
            True,
            False,
            False,
            False,
            False,
        ]  # Y free

        # ====================================================================
        # Graph Definition (Declarative)
        # ====================================================================

        GRASP_FG_GRAPH = {
            "name": "grasp_fg_graph",

            # States with their constraints
            "states": {
                "placement": {"constraints": ["placement"]},
                "gripper-above-tool": {
                    "constraints": ["placement", "gripper_tool_aligned"]
                },
                "grasp-placement": {"constraints": ["grasp", "placement"]},
                "tool-in-air": {"constraints": ["grasp", "tool_in_air"]},
                "grasp": {"constraints": ["grasp"]},
            },

            # Edges: from -> to via containing state
            "edges": {
                # Self-loops
                "transit": {
                    "from": "placement", "to": "placement", "in": "placement"
                },
                "transfer": {"from": "grasp", "to": "grasp", "in": "grasp"},

                # From placement
                "approach-tool": {
                    "from": "placement",
                    "to": "gripper-above-tool",
                    "in": "placement",
                },

                # From gripper-above-tool
                "move-gripper-away": {
                    "from": "gripper-above-tool",
                    "to": "placement",
                    "in": "placement",
                },
                "grasp-tool": {
                    "from": "gripper-above-tool",
                    "to": "grasp-placement",
                    "in": "placement",
                },

                # From grasp-placement
                "release-tool": {
                    "from": "grasp-placement",
                    "to": "gripper-above-tool",
                    "in": "placement",
                },
                "lift-tool": {
                    "from": "grasp-placement",
                    "to": "tool-in-air",
                    "in": "grasp",
                },

                # From tool-in-air
                "lower-tool": {
                    "from": "tool-in-air",
                    "to": "grasp-placement",
                    "in": "grasp",
                },
                "move-tool-away": {
                    "from": "tool-in-air",
                    "to": "grasp",
                    "in": "grasp",
                },

                # From grasp
                "approach-dispenser": {
                    "from": "grasp",
                    "to": "tool-in-air",
                    "in": "grasp",
                },
            },

            # Edge path constraints (grouped by constraint)
            # Edges not listed are free motion (no path constraints)
            "edge_constraints": {
                "placement/complement": [
                    "transit",
                    "approach-tool",
                    "move-gripper-away",
                    "grasp-tool",
                    "release-tool",
                ],
                "tool_in_air/complement": ["lift-tool", "lower-tool"],
            },

            # Free motion edges (no path constraints)
            "free_motion_edges": [
                "transfer",
                "move-tool-away",
                "approach-dispenser",
            ],

            # Constant RHS settings (CORBA only)
            "constant_rhs": {
                "grasp": True,
                "placement": True,
                "gripper_tool_aligned": True,
                "tool_in_air": True,
                "placement/complement": False,
                "tool_in_air/complement": False,
            },
        }

        # Collision parameters
        TOOL_CONTACT_JOINT = "frame_gripper/root_joint"
        DISPENSER_CONTACT_JOINT = "universe"
        CONTACT_MARGIN = -0.02  # Allow 2cm penetration

        # Graph nodes
        GRAPH_NODES = [
            "grasp",
            "tool-in-air",
            "grasp-placement",
            "gripper-above-tool",
            "placement",
        ]
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

        @classmethod
        def init_poses(cls):
            """Initialize tool poses from configuration."""
            tool_pose_xyzrpy = SpacelabTaskDefaults.OBJECTS_INFO[
                "frame_gripper"
            ]["default_pose_xyzrpy"]
            tool_pose_quat = xyzrpy_to_xyzquat(tool_pose_xyzrpy)

            cls.TOOL_ON_DISPENSER = tool_pose_quat.tolist()

            # Lifted position
            cls.TOOL_IN_AIR = tool_pose_quat.copy()
            cls.TOOL_IN_AIR[1] -= cls.LIFT_HEIGHT  # Use inherited constant
            cls.TOOL_IN_AIR = cls.TOOL_IN_AIR.tolist()

        @classmethod
        def get_constraint_defs(cls):
            """Return constraint definitions for this task.
            
            Each tuple: (type, name, args_dict)
            - type: "grasp", "placement", or "complement"
            - name: constraint name
            - args: dict with keys depending on type
              - grasp: gripper, obj, transform, mask
              - placement/complement: obj, transform, mask
            """
            return [
                ("grasp", "grasp", {
                    "gripper": cls.GRIPPER_NAME,
                    "obj": cls.TOOL_NAME,
                    "transform": cls.TOOL_IN_GRIPPER,
                    "mask": cls.GRASP_MASK,
                }),
                ("placement", "placement", {
                    "obj": cls.TOOL_NAME,
                    "transform": cls.TOOL_ON_DISPENSER,
                    "mask": cls.PLACEMENT_MASK,
                }),
                ("complement", "placement", {
                    "obj": cls.TOOL_NAME,
                    "transform": cls.TOOL_ON_DISPENSER,
                    "mask": cls.PLACEMENT_COMPLEMENT_MASK,
                }),
                ("grasp", "gripper_tool_aligned", {
                    "gripper": cls.GRIPPER_NAME,
                    "obj": cls.TOOL_NAME,
                    "transform": cls.GRIPPER_ABOVE_TOOL,
                    "mask": cls.GRASP_MASK,
                }),
                ("placement", "tool_in_air", {
                    "obj": cls.TOOL_NAME,
                    "transform": cls.TOOL_IN_AIR,
                    "mask": cls.PLACEMENT_MASK,
                }),
                ("complement", "tool_in_air", {
                    "obj": cls.TOOL_NAME,
                    "transform": cls.TOOL_IN_AIR,
                    "mask": cls.PLACEMENT_COMPLEMENT_MASK,
                }),
            ]


__all__ = [
    "TaskConfigurations",
]
