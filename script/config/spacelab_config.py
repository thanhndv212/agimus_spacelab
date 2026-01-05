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

    class DisplayAllStates(SpacelabTaskDefaults):
        """Catch-all config for exploring feasible goal states.

        Design intent:
        - Include all objects from canonical `ManipulationConfig.OBJECTS`.
        - Keep constraints/transform configs/masks empty placeholders.
        - Provide helpers to enumerate feasible goal states (e.g. grasps)
          from canonical `VALID_PAIRS`.

        This is meant for debug/visualization and for quickly generating a
        feasible target (goal) given a constraint/state naming convention.
        """

        # ------------------------------------------------------------------
        # Scene selection
        # ------------------------------------------------------------------

        # Include all known robot joint groups by default.
        ROBOTS = list(SpacelabTaskDefaults.JOINT_GROUPS.keys())

        # Include all canonical objects by default.
        OBJECTS = list(SpacelabTaskDefaults.OBJECTS_INFO.keys())

        # Some task plumbing expects a TOOL_NAME attribute even when there are
        # no explicit constraint defs. Use the first object as a harmless
        # placeholder.
        TOOL_NAME = OBJECTS[0] if OBJECTS else ""

        # Include all known gripper frames (flatten canonical nested schema).
        GRIPPERS = [
            gripper_frame
            for _group, mapping in SpacelabTaskDefaults.GRIPPERS_INFO.items()
            for gripper_frame in mapping.keys()
        ]

        # Per-object handles and contacts.
        HANDLES_PER_OBJECT = [
            SpacelabTaskDefaults.OBJECTS_INFO[obj]["handles"]
            for obj in OBJECTS
        ]
        CONTACT_SURFACES_PER_OBJECT = [
            SpacelabTaskDefaults.OBJECTS_INFO[obj]["contact_surfaces"]
            for obj in OBJECTS
        ]

        # Environment contact surfaces (canonical).
        ENVIRONMENT_CONTACTS = SpacelabTaskDefaults.ENVIRONMENT_CONTACTS

        # ------------------------------------------------------------------
        # Planning constraints (placeholders)
        # ------------------------------------------------------------------

        # Canonical mapping gripper_frame -> [handle, ...]
        VALID_PAIRS = SpacelabTaskDefaults.VALID_PAIRS_INFO

        # Optional rule set for ConstraintGraphFactory (leave empty for now).
        RULES = None

        # Placeholders for task-specific transforms and masks.
        # The task can fill these as needed.
        TRANSFORMS = {}
        MASKS = {}

        # Generic numeric defaults used by tasks.
        PATH_VALIDATION_STEP = 0.01
        PATH_PROJECTOR_STEP = 0.1
        MAX_RANDOM_ATTEMPTS = 1000

        @classmethod
        def init_poses(cls):
            """Initialize any derived poses.

            Intentionally empty: this catch-all config does not prescribe
            object-relative transforms.
            """

        @classmethod
        def get_constraint_defs(cls):
            """Return constraint definitions for this task.

            Intentionally empty: fill this in for a specific experiment.
            """

            return []

        @classmethod
        def feasible_grasp_goal_states(cls):
            """Enumerate feasible grasp goal state names.

            This follows the factory naming pattern used elsewhere:
            - "<gripper_frame> grasps <handle>"
            """
            goals = []
            for gripper, handles in cls.VALID_PAIRS.items():
                for handle in handles:
                    goals.append(f"{gripper} grasps {handle}")
            return goals

        @classmethod
        def with_grasp_goals(cls, goal_states):
            """Return a derived config with VALID_PAIRS filtered to goals.

            This dramatically reduces factory graph size by only including
            the gripper-handle pairs actually needed for the specified goals.

            Args:
                goal_states: Iterable of goal state strings, e.g.
                    ["spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool"]
                    Strings not matching "<gripper> grasps <handle>" ignored.

            Returns:
                A derived config class with minimal VALID_PAIRS, OBJECTS,
                GRIPPERS, and HANDLES_PER_OBJECT.
            """
            import re

            # Parse goal states to extract (gripper, handle) pairs
            pattern = re.compile(r"^(.+)\s+grasps\s+(.+)$")
            needed_pairs = {}  # gripper -> set of handles
            for goal in goal_states:
                m = pattern.match(goal.strip())
                if m:
                    gripper, handle = m.group(1), m.group(2)
                    needed_pairs.setdefault(gripper, set()).add(handle)

            if not needed_pairs:
                raise ValueError(
                    "No valid grasp goals found. Expected format: "
                    "'<gripper> grasps <handle>'"
                )

            # Determine which objects contain the needed handles
            handle_to_object = {}
            for obj, info in cls.OBJECTS_INFO.items():
                for h in info.get("handles", []):
                    handle_to_object[h] = obj

            needed_objects = set()
            for handles in needed_pairs.values():
                for h in handles:
                    obj = handle_to_object.get(h)
                    if obj:
                        needed_objects.add(obj)

            if not needed_objects:
                raise ValueError(
                    f"Could not find objects for handles: {needed_pairs}"
                )

            # Build minimal config
            objects = [o for o in cls.OBJECTS if o in needed_objects]
            grippers = [g for g in cls.GRIPPERS if g in needed_pairs]

            handles_per_object = [
                cls.OBJECTS_INFO[obj]["handles"] for obj in objects
            ]
            contacts_per_object = [
                cls.OBJECTS_INFO[obj]["contact_surfaces"] for obj in objects
            ]

            # Build filtered VALID_PAIRS (only requested grasps)
            valid_pairs = {}
            for g in grippers:
                valid_pairs[g] = list(needed_pairs.get(g, []))

            # Create derived class
            Filtered = type("DisplayAllStatesFiltered", (cls,), {})
            Filtered.OBJECTS = objects
            Filtered.GRIPPERS = grippers
            Filtered.HANDLES_PER_OBJECT = handles_per_object
            Filtered.CONTACT_SURFACES_PER_OBJECT = contacts_per_object
            Filtered.VALID_PAIRS = valid_pairs
            Filtered.TOOL_NAME = objects[0] if objects else ""
            return Filtered

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
