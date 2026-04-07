"""
Standalone SpaceLab configuration — no package imports for robot/object data.

This file is intentionally self-contained so it can live in script/config/
without being part of the agimus_spacelab package.  Only framework utilities
(xyzrpy_to_xyzquat) are imported from the package.
"""

from typing import Dict, List
import numpy as np

from agimus_spacelab.utils import xyzrpy_to_xyzquat

# ---------------------------------------------------------------------------
# File paths
# ---------------------------------------------------------------------------

DEFAULT_PATHS = {
    "robot": {
        "spacelab": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf",
        }
    },
    "environment": {
        "ground_demo": "package://spacelab_mock_hardware/description/urdf/ground_demo.urdf"
    },
    "objects": {
        "RS1": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS1.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS1.srdf",
        },
        "RS2": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS2.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS2.srdf",
        },
        "RS3": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS3.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS3.srdf",
        },
        "RS4": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS4.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS4.srdf",
        },
        "RS5": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS5.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS5.srdf",
        },
        "RS6": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/RS6.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/RS6.srdf",
        },
        "screw_driver": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/screw_driver.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/screw_driver.srdf",
        },
        "frame_gripper": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/frame_gripper.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/frame_gripper.srdf",
        },
        "cleat_gripper": {
            "urdf": "package://spacelab_mock_hardware/description/urdf/cleat_gripper.urdf",
            "srdf": "package://spacelab_mock_hardware/description/srdf/cleat_gripper.srdf",
        },
    },
}


# ---------------------------------------------------------------------------
# Joint names
# ---------------------------------------------------------------------------


class RobotJoints:
    """Joint names for the composite SpaceLab robot."""

    UR10 = [
        "spacelab/ur10_joint_1_2",
        "spacelab/ur10_joint_2_3",
        "spacelab/ur10_joint_3_4",
        "spacelab/ur10_joint_4_5",
        "spacelab/ur10_joint_5_6",
        "spacelab/ur10_joint_6_7",
    ]

    VISPA_BASE = [
        "spacelab/vispa2_joint_2_3",
        "spacelab/vispa2_joint_3_eeWorkbench",
    ]

    VISPA_ARM = [
        "spacelab/vispa_joint_1_2",
        "spacelab/vispa_joint_2_3",
        "spacelab/vispa_joint_3_4",
        "spacelab/vispa_joint_4_5",
        "spacelab/vispa_joint_5_6",
        "spacelab/vispa_joint_6_7",
    ]

    RS1_ROOT = "RS1/root_joint"
    RS2_ROOT = "RS2/root_joint"
    RS3_ROOT = "RS3/root_joint"
    RS4_ROOT = "RS4/root_joint"
    RS5_ROOT = "RS5/root_joint"
    RS6_ROOT = "RS6/root_joint"
    SCREW_DRIVER_ROOT = "screw_driver/root_joint"
    FRAME_GRIPPER_ROOT = "frame_gripper/root_joint"
    CLEAT_GRIPPER_ROOT = "cleat_gripper/root_joint"

    @classmethod
    def all_robot_joints(cls) -> List[str]:
        return cls.UR10 + cls.VISPA_BASE + cls.VISPA_ARM

    @classmethod
    def all_object_root_joints(cls) -> List[str]:
        return [
            cls.RS1_ROOT, cls.RS2_ROOT, cls.RS3_ROOT,
            cls.RS4_ROOT, cls.RS5_ROOT, cls.RS6_ROOT,
            cls.SCREW_DRIVER_ROOT, cls.FRAME_GRIPPER_ROOT, cls.CLEAT_GRIPPER_ROOT,
        ]


# ---------------------------------------------------------------------------
# Initial configurations
# ---------------------------------------------------------------------------


class InitialConfigurations:
    """Initial joint configurations for robots and objects."""

    UR10 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    VISPA_BASE = [0.0, 0.0]
    VISPA_ARM = [0.0, 3.14, 0.0, 0.0, 0.0, 0.0]

    RS1 = [-0.46567999999999976, 2.0219499999999999, -0.34200800000000015,
           1.5707938223931903, -3.1415918612707121, 2.0943948257717535]
    RS2 = [0.38432000000000011, 2.0219489999999993, -0.34200799999999987,
           1.5707938223931903, -3.1415918612707121, 2.0943948257717535]
    RS3 = [0.38432000000000011, 2.0219489999999993, -0.63200800000000001,
           1.5707938223931903, -3.1415918612707121, 2.0943948257717535]
    RS4 = [0.38432000027622287, 2.0134709992570583, -0.99423100120426089,
           1.4828606077991944, -2.9906356348103711, 2.1010496873808218]
    RS5 = [-0.46567999967380558, 2.0133499991226964, -0.99421000142184635,
           1.4828606077991944, -2.9906356348103711, 2.1010496873808218]
    RS6 = [-0.46567999999999987, 2.0219489999999993, -0.63200800000000001,
           1.5707938223931903, -3.1415918612707121, 2.0943948257717535]

    SCREW_DRIVER = [0.046500000888612281, 1.3322769978599074, -1.2103000009948957,
                    3.1415926529240905, -3.2988203261954171e-07, 1.5707963267957175]
    FRAME_GRIPPER = [-0.13349999871947557, 1.3322770029736009, -1.2132999986768855,
                     -5.098912860707481e-09, -3.1415923241839976, 1.5707963267958001]
    CLEAT_GRIPPER = [0.04650005078371261, 1.510277021366528, -1.212900037140666,
                     4.1022589876578654e-07, 3.1415906427953018, 0.78539700669219625]


# ---------------------------------------------------------------------------
# Joint bounds
# ---------------------------------------------------------------------------


class JointBounds:
    """Joint bounds for robots and free-flying objects."""

    UR10_LIMITS = [(-2 * np.pi, 2 * np.pi)] * 6
    VISPA_BASE_LIMITS = [(-5.0, 5.0), (-5.0, 5.0)]
    VISPA_ARM_LIMITS = [(-np.pi, np.pi)] * 6

    TRANSLATION_BOUNDS = [(-2.0, 2.0), (-3.0, 3.0), (-2.0, 2.0)]
    QUATERNION_BOUNDS = [
        (-1.0001, 1.0001), (-1.0001, 1.0001),
        (-1.0001, 1.0001), (-1.0001, 1.0001),
    ]

    @classmethod
    def freeflyer_bounds(cls) -> List[float]:
        bounds = []
        for t in cls.TRANSLATION_BOUNDS:
            bounds.extend(t)
        for q in cls.QUATERNION_BOUNDS:
            bounds.extend(q)
        return bounds

    @classmethod
    def all_robot_bounds(cls) -> Dict[str, List[float]]:
        bounds = {}
        for i, joint in enumerate(RobotJoints.UR10):
            bounds[joint] = list(cls.UR10_LIMITS[i])
        for i, joint in enumerate(RobotJoints.VISPA_BASE):
            bounds[joint] = list(cls.VISPA_BASE_LIMITS[i])
        for i, joint in enumerate(RobotJoints.VISPA_ARM):
            bounds[joint] = list(cls.VISPA_ARM_LIMITS[i])
        return bounds


# ---------------------------------------------------------------------------
# ManipulationConfig — canonical scene/gripper/object metadata
# ---------------------------------------------------------------------------


class ManipulationConfig:
    """Canonical SpaceLab manipulation configuration."""

    MODEL_PATHS = DEFAULT_PATHS
    ROBOT_NAMES = ["spacelab"]
    ENVIRONMENT_NAMES = ["ground_demo"]

    JOINT_GROUPS = {
        "UR10": RobotJoints.UR10,
        "VISPA_BASE": RobotJoints.VISPA_BASE,
        "VISPA_ARM": RobotJoints.VISPA_ARM,
    }

    ENVIRONMENT_CONTACTS = {
        "ground_demo": ["ground_demo/ground_surface"],
    }

    GRIPPERS = {
        "ur10": {"spacelab/g_ur10_tool": "spacelab/ur10_joint_6_7"},
        "vispa": {"spacelab/g_vispa_tool": "spacelab/vispa_joint_6_7"},
        "vispa2": {
            "spacelab/g_vispa2_wb1": "spacelab/vispa2_joint_3_eeWorkbench",
            "spacelab/g_vispa2_wb2": "spacelab/vispa2_joint_3_eeWorkbench",
            "spacelab/g_vispa2_wb3": "spacelab/vispa2_joint_3_eeWorkbench",
            "spacelab/g_vispa2_wb4": "spacelab/vispa2_joint_3_eeWorkbench",
            "spacelab/g_vispa2_wb5": "spacelab/vispa2_joint_3_eeWorkbench",
            "spacelab/g_vispa2_wb6": "spacelab/vispa2_joint_3_eeWorkbench",
        },
        "frame_gripper": {"frame_gripper/g_FG_part": "frame_gripper/root_joint"},
        "screw_driver": {"screw_driver/g_SD_part": "screw_driver/root_joint"},
    }

    OBJECTS = {
        "frame_gripper": {
            "handles": ["frame_gripper/h_FG_tool"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.FRAME_GRIPPER_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.FRAME_GRIPPER,
        },
        "screw_driver": {
            "handles": ["screw_driver/h_SD_tool"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.SCREW_DRIVER_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.SCREW_DRIVER,
        },
        "RS1": {
            "handles": ["RS1/h_RS1_FG", "RS1/h_RS1_WB",
                        "RS1/h_RS1_CON0", "RS1/h_RS1_CON1",
                        "RS1/h_RS1_CON2", "RS1/h_RS1_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS1_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS1,
        },
        "RS2": {
            "handles": ["RS2/h_RS2_FG", "RS2/h_RS2_WB",
                        "RS2/h_RS2_CON0", "RS2/h_RS2_CON1",
                        "RS2/h_RS2_CON2", "RS2/h_RS2_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS2_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS2,
        },
        "RS3": {
            "handles": ["RS3/h_RS3_FG", "RS3/h_RS3_WB",
                        "RS3/h_RS3_CON0", "RS3/h_RS3_CON1",
                        "RS3/h_RS3_CON2", "RS3/h_RS3_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS3_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS3,
        },
        "RS4": {
            "handles": ["RS4/h_RS4_FG", "RS4/h_RS4_WB",
                        "RS4/h_RS4_CON0", "RS4/h_RS4_CON1",
                        "RS4/h_RS4_CON2", "RS4/h_RS4_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS4_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS4,
        },
        "RS5": {
            "handles": ["RS5/h_RS5_FG", "RS5/h_RS5_WB",
                        "RS5/h_RS5_CON0", "RS5/h_RS5_CON1",
                        "RS5/h_RS5_CON2", "RS5/h_RS5_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS5_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS5,
        },
        "RS6": {
            "handles": ["RS6/h_RS6_FG", "RS6/h_RS6_WB",
                        "RS6/h_RS6_CON0", "RS6/h_RS6_CON1",
                        "RS6/h_RS6_CON2", "RS6/h_RS6_CON3"],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS6_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS6,
        },
    }

    VALID_PAIRS = {
        "spacelab/g_ur10_tool": ["frame_gripper/h_FG_tool"],
        "spacelab/g_vispa_tool": ["screw_driver/h_SD_tool"],
        "spacelab/g_vispa2_wb1": ["RS1/h_RS1_WB"],
        "spacelab/g_vispa2_wb2": ["RS2/h_RS2_WB"],
        "spacelab/g_vispa2_wb3": ["RS3/h_RS3_WB"],
        "spacelab/g_vispa2_wb4": ["RS4/h_RS4_WB"],
        "spacelab/g_vispa2_wb5": ["RS5/h_RS5_WB"],
        "spacelab/g_vispa2_wb6": ["RS6/h_RS6_WB"],
        "frame_gripper/g_FG_part": [
            "RS1/h_RS1_FG", "RS2/h_RS2_FG", "RS3/h_RS3_FG",
            "RS4/h_RS4_FG", "RS5/h_RS5_FG", "RS6/h_RS6_FG",
        ],
        "screw_driver/g_SD_part": [
            "RS1/h_RS1_CON0", "RS1/h_RS1_CON1", "RS1/h_RS1_CON2", "RS1/h_RS1_CON3",
            "RS2/h_RS2_CON0", "RS2/h_RS2_CON1", "RS2/h_RS2_CON2", "RS2/h_RS2_CON3",
            "RS3/h_RS3_CON0", "RS3/h_RS3_CON1", "RS3/h_RS3_CON2", "RS3/h_RS3_CON3",
            "RS4/h_RS4_CON0", "RS4/h_RS4_CON1", "RS4/h_RS4_CON2", "RS4/h_RS4_CON3",
            "RS5/h_RS5_CON0", "RS5/h_RS5_CON1", "RS5/h_RS5_CON2", "RS5/h_RS5_CON3",
            "RS6/h_RS6_CON0", "RS6/h_RS6_CON1", "RS6/h_RS6_CON2", "RS6/h_RS6_CON3",
        ],
    }


# ---------------------------------------------------------------------------
# Task config base and task configurations
# ---------------------------------------------------------------------------


class SpacelabTaskDefaults(ManipulationConfig):
    """Task config base class.

    Inherits SpaceLab-wide defaults from `ManipulationConfig` and provides
    stable aliases to avoid name collisions with task-local fields.
    """

    GRIPPERS_INFO = ManipulationConfig.GRIPPERS
    OBJECTS_INFO = ManipulationConfig.OBJECTS
    VALID_PAIRS_INFO = ManipulationConfig.VALID_PAIRS

    # Optional optimizer configuration for TransitionPlanner solving.
    # When unset (None), TransitionPlanner uses its current defaults.
    TRANSITION_OPTIMIZERS = None
    TRANSITION_OPTIMIZERS_BY_EDGE = None


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
        PATH_OPTIMIZER = "SplineGradientBased_bezier3"

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
        PATH_OPTIMIZER = "SplineGradientBased_bezier3"
        # Common optimizer names (non-exhaustive):
        # - "RandomShortcut": random shortcuts between configurations
        # - "SimpleShortcut": simple shortcut optimizer
        # - "PartialShortcut": shortcut optimizer preserving some structure
        # - "SimpleTimeParameterization": adds time parameterization
        # - "RSTimeParameterization": Reeds-Shepp time parameterization
        # Manipulation-specific (hpp-manipulation):
        # - "Graph-RandomShortcut": graph-aware random shortcut
        # - "Graph-PartialShortcut": graph-aware partial shortcut
        # - "EnforceTransitionSemantic": enforce transition semantics
        # Plugin-based (require ps.loadPlugin()):
        # - "SplineGradientBased_bezier{1,3,5}": spline optimizer
        # - "TOPPRA": time-optimal path parameterization

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
