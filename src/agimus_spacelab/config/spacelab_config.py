"""
Configuration classes for manipulation tasks.
"""

from typing import Dict, List
import numpy as np

# Default URDF paths
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
    }
}


class RobotJoints:
    """Joint names for the composite robot."""
    
    # UR10 joints (6 DOF)
    UR10 = [
        "spacelab/ur10_joint_1_2",
        "spacelab/ur10_joint_2_3",
        "spacelab/ur10_joint_3_4",
        "spacelab/ur10_joint_4_5",
        "spacelab/ur10_joint_5_6",
        "spacelab/ur10_joint_6_7",
    ]
    
    # VISPA joints (8 DOF: 2 base + 6 arm)
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
    
    # Object root joints (freeflyer: 7 DOF each)
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
        """Get all robot joint names."""
        return cls.UR10 + cls.VISPA_BASE + cls.VISPA_ARM

    @classmethod
    def all_object_root_joints(cls) -> List[str]:
        """Get all object root joint names."""
        return [
            cls.RS1_ROOT,
            cls.RS2_ROOT,
            cls.RS3_ROOT,
            cls.RS4_ROOT,
            cls.RS5_ROOT,
            cls.RS6_ROOT,
            cls.SCREW_DRIVER_ROOT,
            cls.FRAME_GRIPPER_ROOT,
            cls.CLEAT_GRIPPER_ROOT,
        ]


class InitialConfigurations:
    """Initial joint configurations for robots and objects."""
    
    # Robot joint configurations (in radians)
    UR10 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 DOF
    VISPA_BASE = [0.0, 0.0]  # 2 DOF base
    VISPA_ARM = [0.0, 3.14, 0.0, 0.0, 0.0, 0.0]  # 6 DOF arm
    
    # Object poses in XYZRPY format [x, y, z, roll, pitch, yaw]
    RS1 = [
        -0.46567999999999976,
        2.0219499999999999,
        -0.34200800000000015,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ]

    RS2 = [
        0.38432000000000011,
        2.0219489999999993,
        -0.34200799999999987,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ]

    RS3 = [
        0.38432000000000011,
        2.0219489999999993,
        -0.63200800000000001,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ]

    RS4 = [
        0.38432000000000011,
        2.0219489999999993,
        -0.99423100120426089,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ]

    RS5 = [
        -0.46567999967380558,
        2.0133499991226964,
        -0.99421000142184635,
        1.4828606077991944,
        -2.9906356348103711,
        2.1010496873808218,
    ]

    RS6 = [
        -0.46567999999999987,
        2.0219489999999993,
        -0.63200800000000001,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ] 

    SCREW_DRIVER = [
        0.046500000888612281,
        1.3322769978599074,
        -1.2103000009948957,
        3.1415926529240905,
        -3.2988203261954171e-07,
        1.5707963267957175,
    ]
    
    FRAME_GRIPPER = [
        -0.13349999871947557,
        1.3322770029736009,
        -1.2132999986768855,
        -5.098912860707481e-09,
        -3.1415923241839976,
        1.5707963267958001,
    ]
    
    CLEAT_GRIPPER = [
        0.04650005078371261,
        1.510277021366528,
        -1.212900037140666,
        4.1022589876578654e-07,
        3.1415906427953018,
        0.78539700669219625,
    ]


class JointBounds:
    """Joint bounds for robots and free-flying objects."""
    
    # Robot joint limits (in radians)
    UR10_LIMITS = [(-2 * np.pi, 2 * np.pi)] * 6
    
    VISPA_BASE_LIMITS = [
        (-5.0, 5.0),  # x
        (-5.0, 5.0),  # y
    ]
    
    VISPA_ARM_LIMITS = [(-np.pi, np.pi)] * 6
    
    # Freeflyer bounds for objects
    TRANSLATION_BOUNDS = [
        (-2.0, 2.0),  # x
        (-3.0, 3.0),  # y
        (-2.0, 2.0),  # z
    ]
    
    # Quaternion bounds (must contain unit quaternion)
    QUATERNION_BOUNDS = [
        (-1.0001, 1.0001),  # qx
        (-1.0001, 1.0001),  # qy
        (-1.0001, 1.0001),  # qz
        (-1.0001, 1.0001),  # qw
    ]
    
    @classmethod
    def freeflyer_bounds(cls) -> List[float]:
        """Get combined translation + quaternion bounds as flat list."""
        bounds = []
        for t_bound in cls.TRANSLATION_BOUNDS:
            bounds.extend(t_bound)
        for q_bound in cls.QUATERNION_BOUNDS:
            bounds.extend(q_bound)
        return bounds
    
    @classmethod
    def all_robot_bounds(cls) -> Dict[str, List[float]]:
        """Get all robot joint bounds."""
        bounds = {}
        
        # UR10 bounds
        for i, joint in enumerate(RobotJoints.UR10):
            bounds[joint] = list(cls.UR10_LIMITS[i])
            
        # VISPA base bounds
        for i, joint in enumerate(RobotJoints.VISPA_BASE):
            bounds[joint] = list(cls.VISPA_BASE_LIMITS[i])
            
        # VISPA arm bounds
        for i, joint in enumerate(RobotJoints.VISPA_ARM):
            bounds[joint] = list(cls.VISPA_ARM_LIMITS[i])
            
        return bounds


class ManipulationConfig:
    """Configuration for manipulation planning."""

    # ---------------------------------------------------------------------
    # Defaults (robot/environment/object metadata)
    # ---------------------------------------------------------------------

    # Canonical model paths (URDF/SRDF), shared by all tasks.
    MODEL_PATHS = DEFAULT_PATHS

    # Default scene selection (tasks can override).
    ROBOT_NAMES = ["spacelab"]
    ENVIRONMENT_NAMES = ["ground_demo"]

    # Named robot joint groups (used by tasks to decide what to move/freeze).
    JOINT_GROUPS = {
        "UR10": RobotJoints.UR10,
        "VISPA_BASE": RobotJoints.VISPA_BASE,
        "VISPA_ARM": RobotJoints.VISPA_ARM,
    }

    # Default environment contact surfaces (canonical dict form).
    ENVIRONMENT_CONTACTS = {
        "ground_demo": ["ground_demo/ground_surface"],
    }

    # Define all grippers available on the robots
    GRIPPERS = {
        "ur10": {"spacelab/g_ur10_tool": "spacelab/ur10_joint_6_7"},
        "vispa": {"spacelab/g_vispa_tool": "spacelab/vispa_joint_6_7"},
        "vispa2": {
            "spacelab/g_vispa2_wb1": "spacelab/vispa2_joint_3_eeWorkbench"
        },
        "frame_gripper": {
            "frame_gripper/g_FG_part": "frame_gripper/root_joint"
        },
        "screw_driver": {"screw_driver/g_SD_part": "screw_driver/root_joint"},
    }

    # Define all objects and their handles
    OBJECTS = {
        "frame_gripper": {
            "handles": [
                "frame_gripper/h_FG_tool",
            ],
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
            "handles": [
                "RS1/h_RS1_FG",
                "RS1/h_RS1_WB",
                "RS1/h_RS1_CON0",
                "RS1/h_RS1_CON1",
                "RS1/h_RS1_CON2",
                "RS1/h_RS1_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS1_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS1,
        },
        "RS2": {
            "handles": [
                "RS2/h_RS2_FG",
                "RS2/h_RS2_WB",
                "RS2/h_RS2_CON0",
                "RS2/h_RS2_CON1",
                "RS2/h_RS2_CON2",
                "RS2/h_RS2_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS2_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS2,
        },
        "RS3": {
            "handles": [
                "RS3/h_RS3_FG",
                "RS3/h_RS3_WB",
                "RS3/h_RS3_CON0",
                "RS3/h_RS3_CON1",
                "RS3/h_RS3_CON2",
                "RS3/h_RS3_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS3_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS3,
        },
        "RS4": {
            "handles": [
                "RS4/h_RS4_FG",
                "RS4/h_RS4_WB",
                "RS4/h_RS4_CON0",
                "RS4/h_RS4_CON1",
                "RS4/h_RS4_CON2",
                "RS4/h_RS4_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS4_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS4,
        },
        "RS5": {
            "handles": [
                "RS5/h_RS5_FG",
                "RS5/h_RS5_WB",
                "RS5/h_RS5_CON0",
                "RS5/h_RS5_CON1",
                "RS5/h_RS5_CON2",
                "RS5/h_RS5_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS5_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS5,
        },
        "RS6": {
            "handles": [
                "RS6/h_RS6_FG",
                "RS6/h_RS6_WB",
                "RS6/h_RS6_CON0",
                "RS6/h_RS6_CON1",
                "RS6/h_RS6_CON2",
                "RS6/h_RS6_CON3",
            ],
            "contact_surfaces": [],
            "root_joint": RobotJoints.RS6_ROOT,
            "default_pose_xyzrpy": InitialConfigurations.RS6,
        },
    }

    # Define valid gripper-handle pairs
    VALID_PAIRS = {
        "spacelab/g_ur10_tool": [
            "frame_gripper/h_FG_tool",
            # "screw_driver/h_SD_tool",
        ],
        "spacelab/g_vispa_tool": [
            # "frame_gripper/h_FG_tool",
            "screw_driver/h_SD_tool",
        ],
        "spacelab/g_vispa2_wb1": [
            "RS1/h_RS1_WB",
            "RS2/h_RS2_WB",
            "RS3/h_RS3_WB",
            "RS4/h_RS4_WB",
            "RS5/h_RS5_WB",
            "RS6/h_RS6_WB",
        ],
        "frame_gripper/g_FG_part": [
            "RS1/h_RS1_FG",
            "RS2/h_RS2_FG",
            "RS3/h_RS3_FG",
            "RS4/h_RS4_FG",
            "RS5/h_RS5_FG",
            "RS6/h_RS6_FG",
        ],
        "screw_driver/g_SD_part": [
            "RS1/h_RS1_CON0",
            "RS1/h_RS1_CON1",
            "RS1/h_RS1_CON2",
            "RS1/h_RS1_CON3",
            "RS2/h_RS2_CON0",
            "RS2/h_RS2_CON1",
            "RS2/h_RS2_CON2",
            "RS2/h_RS2_CON3",
            "RS3/h_RS3_CON0",
            "RS3/h_RS3_CON1",
            "RS3/h_RS3_CON2",
            "RS3/h_RS3_CON3",
            "RS4/h_RS4_CON0",
            "RS4/h_RS4_CON1",
            "RS4/h_RS4_CON2",
            "RS4/h_RS4_CON3",
            "RS5/h_RS5_CON0",
            "RS5/h_RS5_CON1",
            "RS5/h_RS5_CON2",
            "RS5/h_RS5_CON3",
            "RS6/h_RS6_CON0",
            "RS6/h_RS6_CON1",
            "RS6/h_RS6_CON2",
            "RS6/h_RS6_CON3",
        ]
    }


__all__ = [
    "DEFAULT_PATHS",
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig",
]
