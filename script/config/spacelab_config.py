"""
Configuration classes for manipulation tasks.
"""

from typing import Dict, List, Optional
import numpy as np


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
    SCREW_DRIVER_ROOT = "screw_driver/root_joint"
    FRAME_GRIPPER_ROOT = "frame_gripper/root_joint"
    CLEAT_GRIPPER_ROOT = "cleat_gripper/root_joint"
    
    @classmethod
    def all_robot_joints(cls) -> List[str]:
        """Get all robot joint names."""
        return cls.UR10 + cls.VISPA_BASE + cls.VISPA_ARM
    
    @classmethod
    def all_object_roots(cls) -> List[str]:
        """Get all object root joint names."""
        return [
            cls.RS1_ROOT,
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
        0.46567999999999976,
        2.0219499999999999,
        -0.34200800000000015,
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
    
    # Define all grippers available on the robots
    GRIPPERS = {
        "ur10_gripper": "spacelab/gripper",
        "vispa_gripper": "spacelab/gripper",
    }
    
    # Define all objects and their handles
    OBJECTS = {
        "frame_gripper": {
            "handles": [
                "frame_gripper/h_FG_tool",
                "frame_gripper/h_FG_side"
            ],
            "contact_surfaces": [],
        },
        "screw_driver": {
            "handles": ["screw_driver/h_SD_tool"],
            "contact_surfaces": [],
        },
        "cleat_gripper": {
            "handles": ["cleat_gripper/h_CG_tool"],
            "contact_surfaces": [],
        },
        "RS1": {
            "handles": ["RS1/h_RS_top", "RS1/h_RS_front"],
            "contact_surfaces": [],
        },
    }
    
    # Define valid gripper-handle pairs
    VALID_PAIRS = {
        "ur10_gripper": [
            "frame_gripper/h_FG_tool",
            "screw_driver/h_SD_tool",
            "RS1/h_RS_top",
        ],
        "vispa_gripper": [
            "cleat_gripper/h_CG_tool",
            "RS1/h_RS_front",
            "frame_gripper/h_FG_side",
        ],
    }
    
    # Environment contact surfaces
    ENV_CONTACTS = ["ground_demo/surface"]


__all__ = [
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig",
]
