from typing import List


def joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
        prefix + "joint7",
    ]


def base_link_name(prefix: str = "panda_") -> str:
    return prefix + "link0"


def end_effector_name(prefix: str = "panda_") -> str:
    return prefix + "hand_tcp"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]


def open_gripper_joint_positions() -> List[float]:
    return [0.04, 0.04]


def closed_gripper_joint_positions() -> List[float]:
    return [0.0, 0.0]
