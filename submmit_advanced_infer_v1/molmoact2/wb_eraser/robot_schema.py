"""Shared AIWorker SG2 observation names and timestamp validation."""

import math

CAMERAS = ("cam_left_head", "cam_left_wrist", "cam_right_wrist")
ARM_LEFT = tuple(f"arm_l_joint{i}" for i in range(1, 8)) + ("gripper_l_joint1",)
ARM_RIGHT = tuple(f"arm_r_joint{i}" for i in range(1, 8)) + ("gripper_r_joint1",)
STATIONARY_JOINTS = ARM_LEFT + ARM_RIGHT + ("head_joint1", "head_joint2", "lift_joint")
MOBILE_JOINTS = STATIONARY_JOINTS + ("linear_x", "linear_y", "angular_z")
PROFILES = {"stationary": STATIONARY_JOINTS, "mobile": MOBILE_JOINTS}
SLICES = {"arm_left": (0, 8), "arm_right": (8, 16), "head": (16, 18), "lift": (18, 19)}


def check_timestamps(stamps_ns, now_ns, max_age_s, max_skew_s):
    if (not stamps_ns or any(not isinstance(s, int) or s <= 0 for s in stamps_ns)
            or not all(math.isfinite(v) and v > 0 for v in (max_age_s, max_skew_s))):
        raise ValueError("invalid observation timestamps or timing limits")
    ages = [(now_ns - stamp) / 1e9 for stamp in stamps_ns]
    if min(ages) < -0.05 or max(ages) > max_age_s:
        raise ValueError("observation is stale or its clock differs from the ROS client")
    if (max(stamps_ns) - min(stamps_ns)) / 1e9 > max_skew_s:
        raise ValueError("camera/joint-state timestamp skew exceeds the configured bound")
