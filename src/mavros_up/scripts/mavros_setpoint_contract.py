import math
from dataclasses import dataclass

# MAVROS exposes PositionTarget constants using MAV_FRAME names. The ROS
# message is interpreted in ROS ENU by MAVROS before it is sent to PX4.
FRAME_LOCAL_NED = 1
IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
IGNORE_YAW_RATE = 2048


@dataclass(frozen=True)
class PositionTargetFields:
    coordinate_frame: int
    type_mask: int
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float
    yaw_rate: float


def resolve_target(
    reference: tuple[float, float, float] | None,
    target: tuple[float, float, float],
    *,
    use_current_position_reference: bool,
) -> tuple[float, float, float] | None:
    if not use_current_position_reference:
        return tuple(float(value) for value in target)
    if reference is None:
        return None
    return tuple(float(reference[i] + target[i]) for i in range(3))


def quaternion_to_yaw(
    orientation: tuple[float, float, float, float] | None,
) -> float | None:
    if orientation is None or len(orientation) != 4:
        return None
    x, y, z, w = (float(value) for value in orientation)
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return None
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def resolve_yaw(
    orientation: tuple[float, float, float, float] | None,
    *,
    configured_yaw: float,
    hold_current_yaw: bool,
) -> float | None:
    if not hold_current_yaw:
        return float(configured_yaw)
    return quaternion_to_yaw(orientation)


def make_position_target(
    position: tuple[float, float, float],
    yaw: float,
    *,
    yaw_rate: float = 0.0,
) -> PositionTargetFields:
    return PositionTargetFields(
        FRAME_LOCAL_NED,
        IGNORE_VX
        | IGNORE_VY
        | IGNORE_VZ
        | IGNORE_AFX
        | IGNORE_AFY
        | IGNORE_AFZ
        | IGNORE_YAW_RATE,
        tuple(float(value) for value in position),
        (math.nan, math.nan, math.nan),
        (math.nan, math.nan, math.nan),
        float(yaw),
        float(yaw_rate),
    )
