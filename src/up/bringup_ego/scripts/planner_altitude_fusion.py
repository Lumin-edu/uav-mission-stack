#!/usr/bin/env python3

"""
规划器高度融合节点 — 为 EGO 创建高度一致的里程计和点云。

功能说明：
  Point-LIO 提供精确的水平 (x/y) 位置，但高度 (z) 随时间漂移。
  Point-LIO 的 base_link 数值对应 MID360 IMU 原点；本节点先将其转换为
  机体中心 base，再进行 PX4 高度融合。
  PX4 飞控的测高模块（激光/气压计融合）提供稳定的 z。
  本节点将两者的优势合并：保留补偿后 base 的 x/y/姿态，
  用 PX4 的测量高度替换 z，同时也将点云的 z 做同样的平移补偿。

数据流：
  /odom (Point-LIO base_link/IMU) ──→ base 杆臂补偿 + z 替换 ──→ /ego/odom_fused
  /cloud_registered (世界系点云)  ──→ 同步 z 平移补偿       ──→ /ego/cloud_registered_fused
  
  EGO 规划器消费 /ego/odom_fused 和 /ego/cloud_registered_fused，
  两者在同一个高度参考系下，不会因 Point-LIO z 漂移产生矛盾。

已验证的 visual odom 桥接链路保持原样，
本节点是 EGO 规划链路的独立适配层。
"""

import copy
import math
import struct
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float64

from rigid_transform import compose_pose, normalize_quaternion


def base_height_from_px4_ned(
    reference_base_z: float,
    reference_px4_z: float,
    current_px4_z: float,
) -> float:
    """Map PX4 NED z changes to the ROS-up base-center height."""
    values = (reference_base_z, reference_px4_z, current_px4_z)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("height reference values must be finite")
    return float(reference_base_z) - (float(current_px4_z) - float(reference_px4_z))


class PlannerAltitudeFusion(Node):
    """
    规划位姿融合节点：Point-LIO 补偿后的 base 管水平，PX4 测高模块管垂直。

    工作原理：
      - 先计算 T_odom_base = T_odom_base_link * T_base_link_base
      - 启动时捕获参考点：记下此时 base z 和 PX4 z 的对应关系
      - 之后每帧用 PX4 高度变化反算 base z
      - 点云 z 加 fused_base_z - raw_base_z，保持障碍物与 base 高度一致
      - 输出健康状态到 /ego/height_fusion_healthy
    """

    # dist_bottom_sensor_bitfield 中表示激光测距仪的 bit 位
    # 1 = RANGE_SENSOR (PX4 中 laser_range_finder 类型)
    RANGE_SENSOR_BIT = 1

    def __init__(self) -> None:
        super().__init__("planner_altitude_fusion")

        self.raw_odom_topic = str(
            self.declare_parameter("raw_odom_topic", "/odom").value
        )
        self.raw_cloud_topic = str(
            self.declare_parameter("raw_cloud_topic", "/cloud_registered").value
        )
        self.px4_topic = str(
            self.declare_parameter(
                "px4_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.fused_odom_topic = str(
            self.declare_parameter("fused_odom_topic", "/ego/odom_fused").value
        )
        self.fused_cloud_topic = str(
            self.declare_parameter(
                "fused_cloud_topic", "/ego/cloud_registered_fused"
            ).value
        )
        self.health_topic = str(
            self.declare_parameter("health_topic", "/ego/height_fusion_healthy").value
        )
        self.correction_topic = str(
            self.declare_parameter(
                "correction_topic", "/ego/pointlio_z_correction"
            ).value
        )
        self.vehicle_frame = str(
            self.declare_parameter("vehicle_frame", "base").value
        )
        # Point-LIO base_link is the MID360 IMU origin. The translation is the
        # aircraft-center base origin expressed in base_link coordinates:
        # IMU -> LiDAR [-0.011, -0.02329, 0.04412] plus
        # LiDAR -> aircraft center [0, 0, -0.10].
        self.base_link_to_base_translation = self._vector_parameter(
            "base_link_to_base_translation", [-0.011, -0.02329, -0.05588], 3
        )
        self.base_link_to_base_rotation_xyzw = normalize_quaternion(
            self._vector_parameter(
                "base_link_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0], 4
            )
        )
        self.require_rangefinder = bool(
            self.declare_parameter("require_rangefinder", True).value
        )
        self.max_px4_age_sec = float(
            self.declare_parameter("max_px4_age_sec", 0.30).value
        )
        self.max_odom_age_sec = float(
            self.declare_parameter("max_odom_age_sec", 0.30).value
        )
        self.max_correction_age_sec = float(
            self.declare_parameter("max_correction_age_sec", 0.30).value
        )
        self.max_abs_correction_m = float(
            self.declare_parameter("max_abs_correction_m", 3.0).value
        )
        self.print_rate_hz = float(
            self.declare_parameter("print_rate_hz", 1.0).value
        )
        if min(
            self.max_px4_age_sec,
            self.max_odom_age_sec,
            self.max_correction_age_sec,
            self.max_abs_correction_m,
        ) <= 0.0:
            raise ValueError("fusion ages and correction limit must be positive")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_pub = self.create_publisher(Odometry, self.fused_odom_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.fused_cloud_topic, 10)
        self.health_pub = self.create_publisher(Bool, self.health_topic, health_qos)
        self.correction_pub = self.create_publisher(Float64, self.correction_topic, 10)
        self.create_subscription(Odometry, self.raw_odom_topic, self.odom_callback, 10)
        self.create_subscription(PointCloud2, self.raw_cloud_topic, self.cloud_callback, 10)
        self.create_subscription(VehicleLocalPosition, self.px4_topic, self.px4_callback, px4_qos)

        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_px4_monotonic: Optional[float] = None
        self.latest_odom_monotonic: Optional[float] = None
        self.ref_base_z: Optional[float] = None
        self.ref_px4_z: Optional[float] = None
        self.last_px4_z_reset_counter: Optional[int] = None
        self.latest_z_correction: Optional[float] = None
        self.latest_correction_monotonic: Optional[float] = None
        self.last_health: Optional[bool] = None
        self.last_print_monotonic = -math.inf
        self.last_warn_monotonic = -math.inf
        self.create_timer(0.1, self.health_timer)

        self.get_logger().info(
            "Hardware planner altitude fusion: "
            f"base-center x/y + PX4 z, require_rangefinder={self.require_rangefinder}, "
            f"{self.raw_odom_topic} -> {self.fused_odom_topic}, "
            f"{self.raw_cloud_topic} -> {self.fused_cloud_topic}, "
            f"output_child={self.vehicle_frame}, "
            f"t_base_link_base={self.base_link_to_base_translation}"
        )

    def _vector_parameter(self, name: str, default: list[float], size: int) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.declare_parameter(name, default).value)
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {size} finite values")
        return values

    @staticmethod
    def finite(*values: float) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_monotonic >= 2.0:
            self.last_warn_monotonic = now
            self.get_logger().warn(message)

    def px4_message_valid(self, msg: Optional[VehicleLocalPosition]) -> bool:
        """
        检查 PX4 消息是否为有效的高度数据源。

        条件：
          - z_valid 和 v_z_valid 均为 true（PX4 高度估计有效）
          - z 和 vz 值有限
          - 若 require_rangefinder=True，还需要 dist_bottom 有效且为激光测距仪
        """
        if msg is None or not msg.z_valid or not msg.v_z_valid:
            return False
        if not self.finite(msg.z, msg.vz):
            return False
        if self.require_rangefinder and not (
            msg.dist_bottom_valid
            and int(msg.dist_bottom_sensor_bitfield) & self.RANGE_SENSOR_BIT
        ):
            return False
        return True

    def px4_is_fresh(self) -> bool:
        return bool(
            self.px4_message_valid(self.latest_px4)
            and self.latest_px4_monotonic is not None
            and time.monotonic() - self.latest_px4_monotonic <= self.max_px4_age_sec
        )

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        """
        PX4 本地位置回调，同时处理 z_reset_counter 变化。

        PX4 的 z 是 NED 系（D轴向下），对应高度是 -z。
        如果 PX4 发生了 EKF z 重置（z_reset_counter 变化），
        会通过 delta_z 平移内部存储的参考点，保持 EGO 高度的连续性。
        """
        reset_counter = int(msg.z_reset_counter)
        if self.last_px4_z_reset_counter is None:
            self.last_px4_z_reset_counter = reset_counter
        elif reset_counter != self.last_px4_z_reset_counter:
            delta_z = float(msg.delta_z)
            if self.ref_px4_z is not None and math.isfinite(delta_z):
                # PX4 reports new_z = old_z + delta_z. Move the stored origin
                # by the same delta to keep EGO's ROS-up height continuous.
                self.ref_px4_z += delta_z
                self.get_logger().warn(
                    "PX4 z reset detected; rebased planner height by "
                    f"delta_z={delta_z:+.3f} m."
                )
            self.last_px4_z_reset_counter = reset_counter
        self.latest_px4 = msg
        self.latest_px4_monotonic = time.monotonic()

    def capture_reference(self, base_z: float) -> bool:
        """
        捕获初始高度参考点。

        PX4 的 z 是 NED D轴（向下为正），补偿后 base 的 z 是 ROS 系（向上为正）。
        两者符号相反，此处记录原始的数值关系：
          - ref_base_z: 启动时 base 的 z（ROS，上正）
          - ref_px4_z: 启动时 PX4 NED 的 z（NED，下正）

        后续融合通过公式计算 EGO 用的 z（ROS 上正）：
          ego_z = ref_base_z - (px4_current_z - ref_px4_z)
        """
        if self.ref_base_z is not None and self.ref_px4_z is not None:
            return True
        if not self.px4_is_fresh() or self.latest_px4 is None:
            return False
        self.ref_base_z = float(base_z)
        self.ref_px4_z = float(self.latest_px4.z)
        self.last_px4_z_reset_counter = int(self.latest_px4.z_reset_counter)
        self.get_logger().info(
            "Captured hardware planner height reference: "
            f"base_z={self.ref_base_z:+.3f} m, "
            f"px4_ned_z={self.ref_px4_z:+.3f} m."
        )
        return True

    def pointlio_base_link_pose_to_base(
        self, msg: Odometry
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        """把 Point-LIO 的 odom->base_link(IMU) 位姿转换成 odom->base。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        try:
            # 核心杆臂补偿：
            #   T_odom_base = T_odom_base_link * T_base_link_base
            #   p_odom_base = p_odom_base_link
            #                 + R_odom_base_link * t_base_link_base
            # 固定平移必须先随当前姿态旋转到 odom，不能直接逐轴相加。
            return compose_pose(
                (float(p.x), float(p.y), float(p.z)),
                (float(q.x), float(q.y), float(q.z), float(q.w)),
                self.base_link_to_base_translation,
                self.base_link_to_base_rotation_xyzw,
            )
        except ValueError as error:
            self.warn(f"Rejecting invalid Point-LIO pose: {error}")
            return None

    def odom_callback(self, msg: Odometry) -> None:
        """
        Point-LIO 里程计回调 — 核心融合逻辑。

        步骤：
          1. 将 base_link/IMU 位姿补偿为机体中心 base 位姿
          2. 首次调用时用 base z 捕获高度参考点 (capture_reference)
          3. 计算 EGO 用的融合高度 z：
               px4_delta = px4_current_z - ref_px4_z          ← PX4 高度变化 (NED)
               ego_z = ref_base_z - px4_delta                  ← 反算回 ROS 上正高度
          4. 点云校正量 = fused_base_z - raw_base_z
          5. 融合速度：vz = -px4_vz（NED vz 取反）
          6. 输出 child_frame_id=base 的 /ego/odom_fused
        """
        vehicle_pose = self.pointlio_base_link_pose_to_base(msg)
        if vehicle_pose is None:
            return
        vehicle_position, vehicle_orientation = vehicle_pose
        base_x, base_y, base_z = vehicle_position
        if not self.finite(base_x, base_y, base_z):
            self.warn("Rejecting non-finite Point-LIO odometry in altitude fusion.")
            return
        if not self.capture_reference(base_z):
            self.warn("Waiting for valid PX4 range-aided vertical state.")
            return
        if not self.px4_is_fresh() or self.latest_px4 is None:
            self.warn("Planner altitude fusion stopped because PX4 height is invalid or stale.")
            return

        # 核心融合公式：
        # PX4 z 在 NED 系下向下为正：上升时 z 减小，下降时 z 增大。
        # base z 在 ROS 系中向上为正：上升时 z 增大，下降时 z 减小。
        # 用参考点对齐后，通过 PX4 z 的变化量反推出 ROS 系下的当前高度。
        #   px4_delta = px4_current_z - ref_px4_z    (上升时为负，下降时为正)
        #   ego_z = ref_base_z - px4_delta            (ROS 上正，符号与 NED 相反)
        fused_z = base_height_from_px4_ned(
            self.ref_base_z,
            self.ref_px4_z,
            float(self.latest_px4.z),
        )
        # 点云位于 odom 世界系，不做 x/y 杆臂平移。这里只使用同一个高度差
        # 平移点云 z，使障碍物与 EGO 使用的 base 中心保持相同高度参考。
        correction = fused_z - base_z
        # 修正量越界保护（防止 PX4 高度突变导致点云跳变）
        if not self.finite(fused_z, correction) or abs(correction) > self.max_abs_correction_m:
            self.warn(
                "Planner altitude correction rejected: "
                f"correction={correction:+.3f} m, limit={self.max_abs_correction_m:.3f} m."
            )
            return

        # x/y/orientation 来自补偿后的 base；z 来自 PX4 测高反算。
        fused = copy.deepcopy(msg)
        fused.child_frame_id = self.vehicle_frame
        fused.pose.pose.position.x = base_x
        fused.pose.pose.position.y = base_y
        fused.pose.pose.position.z = fused_z
        fused.pose.pose.orientation.x = vehicle_orientation[0]
        fused.pose.pose.orientation.y = vehicle_orientation[1]
        fused.pose.pose.orientation.z = vehicle_orientation[2]
        fused.pose.pose.orientation.w = vehicle_orientation[3]
        # vz 转换：PX4 NED 的 vz（下正）→ ROS 的 vz（上正），取反
        fused.twist.twist.linear.z = -float(self.latest_px4.vz)
        if self.finite(self.latest_px4.epv) and float(self.latest_px4.epv) >= 0.0:
            fused.pose.covariance[14] = float(self.latest_px4.epv) ** 2
        if self.finite(self.latest_px4.evv) and float(self.latest_px4.evv) >= 0.0:
            fused.twist.covariance[14] = float(self.latest_px4.evv) ** 2

        self.latest_z_correction = correction
        publish_time = time.monotonic()
        self.latest_odom_monotonic = publish_time
        self.latest_correction_monotonic = publish_time
        self.odom_pub.publish(fused)
        self.correction_pub.publish(Float64(data=correction))

        now = time.monotonic()
        if self.print_rate_hz > 0.0 and now - self.last_print_monotonic >= 1.0 / self.print_rate_hz:
            self.last_print_monotonic = now
            self.get_logger().info(
                "planner height fused | "
                f"base_z={base_z:+.3f} px4_ned_z={float(self.latest_px4.z):+.3f} "
                f"ego_z={fused_z:+.3f} cloud_dz={correction:+.3f} m"
            )

    @staticmethod
    def shift_cloud_z(msg: PointCloud2, correction: float) -> Optional[PointCloud2]:
        """
        将点云中每个点的 z 字段加上修正量。

        通过 fused_base_z - raw_base_z 把 Point-LIO 世界系点云的 z 与
        PX4 反算高度对齐，确保障碍物和机体中心在 EGO 看来使用同一个高度参考。

        操作：
          - 找到 z 字段（必须是 FLOAT32 类型）
          - 遍历每个点，z += correction
        """
        z_fields = [field for field in msg.fields if field.name == "z"]
        if len(z_fields) != 1:
            return None
        field = z_fields[0]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            return None
        if msg.point_step <= 0 or field.offset + 4 > msg.point_step:
            return None

        data = bytearray(msg.data)
        if int(msg.row_step) * int(msg.height) > len(data):
            return None
        fmt = ">f" if msg.is_bigendian else "<f"
        for row in range(int(msg.height)):
            row_base = row * int(msg.row_step)
            for column in range(int(msg.width)):
                offset = row_base + column * int(msg.point_step) + int(field.offset)
                value = struct.unpack_from(fmt, data, offset)[0]
                if math.isfinite(value):
                    struct.pack_into(fmt, data, offset, value + correction)

        shifted = copy.deepcopy(msg)
        shifted.data = bytes(data)
        return shifted

    def cloud_callback(self, msg: PointCloud2) -> None:
        if (
            not self.px4_is_fresh()
            or self.latest_z_correction is None
            or self.latest_correction_monotonic is None
            or time.monotonic() - self.latest_correction_monotonic
            > self.max_correction_age_sec
        ):
            self.warn("Registered cloud withheld until a fresh fused height is available.")
            return
        shifted = self.shift_cloud_z(msg, self.latest_z_correction)
        if shifted is None:
            self.warn("Registered cloud does not contain a supported float32 z field.")
            return
        self.cloud_pub.publish(shifted)

    def health_timer(self) -> None:
        now = time.monotonic()
        odom_fresh = bool(
            self.latest_odom_monotonic is not None
            and now - self.latest_odom_monotonic <= self.max_odom_age_sec
        )
        correction_fresh = bool(
            self.latest_z_correction is not None
            and self.latest_correction_monotonic is not None
            and now - self.latest_correction_monotonic
            <= self.max_correction_age_sec
        )
        healthy = bool(
            self.px4_is_fresh()
            and odom_fresh
            and correction_fresh
        )
        if healthy != self.last_health:
            self.last_health = healthy
            self.get_logger().info(
                "Hardware planner altitude fusion healthy."
                if healthy
                else "Hardware planner altitude fusion not ready or unhealthy."
            )
        self.health_pub.publish(Bool(data=healthy))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerAltitudeFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
