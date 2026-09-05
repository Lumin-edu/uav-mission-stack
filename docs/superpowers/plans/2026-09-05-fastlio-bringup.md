# FAST-LIO Fixed-Point Bringup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a standalone `src/fastlio_bringup` ROS 2 package that runs FAST-LIO localization and safe PX4 fixed-point hover without importing files from `bringup_pointlio_hover`.

**Architecture:** The package-local launch file starts `fast_lio/fastlio_mapping`, a package-local FAST-LIO bridge, optional diagnostics, and a package-local fixed-point Offboard controller. The bridge consumes FAST-LIO `/Odometry` (`camera_init -> body`), applies a configurable body-to-base rigid transform, aligns to PX4 NED, and publishes `VehicleOdometry`; the controller consumes PX4 local position and publishes position-mode Offboard setpoints.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, Python 3, `rclpy`, `nav_msgs`, `px4_msgs`, `tf2_ros`, FAST-LIO C++ node, `unittest`, `colcon`.

**Spec:** `docs/superpowers/specs/2026-09-05-fastlio-bringup-design.md`

## Global Constraints

- All package-owned launch, config, scripts, tests, and docs live below `src/fastlio_bringup`.
- No script, launch file, or install rule may import, execute, or copy from `bringup_pointlio_hover`.
- Livox driver and `MicroXRCEAgent` are external processes; launch only consumes their ROS topics.
- FAST-LIO input topics default to `/livox/lidar` and `/livox/imu`; estimator output defaults to `/Odometry` with `camera_init -> body` semantics.
- Default body-to-base transform is `[-0.011, -0.02329, -0.05588]` translation and identity quaternion.
- Default coordinate conversion is ROS `(x, y, z)` to PX4 NED `(x, -y, -z)`.
- Automatic arming is disabled by default; Offboard requires a valid reference and a two-second pre-stream.
- Verify with standalone unit tests, Python compilation, `ros2 launch --show-args`, and a package-selective colcon build when ROS dependencies are available.

## File Map

- Create: `src/fastlio_bringup/CMakeLists.txt` - ament package, install rules, and test registration.
- Create: `src/fastlio_bringup/package.xml` - runtime/build dependencies and package metadata.
- Create: `src/fastlio_bringup/launch/fastlio_hover.launch.py` - complete launch composition and declared arguments.
- Create: `src/fastlio_bringup/config/mid360_fastlio.yaml` - package-local FAST-LIO MID-360 parameters.
- Create: `src/fastlio_bringup/scripts/rigid_transform.py` - pure-Python transform math used by the bridge and tests.
- Create: `src/fastlio_bringup/scripts/fastlio_to_px4_visual_odom.py` - FAST-LIO `/Odometry` to PX4 `VehicleOdometry` bridge.
- Create: `src/fastlio_bringup/scripts/fixed_point_hover.py` - safe position-mode Offboard controller.
- Create: `src/fastlio_bringup/scripts/px4_dds_monitor.py` - PX4 topic health monitor.
- Create: `src/fastlio_bringup/scripts/px4_control_watchdog.py` - required-input publisher watchdog.
- Create: `src/fastlio_bringup/scripts/px4_fastlio_position_compare.py` - PX4 vs. bridge NED increment logger.
- Create: `src/fastlio_bringup/test/test_rigid_transform.py` - transform behavior tests runnable without ROS.
- Create: `src/fastlio_bringup/README.md` - build, external process startup, launch examples, frame semantics, and checks.

### Task 1: Add transform regression tests

**Files:**
- Create: `src/fastlio_bringup/test/test_rigid_transform.py`
- Create: `src/fastlio_bringup/scripts/rigid_transform.py`

**Interfaces:**
- Produces `normalize_quaternion(values)`, `quaternion_multiply(lhs, rhs)`, `rotate_vector(quaternion, vector)`, and `compose_pose(world_parent_position, world_parent_orientation, parent_child_translation, parent_child_rotation)` for the bridge.

- [ ] **Step 1: Write the failing tests**

Create a test module that adds the package `scripts` directory to `sys.path` and asserts:

```python
def test_level_pose_applies_body_to_base_offset_with_correct_sign():
    position, orientation = compose_pose((0.0, 0.0, 0.0), IDENTITY, BODY_TO_BASE, IDENTITY)
    assert_close(position, BODY_TO_BASE)
    assert_close(orientation, IDENTITY)

def test_yaw_rotates_body_lever_arm_into_world():
    position, _ = compose_pose((1.0, 2.0, 3.0), yaw_90, BODY_TO_BASE, IDENTITY)
    assert_close(position, (1.02329, 1.989, 2.94412))

def test_roll_and_pitch_rotate_vertical_offset():
    assert_close(rotate_vector(roll_90, (0.0, 0.0, -0.10)), (0.0, 0.10, 0.0))
    assert_close(rotate_vector(pitch_90, (0.0, 0.0, -0.10)), (-0.10, 0.0, 0.0))

def test_static_rotation_is_composed_after_body_orientation():
    _, orientation = compose_pose((0.0, 0.0, 0.0), yaw_90, (0.0, 0.0, 0.0), yaw_90)
    assert_close(orientation, (0.0, 0.0, 1.0, 0.0))

def test_zero_quaternion_is_rejected():
    with self.assertRaisesRegex(ValueError, "norm is zero"):
        compose_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), BODY_TO_BASE, IDENTITY)
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 src/fastlio_bringup/test/test_rigid_transform.py
```

Expected: import failure because `scripts/rigid_transform.py` does not exist yet.

- [ ] **Step 3: Implement the minimal transform helper**

Implement finite length/value checks, quaternion normalization, Hamilton product, quaternion-vector rotation using the expanded `q * v * q^-1` formula, and pose composition with the explicit `T_world_child = T_world_parent * T_parent_child` contract. Reject vectors of the wrong length, non-finite values, and zero-norm quaternions.

- [ ] **Step 4: Run the transform tests to verify they pass**

Run the same command and expect all five tests to pass with exit code 0.

- [ ] **Step 5: Commit the transform unit**

```bash
git add src/fastlio_bringup/test/test_rigid_transform.py src/fastlio_bringup/scripts/rigid_transform.py
git commit -m "test: add FAST-LIO bringup transform contract"
```

### Task 2: Add the FAST-LIO to PX4 bridge

**Files:**
- Create: `src/fastlio_bringup/scripts/fastlio_to_px4_visual_odom.py`

**Interfaces:**
- Consumes `nav_msgs/msg/Odometry` on `fastlio_topic` (default `/Odometry`), PX4 `VehicleLocalPosition`, and optional `TimesyncStatus`.
- Publishes PX4 `VehicleOdometry` on `output_topic` (default `/fmu/in/vehicle_visual_odometry`).
- Uses Task 1 `compose_pose` and exposes parameters `body_to_base_translation`, `body_to_base_rotation_xyzw`, `fastlio_to_px4_rotation`, `fastlio_y_to_px4_y_sign`, `yaw_sign`, `yaw_offset`, `publish_orientation`, `publish_velocity`, `use_px4_reference`, `max_output_abs_z`, and `max_position_jump`.

- [ ] **Step 1: Add a bridge-focused failing smoke test**

Extend `test/test_rigid_transform.py` with a source-contract test that reads the bridge source and asserts it contains the FAST-LIO defaults and does not contain a runtime import of `bringup_pointlio_hover`:

```python
def test_bridge_defaults_to_fastlio_odometry_and_never_imports_pointlio_package():
    source = Path(__file__).resolve().parents[1] / "scripts" / "fastlio_to_px4_visual_odom.py"
    text = source.read_text(encoding="utf-8")
    assert '"/Odometry"' in text
    assert '"body_to_base_translation"' in text
    assert "bringup_pointlio_hover" not in text
```

Run the test before creating the bridge and verify it fails because the source file is absent.

- [ ] **Step 2: Implement the bridge**

Port the established visual-odometry behavior into a package-local node, renaming all user-facing log messages and parameter names to FAST-LIO/body terminology. The callback must:

1. Validate and compose the FAST-LIO body pose into the vehicle base pose.
2. Capture the first base pose and optional PX4 reference/heading.
3. Map the relative position to NED with the configured 3x3 matrix and yaw offset.
4. Drop non-finite, out-of-Z-limit, and excessive-jump outputs.
5. Fill `VehicleOdometry` timestamp, NED pose, optional yaw quaternion, optional NED velocity, variances, reset counter, and quality, then publish.

Use BEST_EFFORT/TRANSIENT_LOCAL QoS compatible with PX4 DDS and throttle warnings/log output. Keep imports limited to standard library, `rclpy`, `nav_msgs`, `px4_msgs`, and the package-local `rigid_transform` module.

- [ ] **Step 3: Run bridge source and syntax tests**

Run:

```bash
python3 src/fastlio_bringup/test/test_rigid_transform.py
python3 -m py_compile src/fastlio_bringup/scripts/fastlio_to_px4_visual_odom.py
```

Expected: all tests pass and compilation exits 0.

- [ ] **Step 4: Commit the bridge**

```bash
git add src/fastlio_bringup/scripts/fastlio_to_px4_visual_odom.py src/fastlio_bringup/test/test_rigid_transform.py
git commit -m "feat: add FAST-LIO PX4 visual odometry bridge"
```

### Task 3: Add the fixed-point controller and diagnostics

**Files:**
- Create: `src/fastlio_bringup/scripts/fixed_point_hover.py`
- Create: `src/fastlio_bringup/scripts/px4_dds_monitor.py`
- Create: `src/fastlio_bringup/scripts/px4_control_watchdog.py`
- Create: `src/fastlio_bringup/scripts/px4_fastlio_position_compare.py`

**Interfaces:**
- Controller subscribes to PX4 local position/status and publishes Offboard mode, trajectory setpoint, and vehicle command topics.
- Diagnostics use configurable PX4 topic parameters and never control the vehicle.

- [ ] **Step 1: Add a controller safety-contract test**

Extend the test module with a source-contract test checking that the controller defaults to manual arm, has a two-second pre-stream formula, waits for valid local position, and publishes position mode:

```python
def test_hover_controller_keeps_manual_arm_and_position_mode_defaults():
    text = (Path(__file__).resolve().parents[1] / "scripts" / "fixed_point_hover.py").read_text()
    assert '"auto_arm", False' in text
    assert "2.0 * self.control_rate_hz" in text
    assert "msg.position = True" in text
    assert "msg.xy_valid and msg.z_valid" in text
```

Run it before creating the controller and verify the expected missing-file failure.

- [ ] **Step 2: Implement the controller**

Use the existing safe state sequence: continuously publish heartbeat/setpoint, count at least `max(10, 2 * rate)` cycles, wait for a delayed valid reference, optionally arm no more than once per second, request Offboard no more than once per second after arming, and keep the relative target fixed. Expose topic/rate/target/reference parameters and log manual-arm guidance.

- [ ] **Step 3: Implement diagnostics**

Copy only the behavior needed into package-local files, changing names/logs to FAST-LIO. The DDS monitor reports message receipt and validity every two seconds; the watchdog checks publishers for the three required PX4 input topics; the comparison node compares bridge NED increments to PX4 local-position increments and reports direction mismatches/stale data.

- [ ] **Step 4: Run syntax and test checks**

```bash
python3 src/fastlio_bringup/test/test_rigid_transform.py
python3 -m py_compile src/fastlio_bringup/scripts/*.py
```

Expected: all tests pass and every script compiles.

- [ ] **Step 5: Commit controller and diagnostics**

```bash
git add src/fastlio_bringup/scripts src/fastlio_bringup/test/test_rigid_transform.py
git commit -m "feat: add FAST-LIO hover controller diagnostics"
```

### Task 4: Add package metadata, config, launch, and documentation

**Files:**
- Create: `src/fastlio_bringup/package.xml`
- Create: `src/fastlio_bringup/CMakeLists.txt`
- Create: `src/fastlio_bringup/config/mid360_fastlio.yaml`
- Create: `src/fastlio_bringup/launch/fastlio_hover.launch.py`
- Create: `src/fastlio_bringup/README.md`

**Interfaces:**
- Package name is `hx_fastlio_bringup`.
- Launch executable is `fastlio_hover.launch.py`.
- FAST-LIO node is `fast_lio/fastlio_mapping` and receives the installed YAML path through `parameters`.

- [ ] **Step 1: Add package metadata and install rules**

Declare `ament_cmake` build tooling and runtime dependencies for `launch`, `launch_ros`, `rclpy`, `nav_msgs`, `px4_msgs`, `fast_lio`, `tf2_ros`, and test dependencies. Install `launch`, `config`, and all executable scripts into the package share/lib destinations; register the standalone unittest through `ament_add_test`.

- [ ] **Step 2: Add package-local FAST-LIO configuration**

Create `mid360_fastlio.yaml` with `/livox/lidar`, `/livox/imu`, Livox type 1, four scan lines, timestamp unit 3, 10 Hz scan rate, estimator covariances, MID-360 extrinsic translation/identity rotation, and publication flags suitable for odometry plus optional map/scan visualization.

- [ ] **Step 3: Add the launch description**

Declare switches `use_fastlio`, `use_visual_odom`, `use_hover_control`, `use_px4_monitor`, `use_px4_control_watchdog`, `use_position_compare`, `use_rviz`, and `use_body_to_base_tf`, plus topic/config/safety arguments. Start FAST-LIO with package-local config, optional RViz with the FAST-LIO RViz file, static `body -> base` TF with the same transform constants, the bridge, diagnostics, and controller. Keep defaults manual-arm and conservative.

- [ ] **Step 4: Write operating documentation**

Document build commands, external Livox and `MicroXRCEAgent` startup, a complete launch example, frame semantics (`camera_init -> body`, then body-to-base), topic health checks, manual-arm behavior, and warnings about only one PX4 Offboard source.

- [ ] **Step 5: Run package checks**

```bash
python3 -m py_compile src/fastlio_bringup/launch/fastlio_hover.launch.py src/fastlio_bringup/scripts/*.py
source /opt/ros/humble/setup.bash
ros2 launch src/fastlio_bringup/launch/fastlio_hover.launch.py --show-args
colcon build --packages-select hx_fastlio_bringup --symlink-install
```

Expected: Python compilation succeeds; launch arguments render; colcon exits 0 when the workspace dependencies are installed. If ROS is unavailable, record that limitation and still run every non-ROS check.

- [ ] **Step 6: Commit the package surface**

```bash
git add src/fastlio_bringup
git commit -m "feat: add standalone FAST-LIO fixed-point bringup"
```

### Task 5: Final verification and requirement audit

**Files:**
- Verify all files under `src/fastlio_bringup` and the design/plan docs.

- [ ] **Step 1: Check independence**

Run:

```bash
rg -n "bringup_pointlio_hover|pointlio_to_px4|Point-LIO|Pointlio|pointlio" src/fastlio_bringup
```

Expected: no runtime dependency or Point-LIO naming in the new package; any match must be absent, including comments and documentation.

- [ ] **Step 2: Run the full standalone test and syntax suite**

```bash
python3 src/fastlio_bringup/test/test_rigid_transform.py
python3 -m py_compile src/fastlio_bringup/launch/fastlio_hover.launch.py src/fastlio_bringup/scripts/*.py
```

Expected: exit code 0 for both commands.

- [ ] **Step 3: Inspect the final diff and status**

```bash
git diff HEAD~1 -- src/fastlio_bringup
git status --short
```

Confirm no unrelated user changes were modified and the package contains no symlink or absolute path to another bringup package.

- [ ] **Step 4: Report evidence and residual runtime prerequisites**

Report exact test/build results. Explicitly note that real-flight validation still requires the external Livox driver, PX4 firmware matching `px4_msgs`, `MicroXRCEAgent`, and hardware safety checks.
