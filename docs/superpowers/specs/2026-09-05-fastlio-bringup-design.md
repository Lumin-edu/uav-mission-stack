# FAST-LIO Fixed-Point Bringup Design

## Goal

Add a self-contained ROS 2 package at `src/fastlio_bringup` for real-vehicle
fixed-point hover using the existing FAST-LIO package as the estimator and PX4
offboard control as the actuator interface.

## Scope and Boundary

The package owns its launch file, FAST-LIO parameter file, PX4 visual-odometry
bridge, fixed-point controller, diagnostics, coordinate-transform helper,
tests, and operating documentation. It must not import, execute, install, or
reference files from `bringup_pointlio_hover`.

Livox driver startup and `MicroXRCEAgent` startup remain external operations.
The package consumes `/livox/lidar`, `/livox/imu`, and PX4 DDS topics but does
not manage USB/network device processes.

The only external ROS package used for localization is `fast_lio`; its node is
started as `fast_lio/fastlio_mapping` with the package-local YAML file. PX4
messages come from `px4_msgs`, and the optional static transform uses
`tf2_ros`.

## Data Flow

```text
external Livox driver
  -> /livox/lidar and /livox/imu
  -> fast_lio/fastlio_mapping
  -> /Odometry (camera_init -> body)
  -> fastlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> fixed_point_hover.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint and /fmu/in/vehicle_command
```

FAST-LIO publishes the estimated IMU body origin as `body`. The bridge treats
this as `body_link` semantically and applies a configurable rigid transform to
the vehicle-center frame `base` before alignment and NED conversion. The
default transform is the existing MID-360 installation measurement:

```yaml
body_to_base_translation: [-0.011, -0.02329, -0.05588]
body_to_base_rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

The transform is applied as
`T_world_base = T_world_body * T_body_base`; the translation is rotated by the
current FAST-LIO orientation, so roll/pitch do not turn the lever arm into a
constant world-axis offset.

## Components

### `launch/fastlio_hover.launch.py`

Declares package-level switches for FAST-LIO, bridge, hover control, monitors,
comparison, RViz, and static TF. It starts FAST-LIO with local configuration,
then starts the bridge and controller with explicit topic and transform
parameters. Defaults are conservative: visual odometry and hover enabled,
manual arm required, 50 Hz control, 0.2 m relative takeoff target, and no RViz
or verbose monitors.

### `config/mid360_fastlio.yaml`

Contains only the parameters needed by the installed FAST-LIO node for the
Livox MID-360 topics, estimator, publication, and extrinsics. It is copied from
the known FAST-LIO MID-360 values but is package-local and can be changed
without editing the upstream perception package.

### `scripts/fastlio_to_px4_visual_odom.py`

Subscribes to `nav_msgs/msg/Odometry` on `/Odometry`, `VehicleLocalPosition`
and optional `TimesyncStatus`. It validates finite poses, composes the body to
base transform, captures the initial estimator reference, maps ROS FLU-like
coordinates to PX4 NED `(x, -y, -z)`, optionally uses the first valid PX4 local
position as the absolute reference, rejects excessive jumps, and publishes
`VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`.

### `scripts/fixed_point_hover.py`

Publishes position-mode offboard heartbeats and a position setpoint at a
configurable rate. It waits for valid PX4 local position, captures a relative
target after a delay, pre-streams at least two seconds, optionally arms, and
only requests Offboard after arming. Manual arm is the default.

### Diagnostics

`px4_dds_monitor.py` reports status, local-position validity, and range data.
`px4_control_watchdog.py` reports whether the three required PX4 input topics
have publishers. `px4_fastlio_position_compare.py` compares relative PX4 local
position against the bridge's NED visual-odometry output.

### `scripts/rigid_transform.py` and tests

The helper contains finite-vector validation, quaternion normalization and
composition, vector rotation, and pose composition. Unit tests cover offset
sign, yaw/roll/pitch rotation, quaternion composition, and invalid quaternions.

## Safety and Error Handling

- No automatic arming unless `hover_auto_arm` is explicitly enabled.
- No Offboard request until the required setpoint pre-stream and valid position
  reference are available.
- Invalid/non-finite source poses and output positions are dropped.
- Output Z and frame-to-frame position jumps are bounded by parameters.
- Stale/missing DDS topics are reported by diagnostics.
- Launch arguments expose topic names and all safety-relevant limits.

## Verification

The package must pass the standalone Python unit test without ROS runtime
imports, pass `python3 -m py_compile` for every script and launch file, and
build with:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hx_fastlio_bringup --symlink-install
```

The launch file must parse with `ros2 launch ... --show-args` after sourcing the
workspace, and the README must document external Livox/PX4-agent startup,
launch examples, topic checks, and the FAST-LIO frame semantics.
