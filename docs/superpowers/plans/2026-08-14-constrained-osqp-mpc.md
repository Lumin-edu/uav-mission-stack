# Constrained OSQP MPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace first-command-only clipping with an OSQP QP that enforces conservative hard limits across the complete MPC horizon.

**Architecture:** Move numerical MPC into a focused `constrained_mpc.py` module. Reuse constant sparse Hessian and constraint matrices, update state/reference-dependent vectors per control cycle, and expose immutable solve statistics to the ROS wrapper.

**Tech Stack:** Python 3.10, NumPy, SciPy sparse matrices, OSQP, ROS 2 Humble, `unittest`/ament.

**Spec:** `docs/superpowers/specs/2026-08-14-constrained-osqp-mpc-design.md`

## Global Constraints

- Never silently fall back to the unconstrained dense solver.
- Accept only an OSQP `solved` result within the configured solve-time and constraint tolerances.
- Preserve position-hold fallback and disabled hardware-output defaults.
- Preserve all unrelated user changes and do not create a Git commit without an explicit request.

---

### Task 1: Horizon-wide constrained solver

**Files:**
- Create: `src/up/bringup_ego_mpc/scripts/constrained_mpc.py`
- Modify: `src/up/bringup_ego_mpc/test/test_ego_mpc_controller.py`
- Modify: `src/up/bringup_ego_mpc/CMakeLists.txt`

**Interfaces:**
- Produces: `ConstrainedLinearMpc.solve(state, references, previous) -> np.ndarray`
- Produces: `ConstrainedLinearMpc.last_sequence`, `last_predicted_states`, and `last_stats`
- Produces: `MpcSolveError` and immutable `MpcSolveStats`

- [ ] **Step 1: Write failing horizon tests**

Add tests which demand `last_sequence`, assert every predicted velocity and acceleration remains inside hand-calculated limits, and assert every acceleration difference satisfies its correct time-scaled jerk bound.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/up/bringup_ego_mpc/test/test_ego_mpc_controller.py
```

Expected: import or constructor failure because `ConstrainedLinearMpc` does not exist.

- [ ] **Step 3: Implement the sparse QP**

Build the condensed prediction model and cost. Build constant constraint rows for velocity L1 signs, acceleration L1 signs, a 16-sided horizontal polygon, vertical bounds, and jerk L1 signs. Set up OSQP once and update `q`, `l`, and `u` per solve.

- [ ] **Step 4: Verify GREEN**

Run the same test command. Expected: all numerical and prior safety tests pass.

### Task 2: ROS failure handling and diagnostics

**Files:**
- Modify: `src/up/bringup_ego_mpc/scripts/ego_mpc_controller.py`
- Modify: `src/up/bringup_ego_mpc/test/test_ego_mpc_controller.py`

**Interfaces:**
- Consumes: `ConstrainedLinearMpc`, `MpcSolveError`, and `MpcSolveStats`
- Produces: `/ego_mpc/solver_diagnostics` as `diagnostic_msgs/DiagnosticArray`

- [ ] **Step 1: Write failing controller tests**

Test that an `MpcSolveError` enters the existing hold path and that solve statistics convert into diagnostic key/value fields without non-finite values.

- [ ] **Step 2: Run tests and verify RED**

Expected: missing diagnostic conversion or uncaught `MpcSolveError`.

- [ ] **Step 3: Wire solver and diagnostics**

Replace the in-file dense solver, publish throttled solver diagnostics, and force an ERROR diagnostic on every rejected solution while retaining the position-hold fallback.

- [ ] **Step 4: Verify GREEN**

Run the test file and confirm all tests pass.

### Task 3: Runtime configuration and deployment dependency

**Files:**
- Modify: `src/up/bringup_ego_mpc/package.xml`
- Modify: `src/up/bringup_ego_mpc/launch/ego_mpc_hw.launch.py`
- Modify: `src/up/bringup_ego_mpc/README.md`

**Interfaces:**
- Produces parameters: `solver_eps_abs`, `solver_eps_rel`, `solver_max_iter`, `solver_time_limit_ms`, and `solver_constraint_tolerance`

- [ ] **Step 1: Add dependency and launch configuration**

Declare SciPy and OSQP runtime requirements, pass conservative solver defaults from launch, and validate every parameter at controller startup.

- [ ] **Step 2: Document installation and safety semantics**

Document OSQP installation, conservative polyhedral limits, diagnostic topic, failure-to-hold behavior, and the remaining non-hard-real-time boundary.

- [ ] **Step 3: Parse launch configuration**

Run `ros2 launch hx_bringup_ego_mpc ego_mpc_hw.launch.py --show-args` with ROS logs under `/tmp`; expect all new parameters to appear.

### Task 4: Full verification

**Files:**
- Verify all modified package files.

**Interfaces:**
- Consumes all prior tasks; produces the release evidence.

- [ ] **Step 1: Build and test**

Run `colcon build --packages-select hx_bringup_ego_mpc --symlink-install`, `colcon test`, and `colcon test-result --verbose`; expect zero failures.

- [ ] **Step 2: Numerical stress test**

Solve large position/velocity references from zero and nonzero previous acceleration. Report maximum speed, acceleration, horizontal acceleration, vertical acceleration, jerk, constraint violation, solve time, and iterations.

- [ ] **Step 3: Output-disabled smoke launch**

Start for six seconds with output, auto-arm, auto-offboard, and startup goal disabled. Confirm initialization, explicit disabled-output warning, correct watchdog not-ready behavior without inputs, and no residual processes.
