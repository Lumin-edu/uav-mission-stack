# Constrained OSQP MPC Design

## Goal

Replace the unconstrained NumPy solve in `hx_bringup_ego_mpc` with an OSQP-backed sparse QP whose complete prediction horizon satisfies conservative hard velocity, acceleration, and jerk limits.

## Model and Objective

Keep the existing 3-D double-integrator state `x = [p_n, p_e, p_d, v_n, v_e, v_d]`, acceleration input `u = [a_n, a_e, a_d]`, prediction period, horizon, and quadratic tracking cost. The controller updates the linear term and constraint bounds from the current state, reference, and previous acceleration on every solve while reusing a pre-built sparse Hessian and constraint matrix.

## Hard Constraints

- Predicted velocity uses the L1 ball `|v_n| + |v_e| + |v_d| <= max_velocity`, a conservative inner approximation of the configured L2 speed limit.
- Every acceleration uses the L1 ball `|a_n| + |a_e| + |a_d| <= max_acceleration`.
- Horizontal acceleration uses a 16-sided polygon inscribed in the configured L2 circle.
- Vertical acceleration uses `|a_d| <= max_vertical_acceleration`.
- Jerk uses an L1 ball on every acceleration difference. The first difference uses `control_dt`; later differences use the MPC prediction `dt`.
- The existing first-command limiter remains as a defense-in-depth check, not as the primary constraint mechanism.

## Failure Behavior and Diagnostics

Only OSQP status `solved` is accepted. The configured solve-time limit must be strictly shorter than the outer-loop control period. Infeasible, inaccurate, timed-out, non-finite, or constraint-violating results raise a solver error. The ROS controller catches that error, clears the previous acceleration, and publishes a position hold. Solver status, runtime, iterations, residuals, objective, and maximum constraint violation are exposed through `/ego_mpc/solver_diagnostics`.

## Dependencies and Deployment

Runtime requires NumPy, SciPy, and OSQP. Missing OSQP is a startup error; there is no silent fallback to the old unconstrained solver. The launch file exposes OSQP tolerance, iteration, and time-limit parameters with conservative defaults.

## Verification

Tests must show the unconstrained implementation violating a future input constraint, then show all optimized inputs and predicted velocities satisfying their horizon-wide limits. Additional tests cover jerk from a nonzero previous command, infeasibility/error handling, and solver statistics. Package build, unit tests, syntax checks, launch parsing, and an output-disabled smoke launch remain required.

## Residual Boundary

The QP becomes constrained, but Python scheduling and OSQP are not hard real-time. High-dynamic flight still requires timing measurements on the actual flight computer and may justify a later C++/acados migration.
