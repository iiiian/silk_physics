# Convergence Conditions

This note records the practical convergence checks used by the CUDA ADMM cloth
solver. Silk is an animation physics engine, so these checks are intended to
stop when the substep is stable enough for motion, not to certify a highly
accurate optimization solve.

## ADMM residuals

For a constraint block with linearized constraint operator `S`, weighted by `W`,
the solver alternates between:

- main variable update: solve for `x`
- auxiliary projection/update: solve for `z`
- multiplier update: update `u`

The standard ADMM residuals are:

```text
primal residual: r = W(Sx - z)
dual residual:   s = rho S^T W^T W (z - z_old)
```

The primal residual measures direct constraint disagreement. The dual residual
measures how much the auxiliary variable update changes the stationarity
condition for the main solve.

## Scale-aware stopping

A raw residual norm is not enough because its magnitude depends on mesh size,
units, weighting, stiffness, and timestep. The stopping rule compares each
residual against an absolute floor plus a relative scale term:

```text
||r|| <= eps_primal
||s|| <= eps_dual
```

For the primal residual, the natural scale is the larger side of the constraint
equation:

```text
eps_primal =
    sqrt(primal_dof) * admm_abs_tol
  + admm_rel_tol * max(||W Sx||, ||W z||)
```

This prevents a large cloth or large coordinate scale from requiring an
unreasonably tiny absolute residual, while still allowing small systems to stop
through the absolute floor.

The full Boyd-style dual threshold uses a stationarity scale such as
`||S^T u||`. Computing that exactly requires extra reduction/scatter work. The
current hybrid check uses the main-system right-hand side as a cheaper scale
proxy. The right-hand side can be very large because it includes inertia terms,
so the proxy is capped by the larger of the initial dual residual and `1`:

```text
dual_scale = max(min(||rhs||, max(initial_dual_norm, 1)), 1)

eps_dual =
    sqrt(state_dof) * admm_abs_tol
  + admm_rel_tol * dual_scale
```

This is less exact than a full dual scale, but it avoids an additional global
operator application while still making the dual check less dependent on the
first observed residual and less permissive when inertia makes `||rhs||` huge.

## Adaptive linear tolerance

The main variable update is an inexact linear solve. For animation, the linear
solve only needs to be accurate enough that CG error does not visibly destabilize
the ADMM iteration. Solving much more accurately is usually wasted work because
ADMM itself is not being run to high numerical accuracy.

The CUDA loop therefore chooses the CG relative tolerance from the previous ADMM
residual progress:

```text
admm_ratio = max(
    primal_norm / max(initial_primal_norm, admm_abs_tol),
    dual_norm / max(initial_dual_norm, admm_abs_tol))

cg_rel_tol = clamp(
    linear_solver_adaptive_factor * admm_ratio,
    linear_solver_rel_tol_min,
    linear_solver_rel_tol_max)
```

The first main solve uses `linear_solver_rel_tol_max`. Later solves tighten as
ADMM residuals shrink. With the default `linear_solver_adaptive_factor = 1e-2`
and `admm_rel_tol = 1e-3`, the linear solve is driven toward about `1e-5` near
nonlinear convergence.

The CG solver still uses its existing residual update/check policy. This
adaptive rule only selects the tolerance passed into that solver. ADMM
termination does not require a final solve at `linear_solver_rel_tol_min`; if the
animation-oriented ADMM residual thresholds pass after a looser solve, the loop
may stop.

## Practical interpretation

The nonlinear loop terminates only when both checks pass:

```text
primal_norm <= eps_primal
dual_norm   <= eps_dual
```

If the primal residual stalls, constraints are not agreeing with their
projections. If the dual residual stalls, the auxiliary variables are still
moving enough to change the main solve. If both are below their thresholds, the
current ADMM subproblem is considered sufficiently converged for the
timestep/substep.
