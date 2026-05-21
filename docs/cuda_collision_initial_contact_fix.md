# CUDA Collision Initial Contact Fix

This note documents a CUDA collision bug seen in the bunny-and-cloth scene, why
the original line-search/barrier logic failed, and what changed to fix it.

The short version:

- The CUDA backend used coplanar CCD roots as the only source of collision
  constraints.
- That is not enough for finite-thickness contact with minimal separation
  `ms`.
- Slow obstacle penetration can happen when a primitive pair is already inside
  the `ms` band at the beginning of an outer iteration but does not produce a
  useful coplanar root.
- At later simulation times the solver also spent many outer iterations on tiny
  CCD rollback steps.
- The fix separates line-search impacts from initial proximity contacts:
  initial contacts become barrier constraints, while only future impacts
  participate in min-TOI rollback.

## Reproduction

Use the dense bunny-and-cloth config:

```bash
cmake --build ./build/linux_profile
timeout 90s ./build/linux_profile/demo/demo -c bunny_and_cloth.json --headless --backend cuda
```

The visible symptom was the bunny ear tip slowly penetrating the cloth in the
CUDA backend while the CPU backend looked acceptable for the same scene. The
late-stage CUDA log also showed many tiny rollback steps:

```text
collision stats line=114 initial=0 earliest toi 0.0125
CCD rollback to toi 0.0125
Outer iter 65
collision stats line=104 initial=1 earliest toi 0.0125
solve 1 initial contacts at current outer state
Outer iter 66
collision stats line=99 initial=1 earliest toi 0.025
CCD rollback to toi 0.025
...
Outer iter 89
collision stats line=98 initial=0 earliest toi 0.0125
earliest toi  0.0125 >= remaining step 5.038455e-07. terminate outer loop.
```

This could require 80+ outer iterations in a single time step.

## Zero TOI Root Data

The first failure mode was a `Zero toi` abort. A diagnostic run printed this
point-triangle event:

```text
zero pt root=0
poly=(5.25902e-12,4.45074e-09,7.00791e-07,-9.57536e-09)
eval0=-9.57536e-09
dist=9.79052e-06
ms=0.00143329
vn=0.000716644
idx=(1698,5955,5950,5943)
```

The cubic was:

```text
f(t) = 5.25902e-12 t^3
     + 4.45074e-09 t^2
     + 7.00791e-07 t
     - 9.57536e-09
```

Solving this polynomial gives a real positive root near:

```text
t = 0.01366246
```

The CUDA root finder used only six bisection iterations and returned the left
endpoint of the bracket. Because `0.01366 < 1 / 64`, the final left endpoint was
`0`. This was not a true zero-time collision; it was a quantized positive root.

Treating such roots as line-search rollback targets is wrong because the
outer-loop update multiplies min TOI by `0.8`. A returned `0` stays `0`, so the
outer loop can make no progress.

## Why The Original CUDA Barrier Was Insufficient

The original CUDA narrowphase did this:

1. Solve the coplanarity cubic for PT/EE.
2. For each root, compute primitive distance.
3. Accept the root if `dist <= ms`.
4. Build a collision response target from reflected velocity.

There are two problems.

First, coplanarity is a zero-thickness event surface. The finite-thickness
contact condition is `distance == ms`, not `coplanar == true`. The code used
`ms` only as a filter after root finding:

```cpp
if (dist2 > ms * ms) {
  continue;
}
```

This does not find the first time the pair enters the `ms` band. A pair can be
inside `ms` at the current outer state and still have no useful coplanar root in
the step. In that case no barrier constraint was generated.

Second, the CUDA barrier target did not enforce geometric separation. It was a
per-DOF reflected-position target:

```cpp
target = pos + (1 - toi) * vel_after;
```

`minimal_separation` was stored in `Collision`, but `compute_barrier()` did not
read it. This target can separate many cases, but it does not guarantee final
primitive distance `>= ms`.

## Paper Connection

The relevant paper is:

```text
paper/Lan et al. - 2022 - Penetration-free projective dynamics on the GPU.pdf
```

The important observation is in Section 4:

- CCD pruning alone can create sticking and oscillating outer iterations.
- Barrier projection must be generated before the next inner solve using an
  active contact set.
- Excessive pruning changes the optimization target too frequently and slows or
  destabilizes convergence.

The CUDA failure matched that description. The solver kept alternating between
small CCD rollbacks and weak/incomplete barrier updates.

## Implemented Fix

The fix has four parts.

### 1. Mark Initial Contacts

`Collision` now has:

```cpp
bool is_initial_contact;
```

The main loop treats these differently from future line-search collisions.

Initial contacts are excluded from min-TOI reduction. A zero or near-zero root
therefore cannot force a zero rollback.

### 2. Emit Proximity Contacts At `t0`

Before processing coplanar roots, CUDA PT/EE narrowphase now checks whether the
candidate pair is already within `ms` at the current outer state:

```text
distance(t0) <= ms
```

If yes, the pair is emitted as an initial contact with `toi = 0`. This catches
slowly developing finite-thickness contact that the coplanar-root path can miss.

The current implementation reuses the existing narrowphase feature tests:

- PT contact uses the point-triangle projection accepted by `exact_pt_uv`.
- EE contact uses closest points from `exact_ee_uv`.

For a triangle mesh, PT plus EE covers the relevant primitive contact cases in
the existing pipeline.

### 3. Use Geometric Projection For Initial Contacts

Future impacts still use reflected velocity targets, matching the existing
collision response behavior.

Initial contacts use a geometric correction instead:

```text
gap = ms - distance
correction = gap * normal
```

The correction is distributed to the four involved vertices by inverse-mass
weights. This gives the barrier target an actual `ms` separation meaning for
initial contacts.

This is intentionally used only for initial contacts. For future impacts, the
reflected target still carries the post-impact velocity response.

### 4. Accumulate Barrier Targets Per DOF

The old CUDA barrier assembly wrote a single `target` value per DOF:

```cpp
target[offset + i] = reflection(i);
```

That is a race when several contacts touch the same vertex, and the final target
depends on write order.

CUDA now accumulates target sums and counts with atomics, then normalizes:

```text
target[dof] = sum(targets for dof) / count(targets for dof)
```

This is closer to the CPU backend, which accumulates collision RHS/LHS
contributions instead of overwriting them.

## Main Loop Policy

The main loop now separates collisions into:

- `initial`: already at/inside contact band at the current outer state.
- `line`: future impacts that can drive CCD rollback.

The policy is:

1. If initial contacts exist and have not yet been solved for this outer state,
   gather only initial-contact barriers and run the inner solver again.
2. Otherwise compute min TOI only over line-search collisions.
3. If min TOI is inside the remaining step, rollback by `0.8 * min_toi`.
4. Gather barrier constraints for the relevant line-search collision set.

The one-pass guard on initial contacts is necessary. Initial proximity detection
uses `outer_state` as `t0`. During the initial-contact barrier solve,
`outer_state` intentionally does not move, only `inner_state` is re-solved. If
the loop repeatedly re-detects initial contacts from the unchanged `outer_state`,
it can get stuck forever on the same active set.

This was confirmed experimentally:

```text
collision stats line=0 initial=3 earliest toi 1
solve 3 initial contacts at current outer state
collision stats line=0 initial=3 earliest toi 1
solve 3 initial contacts at current outer state
...
```

Without the guard, the run timed out.

## Experiments

All experiments used:

```bash
timeout 90s ./build/linux_profile/demo/demo -c bunny_and_cloth.json --headless --backend cuda
```

### Velocity-Separating Predicate

A near-zero separating predicate was tested first. It did not solve the problem.

The diagnostic zero-root event had positive normal relative velocity under the
current normal convention:

```text
vn = 0.000716644
```

So the pair was approaching, not separating. Filtering separating contacts could
not explain or fix the zero TOI.

### Initial Contacts Without Guard

Adding proximity contacts but always re-solving initial contacts caused a loop.
The loop kept rediscovering the same `t0` contacts:

```text
line=0 initial=3 earliest toi 1
solve 3 initial contacts at current outer state
```

This timed out.

### Geometric Initial Targets Without Guard

Changing initial contacts to geometric `ms - distance` targets still looped
without the one-pass active-set guard, for the same reason: detection looked at
the unchanged `outer_state`.

### Final Result

With proximity contacts, geometric initial targets, accumulated barrier targets,
and the one-pass active-set guard:

```text
Finish step 299. Current time 1.5s
Headless simulation exported to 'out.abc'
```

Summary from the final log:

```text
max_outer      = 3
avg_line       = 17.9859
max_line       = 106
avg_initial    = 6.5363
max_initial    = 46
```

There was no `Zero toi`, no exact `earliest toi 0`, and no timeout.

## Why This Fix Helps The Bunny Ear Case

The bunny ear penetration was slow. That is the kind of case where relying only
on coplanar CCD roots is fragile:

- If the cloth is already inside the finite `ms` band at the current outer
  state, there may be no useful future coplanar root to report.
- If no collision is reported, no barrier is generated.
- If no barrier is generated, the next ADMM solve can continue to drift through
  the obstacle.

The new proximity path reports these pairs as initial contacts, so they become
barrier constraints before the next solve.

## Remaining Limitations

This is not a full GPU IPC implementation.

Known limitations:

- The coplanar cubic root finder is still coarse. It no longer creates zero-TOI
  rollback, but future-impact TOIs are still quantized.
- Initial contact projection is a local target construction, not a global proof
  that all final primitive distances are `>= ms`.
- Barrier targets are averaged per DOF. This removes nondeterministic overwrite,
  but it is still an approximation when many contacts compete on one vertex.
- The active-set guard is tied to the current outer-loop structure. A cleaner
  implementation would make the inner solve operate on a fixed active set and
  evaluate new contacts only after accepting the current outer step, as in the
  nested-loop structure from Lan et al.
- A more complete GPU CCD would directly find the first time
  `distance(t) == ms`, for example using a conservative advancement style
  distance CCD, rather than using coplanarity roots plus proximity contacts.

## Files To Inspect

- `silk/src/backend/cuda/collision/ccd.cu`
  - PT/EE CCD and initial proximity contact construction.
- `silk/src/backend/cuda/collision/collision.cuh`
  - `Collision::is_initial_contact`.
- `silk/src/backend/cuda/main_loop.cu`
  - initial-contact vs line-search collision policy.
- `silk/src/backend/cuda/solver/barrier_constraints.cu`
  - target accumulation and barrier filtering.
- `silk/src/backend/cuda/solver/barrier_constraints.cuh`
  - `BarrierCollisionFilter`.

