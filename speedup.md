# CUDA bunny + cloth optimization log

Target command:

```text
./build/linux_release_cuda/demo/demo -c misc/example_config/bunny_and_cloth.json --backend cuda --headless
```

Benchmark repetitions use the same command with `--bench`; convergence settings
are unchanged. Each entry reports wall time and, where available, the summed
ADMM work so that runs with different collision histories are not treated as
identical workloads.

## Reference measurements

| Version | Runs (s) | Median (s) | Speedup vs true baseline |
| --- | --- | ---: | ---: |
| True baseline, unmodified source | 9.50, 8.88, 9.56 | 9.50 | 1.00x |
| Retained optimization set, refreshed after rebuild | 6.17, 6.25, 6.79 | 6.25 | 1.52x |
| Retained optimization set, latest refresh | 6.08, 5.30, 6.46 | 6.08 | 1.56x |
| Retained optimization set, current OIBVH refresh | 6.28, 6.19, 6.14 | 6.19 | 1.54x |
| Retained optimization set, current OIBVH/O37 five-run refresh | 6.24, 5.66, 5.40, 5.94, 5.97 | 5.94 | 1.60x |
| Retained optimization set, current OIBVH/O37 five-run refresh (runs 2–6) | 5.82, 6.38, 5.69, 5.44, 5.81 | 5.81 | 1.64x |
| Retained optimization set, current OIBVH/O37 five-run refresh (latest) | 6.35, 5.93, 5.84, 6.08, 5.97 | 5.97 | 1.59x |
| Retained optimization set, current OIBVH/O37 five-run refresh (latest sequential reference) | 6.163, 6.527, 6.749, 6.607, 6.927 | 6.607 | 1.44x |
| Retained optimization set, current OIBVH/O37 five-run refresh (this turn) | 5.272, 6.028, 5.419, 5.534, 5.530 | 5.534 | 1.72x |
| Retained optimization set, current OIBVH/O37 five-run refresh (continuation) | 6.040, 6.220, 5.980, 6.630, 6.140 | 6.140 | 1.55x |
| Retained optimization set, current OIBVH/O37 five-run refresh (goal continuation) | 5.976, 5.728, 5.863, 5.934, 5.679 | 5.863 | 1.62x |
| Retained optimization set, matched OIBVH recheck | 5.91, 5.82, 6.10 | 5.91 | 1.61x |
| Retained optimization set plus O13, five-run current comparison | 6.09, 6.29, 6.28, 5.98, 6.03 | 6.09 | 1.56x |
| Retained optimization set plus O14, five-run current comparison | 5.95, 6.25, 5.98, 5.97, 5.75 | 5.97 | 1.59x |
| Retained OIBVH reference after O57 revert, five-run matched recheck | 5.516, 5.477, 6.322, 5.948, 5.584 | 5.584 | 1.70x |
| cuBQL, default build config, rebuild on every update, sorted traversal | 6.37, 6.89, 6.64 | 6.64 | 1.43x |

The first refreshed retained runs summed to 3778, 3966, and 4424 ADMM
iterations. The latest refresh summed to 3849, 3341, and 4093 iterations;
the variation is why medians and work counts are recorded together.

## Individual optimizations

Only optimizations retained in the current source are listed below.

| ID | Optimization | Incremental measurement |
| --- | --- | --- |
| O1 | Scalar-identity BSR representation for `weighted_AA` | Not independently meaningful: its runtime benefit is realized through O2 and O5; the representation-only construction cost is below the whole-scene timing resolution |
| O2 | Custom scalar-identity 3-coordinate SpMV with diagonal fusion | 6.25 s retained vs 7.38 s ablated; 1.18x incremental, 15.3% lower wall time |
| O3a | Fused PCG dot product and squared norm | 6.25 s retained vs 6.75 s ablated; 1.08x incremental, 7.4% lower wall time |
| O3b | Fused PCG alpha calculation into the x/r update | 6.25 s retained vs 6.79 s ablated; 1.09x incremental, 8.0% lower wall time |
| O5a | Scalar coarse matrices and scalar-identity coarse storage/assembly | 6.25 s retained vs 7.18 s generic-coarse ablated; 1.15x incremental, 13.0% lower wall time |
| O5b | Fused three-coordinate scalar coarse apply | 6.25 s retained vs 6.46 s three-coordinate ablated; 1.03x incremental, 3.3% lower wall time |

## cuBQL comparison

The standalone cuBQL/OIBVH cloth-ball query medians were:

```text
EE: 884.543 ms -> 363.427 ms
VF: 270.762 ms -> 168.127 ms
```

Those historical cuBQL numbers came from the standalone adapter and are not
used as the corrected production measurement. The production wrapper now
rebuilds with `radixBuilder` on every update and does not use refit.

The final cuBQL integration uses the library's default build configuration,
rebuilds with `radixBuilder` on every update, treats touching boxes as
colliding, and traverses self and tree-tree queries in BVH-sorted order. The
complete canonical EE and VF candidate sets matched OIBVH exactly for all 93
cloth-ball frame transitions. The full bunny-and-cloth runs summed to 3923,
4272, and 4149 ADMM iterations with 151, 171, and 165 CCD rollbacks. Their
6.64 s median is 0.93x incremental versus the 6.16 s OIBVH median (7.8%
slower). The cuBQL implementation and benchmark adapter remain available;
production broadphase has switched back to OIBVH.
