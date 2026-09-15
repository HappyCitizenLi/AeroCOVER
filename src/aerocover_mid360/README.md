# AeroCOVER-Mid360

Current causal AeroCOVER implementation for OPEN, MT, OFFICE and FOREST,
with Mid-360 and OS1-128 sensor configurations. Current evidence is indexed in
`results/retained_experiments`; A1–A5 are mechanism regression/ablation configs,
not a retained six-scene experiment matrix.

Mid-360 keeps one second of historical return points and checked rays. The
point-level history/connectivity/background steps live in the standalone
`aerocover_st_background` library. Each scan:

1. expires old point/ray blocks;
2. clusters current and historical returns with exact 0.30 m connectivity;
3. rejects components containing a per-scan temporal slice wider than 1.5 m
   or enough propagated background points;
4. evaluates the current component slice against a 42-direction free-space
   shell built only from historical full-chord rays;
5. births a track after three strict 42/42 passes in five scans;
6. maintains an existing track with at least 29 bins only within 1.5 m of its
   9-state CA prediction;
7. applies the same 1.5 m innovation cap to strict observations after a track
   missed the preceding scan;
8. rejects any unassigned strict observation within 1.5 m of an already-born
   track prediction before creating a birth candidate;
9. prepares the current ray block/index concurrently with point clustering,
   then commits it only after the decision.

The package has no core evidence, PlanePatch, external tracker, dormant
re-identification, persistent map, rangefinder seed, or truth input.

Inputs:

```text
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

`aerocover_os1.launch` instead consumes `/uav1/ouster/points_world` and
`/uav1/ouster/rays_checked`. Its 1024×128 OS1 configuration requires all 131,072
rays per snapshot and does not subsample. The ray FIFO and point history both
remain `0.50 s`; point input is bounded to 42 m without shortening ray evidence. OS1 uses one connectivity worker after 1/2/4/8-worker
measurements found synchronization overhead, while independent shell
observations and per-ray DDA cell generation retain eight workers. Union, spatial
index merge, ray commit, state updates and output ordering remain serial and
deterministic. The current ray block stays private while it is prepared, so it
cannot support its own scan's shell decision. The production index is DDA-based; no angular-index path is enabled.
One expired ray block is retained for allocation reuse, outside the causal FIFO;
preparation clears old ray IDs and reuses ray/index vector capacities. Stale
cell keys are periodically released when they exceed twice the live cell count
plus 1024. This reduces expiry/allocation latency at the cost of higher RSS;
it is not a fixed-byte memory budget.
Shell queries share a query workspace across historical ray blocks. It caches
only the sphere's geometric cell cover, keyed by the exact center, radius and
grid cell size; each block still supplies its own ray IDs with the original
generation-based deduplication and sorted processing order. The parallel path
uses its existing worker-local workspace; the serial path uses a thread-local
one. Rebuilding invalidates the cached key until the cover is complete.

The five retained core ablations are small YAML overlays: current-frame-only
ST (A1), whole-history extent (A2), no shell (A3), no full-chord requirement
(A4), and one-shot birth (A5).

Outputs:

```text
/aerocover/tracks
/aerocover/diagnostics
/aerocover/background_points
/aerocover/residual_points
/aerocover/markers
```

Build and test:

```bash
source /opt/ros/noetic/setup.bash
catkin build aerocover_mid360 --no-deps
catkin run_tests aerocover_mid360 --no-deps
```
