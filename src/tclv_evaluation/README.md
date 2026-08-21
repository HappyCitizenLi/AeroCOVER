# tclv_evaluation

This Phase 1 package retains the general visibility-truth geometry ABI and the
single frozen B0 algorithm launcher. Truth code is evaluation-only and is not
started or subscribed by B0.

Canonical algorithm entry:

```bash
roslaunch tclv_evaluation b0_canonical.launch
```

It starts only `vofod_mid360` and `lidar_tracker_mid360`, consumes
`points_world + rays_checked`, enables tracker background filtering, and maps
only `vofod_mid360/background_points` as occupied background. Sensor, TF,
clock, scenario, bag replay, and truth must be provided by the caller.

The former `b0_evaluation.launch`/standalone adapter was removed so there is no
second production B0 definition. Benchmark runner and SOFT-VoFOD metrics belong
to later phases.
