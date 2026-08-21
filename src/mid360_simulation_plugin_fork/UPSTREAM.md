# Controlled Mid-360 plugin overlay

This directory is a source snapshot extracted from:

```text
repository: https://github.com/ctu-mrs/Mid360_simulation_plugin.git
branch at audit time: refactoring
commit: dca0420fee13e4dda40f8a609351f64da0a2b7ed
commit date: 2024-08-27T16:07:08+02:00
```

Only the Mid-360 package source and the assets needed by its existing standalone
test and the new RayBundle tests were copied. The upstream Git metadata was not
copied; changes are tracked by the parent `/home/uav/lyk` repository.

The source workspace at `/home/uav/mrs_mid360_ws` and packages installed under
`/opt/ros/noetic` are intentionally left unchanged.

Phase 1 keeps the legacy point-record ABI but gives cloud/RayBundle one source
stamp, publishes `mid360_ray_msgs/ScanIdentity`, and writes parallel PointXYZ
results at emitted-ray indices instead of nondeterministic append order.
