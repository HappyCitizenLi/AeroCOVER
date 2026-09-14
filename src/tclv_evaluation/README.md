# tclv_evaluation

This package keeps the visibility-truth geometry ABI and thin compositions of
VoFOD with the pinned external tracker. Truth is evaluation-only and is never
an algorithm input.

`b0_canonical.launch` defaults to VoFOD-Original. The globally frozen sparse
Mid-360 adaptation is selected with:

```bash
roslaunch tclv_evaluation b0_canonical.launch \
  b0_config:=$(rospack find vofod_mid360)/config/b0_mid360_adapted.yaml \
  method_config:=$(rospack find vofod_mid360)/config/vofod_mid360_adapted.yaml \
  tracker_radius_min:=0.6
```

`ouster_original.launch` exposes the same three arguments and adds the full
Ouster snapshot adapter. Both pipelines consume checked rays, use only the
native rangefinder seed, and retain the `upstream_native` map execution path.
