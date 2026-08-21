# soft_vofod_mid360

Minimal final A3 implementation of SOFT-VoFOD. It consumes only the common,
truth-free Phase 1 contract:

```text
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

Run the algorithm after starting a source and `mid360_ray_preprocessor`:

```bash
roslaunch soft_vofod_mid360 soft_vofod.launch
```

The node publishes private topics under `/soft_vofod`: `events`, `tracks`,
`background_voxels`, `free_voxels`, `candidate_background_voxels`,
`opportunity_debug`, and `diagnostics`. It never subscribes to truth.

