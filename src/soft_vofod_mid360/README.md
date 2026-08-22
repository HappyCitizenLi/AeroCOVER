# soft_vofod_mid360

SOFT-VoFOD V2/B4 implementation. It consumes only the common, truth-free
checked-input contract:

```text
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

Run the algorithm after starting a source and `mid360_ray_preprocessor`:

```bash
roslaunch soft_vofod_mid360 soft_vofod.launch
```

The canonical launch loads `config/soft_vofod_v2_canonical.yaml`. The node
publishes private topics under `/soft_vofod`: `events`, `tracks`,
`background_voxels`, `free_voxels`, `candidate_background_voxels`,
`opportunity_debug`, and `diagnostics`. It never subscribes to truth.

Algorithm, calibration, experiment, ablation, and limitation documentation is
under `docs/SOFT_VOFOD_V2_*.md`. B1-B3 and legacy A1-A3 are benchmark
ablations/development history, not alternate production launch paths.
