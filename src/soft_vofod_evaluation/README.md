# AeroCOVER / VoFOD paper evaluation

The current matrix has 70 runs: two methods on eight Mid-360 scenes, three on
eight Ouster scenes, and five ablations on the six-scene Mid-360 subset.
Sensor-qualified run names are used where required.

| suite | scenes | methods |
|---|---|---|
| `ouster-main` | all eight | `AeroCOVER-OS1`, `VoFOD-Original-OS1`, `VoFOD-Mid360-Adapted-OS1` |
| `mid360-baselines` | all eight | `AeroCOVER-Mid360`, `VoFOD-Mid360-Adapted` |
| `mid360-core` | `P01`, `S2_new`, `P02`, `S3_new`, `M1`, `M2` | Full and A1–A5 |

The adapted VoFOD configuration is selected once on P01/P02 and then frozen;
it has no scene-specific overrides. The same frozen Mid-360 adaptation is
replayed on Ouster as a control, not retuned for Ouster. Ouster source
generation uses Gazebo 11's three-camera 360-degree GPU ray path; all eight
sources are complete.

```bash
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite ouster-main
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite mid360-baselines
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite mid360-core
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite summarize
```

Each source bag is reused across every method for that sensor. The evaluator
uses official TrackEval HOTA with 2 m linear position similarity and 19
thresholds; separate 1 m counts use maximum-cardinality, minimum-distance
one-to-one assignment. Manifests retain source/config/code hashes, replay rate,
timestamps, stable IDs and initialization state.
