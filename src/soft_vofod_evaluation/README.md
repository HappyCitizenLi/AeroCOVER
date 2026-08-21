# soft_vofod_evaluation

Minimal record-once/replay-many benchmark for B0/A1/A2/A3. Truth is read only
by the source recorder/evaluator and is never replayed into an algorithm run.
The executed S01-S07 matrix and its negative result are documented in
`docs/SOFT_VOFOD_EXPERIMENT_REPORT.md` at the workspace root.

```bash
python3 $(rospack find soft_vofod_evaluation)/scripts/run_benchmark.py \
  --algorithms B0,A1,A2 --scenes S01,S02,S03,S04,S05,S06,S07 \
  --noise N0 --seeds 1001 --record --replay --output artifacts

python3 $(rospack find soft_vofod_evaluation)/scripts/run_benchmark.py \
  --algorithms A3 --scenes S01,S02,S03,S04,S05,S06 \
  --noise N0 --seeds 1001 --replay --output artifacts
```

Each source and run has a hash manifest. `evaluate_bag.py` writes JSON, flat
CSV, and per-frame timing CSV. Evaluation fails below 95% source-truth, track,
or timing coverage. `metrics/summary.csv`, `aggregate.json`, and
`ablation_deltas.csv` are rebuilt after successful runs.

The executed development matrix is deterministic N0/seed1001. N1/N2 are
explicitly named simulation-noise modes, not measured Mid-360 noise; they and
the multi-seed/negative-control campaign remain unexecuted. S07/A3 requires
offline throttling and must record it explicitly, for example:

```bash
python3 $(rospack find soft_vofod_evaluation)/scripts/run_benchmark.py \
  --algorithms A3 --scenes S07 --noise N0 --seeds 1001 --replay \
  --replay-rate 0.1 --output artifacts --force
```
