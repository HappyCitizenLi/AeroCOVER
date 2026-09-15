# Upstream provenance and redistribution boundary

> Current entry points are VoFOD-Mid360 / VoFOD-OS1 with the four-scene
> configurations documented in README.md. The sections below retain the
> import/license and migration history, not executable old experiment recipes.
> Inactive PersistentStructure source and retired standalone adaptation
> configs have been removed; native-rangefinder contract fixtures remain in test/.

## Imported snapshot

```text
upstream_project: CTU-MRS VoFOD
canonical_repository: https://github.com/ctu-mrs/vofod.git
upstream_branch: master
upstream_commit: 7da9f33a878a586588f6a626b75cfeacac7824f7
upstream_tree: 6e9565b3cc839a414c803d2fcf1e51fd84ff07c5
nearest_tag_description: evaluation-for-paper-6-g7da9f33
commit_author: Matouš Vrba <vrbamato@fel.cvut.cz>
commit_author_date: 2026-06-30T15:07:33+02:00
commit_committer: GitHub <noreply@github.com>
commit_subject: Update README.md
upstream_package: vofod 1.0.0
import_date: 2026-07-17 Asia/Shanghai
local_directory: src/vofod_mid360
redistribution_status: pending_upstream_license_clarification
```

The audit checkout at `/tmp/vofod_upstream_audit` was a `blob:none` promisor
partial clone. All blobs required by the selected commit were present at import
time and `git fsck --full` passed. The selected tree has no submodules, gitlinks,
symlinks, `.gitattributes`, or Git LFS pointers.

The exact import was a positive whitelist archive:

```bash
git -C /tmp/vofod_upstream_audit archive \
  --format=tar \
  --prefix=vofod_mid360/ \
  7da9f33a878a586588f6a626b75cfeacac7824f7 \
  -- \
  CMakeLists.txt README.md package.xml nodelets.xml \
  config include launch msgs rviz src \
| tar -x -C /home/uav/lyk/src
```

The archive before extraction had SHA-256:

```text
f6cb33b1b1a199de4d96da34d21d707d6e890fb1b54dbdf9432b9836f25a0b8c
```

It imported 29 regular files. No nested `.git` directory was created.

## Deliberately excluded upstream paths

```text
.git/**
.gitignore
tmux/**
```

`.gitignore` was excluded because its broad `*.csv`, `*.pkl`, and `*.pdf`
patterns are unsuitable for an auditable sensor-pattern port. The old tmux
profile targets a different MRS/Ouster environment and is not a catkin build
input. It also contains the unknown-provenance file:

```text
tmux/simulation_sphere/cache/filtered.bag_mavros/imu/data.pkl
size:   30546342 bytes
sha256: 680a099c7a41a8f42456bfc95fd81b4eec86d13b74736087f331b150b8ddaae9
```

That pickle was not deserialized and must never be added to this port or a
release artifact.

## License evidence actually present

The upstream snapshot contains only this package-manifest declaration:

```xml
<license>MIT</license>
```

It contains no `LICENSE`, `COPYING`, `NOTICE`, SPDX header, source-file
copyright header, or complete MIT permission notice. A local `LICENSE` now
preserves the complete standard MIT permission and warranty terms, while
explicitly recording that the upstream holder/year were not supplied. Adding
that text does not cure the missing upstream copyright notice. Public push,
source archive, binary release, or PR must still complete the review below.

## Redistribution statement

The local compatibility fork may be used for private research under the
upstream package's MIT declaration. Public redistribution is **not declared
cleared** by this repository: confirm the upstream copyright notice and the
PCL-derived voxel-grid notices first. Any redistributed copy must retain the
local `LICENSE`, this provenance file, resolved third-party notices, and a
clear description of the Mid-360 adaptations. No endorsement by CTU-MRS or
the paper authors is implied.

## Third-party or derived-code review

| Upstream area | Evidence | Local disposition |
|---|---|---|
| `voxel_grid_weighted.cpp`, `voxel_grid_counted.cpp` | Control flow/comments appear derived from PCL `VoxelGrid`, but the copied files contain no PCL BSD header | Identify the exact PCL source/version and restore the corresponding BSD notice, or replace with a locally documented implementation before redistribution |
| tuple hash in `voxel_map.h` | Import comment attributed a generic tuple hash to “Matthieu M.” | Resolved locally: replaced during B0 with an explicit local linear-index map layout; the attributed tuple-hash block is absent |
| simulation LUT in `vofod_nodelet.cpp` | Import comment said it was copied from “the simulation plugin” without repository, commit, or license | Resolved locally: deleted with the organized-Ouster path; the active nodelet consumes explicit checked directions |
| DDA in `voxel_map.cpp` | Cites Amanatides and Woo, “A Fast Voxel Traversal Algorithm for Ray Tracing” | Preserve the algorithm citation and document whether the retained implementation is independent |
| disabled split-string block in `pc_loader.cpp` | Import comment cited `martinbroadhurst.com` | Resolved locally: dead example block removed |

The imported upstream baseline referenced ROS, PCL, OpenCV, Ouster ROS and MRS;
their source or binaries were not vendored by the archive. The active B0
`package.xml` no longer depends on OpenCV or Ouster ROS. It still depends on
ROS/PCL/MRS interfaces, and binary distribution requires an independent
dependency-license review.

## Local change log

At import time, the 29 whitelisted files were byte-for-byte archive contents.
`UPSTREAM.md` itself is a new local provenance file and was not in the upstream
tree. Subsequent functional changes must be recorded here by phase:

| Phase | Status | Scope |
|---|---|---|
| Whitelist import | complete | Exact commit, 29 files; excluded nested Git metadata, broad ignore file, tmux profile, and unknown pickle |
| Package isolation/rename | complete | Renamed catkin package to `vofod_mid360`, generated interfaces and plugin lookups; C++ algorithm namespace remains `vofod`. Added direct dependency declarations and a single well-formed nodelet-manifest root |
| Mid-360 B0 port | historical, superseded | ExactTime checked rays + valid points; PointXYZI clustering; the temporary synchronous update path was replaced by the later upstream-native execution phase |
| Strict original-style B0 restoration | historical, superseded | Restored historical-map close/far partition, upstream background gates, OBB/BFS classification, counted-voxel cleanup, confidence/covariance and production thresholds; its direction-only seed proxy was later removed |
| Non-baseline extensions | historical, superseded | The faithful common path remains; experimental background-source branches are no longer active |
| Phase 1 canonical B0 | superseded | The temporary target-free timed warm-up was removed before the RAL experiment because it was not upstream VoFOD initialization |
| Native range seed | complete | Restored validated `sensor_msgs/Range` seeding along sensor +x; Original keeps upstream readiness, while the globally frozen Mid-360 adaptation uses a `1e-4` XY occupancy ratio and one sure voxel |
| Shared point-background branch | retired | Removed from the node, configuration, launch path, dependencies, diagnostics and paper method list after its high-FP evaluation |
| Current benchmark volume | complete | Four-scene nominal volume `[-28,60] x [-24,54] x [-0.125,18] m`; 0.25 m voxels, ground centered in the first layer. Exact effective bounds are documented in the current report |
| Upstream-native map execution | complete | Restored point-update-before-classification, one asynchronous free-ray worker with upstream busy-skip/next-detection visibility, and an independent 0.1 s ROS-time separate-background cleanup |
| Fixed-lag PersistentInit | removed | No production consumer or build target remained; inactive source/header removed. Prior import and migration history remain in Git |

This is an independent compatibility fork. The local name and documentation
must not imply endorsement by CTU-MRS, the upstream authors, PCL, or other
third-party projects.

The isolation phase initially changed package/build/plugin identifiers. The
subsequent B0 phase then replaced the active sensor and mapping boundary:

- deleted the MaskCreator nodelet, dynamic-reconfigure interface, Ouster sensor
  configs, fixed image masks, metadata/LUT construction and inactive simulation
  LUT;
- converted the clustering filters to `pcl::PointXYZI`, added empty-input and
  original-index safety fixes, and removed Ouster point types;
- hardened `VoxelMap` layout validation, bounds handling and DDA face/corner
  traversal;
- added `RayUpdateCore`, `MapUpdateDiagnostics`, `QueryVoxels`, Mid-360
  configs/launch/RViz assets and five ROS contracts plus 20 named core tests;
- rewrote the active nodelet around one deterministic synchronous update and
  removed detached cross-frame map work.

The current four-scene VoFOD pipelines derive their classifier from the pinned
upstream version, with explicit local sensor and safety adaptations. This is
not a claim of byte-for-byte equivalence (notably, cleanup writeback deduplication
differs). The current paths retain:

- actual checked per-ray origins/directions/status replace organized Ouster
  dimensions, LUT lookup and row mask;
- explicit NO_RETURN uses the configured reliable range and the same upstream
  ray coefficient as VALID_RETURN;
- robust bounded DDA and endpoint protection remain; point updates occur in
  the scan callback, free-ray updates run in the restored asynchronous worker,
  and the original counted-voxel separate-background rule runs from its own
  0.1 s ROS-time timer;
- `native_rangefinder` uses the actual Gazebo ray rangefinder as the sole
  upstream seed source; both sensor paths share map execution and use the
  explicitly declared four-scene detector/readiness/tracker configurations;
- the temporary direction-only `nadir_seed`, timed warm-up and active
  persistent-structure certificate are absent;
- configured constant detection probability replaces the fixed-organized-scan
  angular-bin formula and is not used as scan-opportunity state.

Apriori PCD loading remains unported. After reset, the synchronized point/ray
pair and range topic are required.
Generic empty/wall fixtures exercise the interface; scene-level evidence is
owned downstream.

Build and test commands are documented in `README.md`; generated experiment
outputs are kept outside this vendored package under the repository-level
`results/` tree and are not part of the imported upstream snapshot.

## Required review before redistribution

1. Ask the upstream maintainer to confirm the complete MIT text, copyright
   notice, files covered, and contribution boundary.
2. Resolve or replace the PCL-derived voxel-grid files with the correct BSD
   notices.
3. Keep the resolved tuple-hash/LUT/dead-example removals in future rebases.
4. Generate a dependency license/SBOM record for the intended distribution.
5. Verify that no excluded pickle, tmux cache, bag-derived data, or nested Git
   object is present.
6. Update this change log and keep upstream/local code boundaries explicit.
