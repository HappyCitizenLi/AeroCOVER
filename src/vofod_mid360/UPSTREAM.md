# Upstream provenance and redistribution boundary

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
copyright header, or complete MIT permission notice. Therefore this file does
not claim that public redistribution has been cleared, does not invent a
copyright year/holder, and does not treat the manifest declaration as a waiver
of third-party notice obligations. Private compatibility work may proceed with
this provenance record; public push, source archive, binary release, or PR must
first complete the review below.

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
| Mid-360 B0 port | complete | ExactTime checked rays + valid points; PointXYZI clustering; synchronous bounded valid/no-return soft updates; endpoint protection; status/mask filters; diagnostics/reset/query; Gate 5/6 tests |
| Strict original-style B0 restoration | complete | Removed altitude/AABB background shortcuts; restored historical-map close/far partition, both upstream background gates, OBB/BFS classification and frontier mutation, counted-voxel separated-background cleanup, upstream-style confidence/covariance, one shared free-ray coefficient and upstream production thresholds; added a switchable direction-only Mid-360 proxy for the upstream rangefinder seed |
| Non-baseline extensions | removed | Package now builds only the faithful standalone B0 path |
| Phase 1 canonical B0 | complete | Replaced the temporary nadir proxy with target-free timed warm-up plus map-maturity gates; consumes and verifies canonical `points_world`; no tracker feedback added |

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

The strict restoration keeps the algorithmic classifier at the pinned
upstream boundary while retaining these explicit Mid-360/robustness changes:

- actual checked per-ray origins/directions/status replace organized Ouster
  dimensions, LUT lookup and row mask;
- explicit NO_RETURN uses the configured reliable range and the same upstream
  ray coefficient as VALID_RETURN;
- robust bounded DDA, endpoint protection and deterministic synchronous commit
  replace the unsafe detached execution without adding target/scene priors;
- the temporary direction-only `nadir_seed` used during strict restoration was
  removed in Phase 1; canonical startup now uses target-free sensor warm-up and
  the existing map-maturity gates without endpoint direction/height rules;
- configured constant detection probability replaces the fixed-organized-scan
  angular-bin formula and is not used as scan-opportunity state.

Apriori PCD loading remains unported. After reset the canonical source must
provide target-free warm-up until both unchanged background gates mature.
Generic empty/wall fixtures exercise the interface; scene-level evidence is
owned downstream.

Build and test commands are documented in `README.md`; generated result files
are intentionally not retained in the current workspace.

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
