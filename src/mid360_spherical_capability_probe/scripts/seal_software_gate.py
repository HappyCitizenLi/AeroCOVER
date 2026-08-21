#!/usr/bin/env python3
"""Create a deterministic, fail-closed Step-16/Gate-3 software seal."""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from mid360_spherical_capability_probe.probe_core import CapabilityEvaluator


EXPECTED_LEAF_TESTS = {
    "livox_ros_driver2": 4,
    "mid360_spherical_capability_probe": 15,
    "mid360_ray_preprocessor": 19,
}
EXPECTED_FORK_HEAD = "6b9356cadf77084619ba406e6a0eb41163b08039"
EXPECTED_SDK_HEAD = "6a940156dd7151c3ab6a52442d86bc83613bd11b"
EXPECTED_GATE2_EVIDENCE_SHA256 = (
    "3673c825324ae77922fa01c143120896199ee2c765de9b8b3fe270d94e68696f"
)
ARTIFACT_NAMES = (
    "stage16_mid360_spherical_capability_unverified.json",
    "stage16_software_code_manifest.sha256",
    "stage16_gate3_software_evidence.json",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(path, *arguments):
    return subprocess.check_output(
        ["git", "-C", str(path)] + list(arguments), text=True
    ).rstrip("\r\n")


def git_head(path):
    return git_output(path, "rev-parse", "HEAD")


def git_status_paths(path):
    output = git_output(
        path, "status", "--porcelain=v1", "--untracked-files=all"
    )
    entries = []
    for line in output.splitlines():
        if len(line) < 4:
            raise ValueError("unparseable git status entry: %r" % line)
        status = line[:2]
        value = line[3:]
        if " -> " in value:
            raise ValueError("rename/copy status is unsupported by this seal")
        entries.append({"status": status, "path": value})
    return entries


def load_manifest(workspace, manifest_path):
    paths = []
    for raw_line in manifest_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        path = workspace / line
        if not path.is_file():
            raise FileNotFoundError("manifest path is missing: %s" % line)
        paths.append((line, path))
    relative_paths = [relative for relative, _ in paths]
    if len(relative_paths) != len(set(relative_paths)):
        raise ValueError("software manifest contains duplicate paths")
    return paths


def integer_attribute(element, name):
    raw_value = element.attrib.get(name, "0")
    try:
        return int(raw_value)
    except ValueError as exc:
        raise ValueError(
            "invalid xUnit %s=%r" % (name, raw_value)
        ) from exc


def parse_test_group(workspace, package, newest_test_source_mtime_ns):
    root = workspace / "build" / package / "test_results" / package
    xml_paths = sorted(root.glob("*.xml"))
    if not xml_paths:
        raise FileNotFoundError("no test XML for %s" % package)
    leaf_tests = 0
    failures = 0
    errors = 0
    skipped = 0
    root_declared_failures = 0
    root_declared_errors = 0
    root_declared_skipped = 0
    root_declared_disabled = 0
    files = []
    for path in xml_paths:
        document = ET.parse(str(path)).getroot()
        cases = list(document.iter("testcase"))
        leaf_tests += len(cases)
        failures += sum(1 for case in cases if case.find("failure") is not None)
        errors += sum(1 for case in cases if case.find("error") is not None)
        skipped += sum(
            1 for case in cases
            if case.find("skipped") is not None or case.find("skip") is not None
        )
        root_declared_failures += integer_attribute(document, "failures")
        root_declared_errors += integer_attribute(document, "errors")
        root_declared_skipped += integer_attribute(document, "skipped")
        root_declared_skipped += integer_attribute(document, "skip")
        root_declared_disabled += integer_attribute(document, "disabled")
        relative = str(path.relative_to(workspace))
        files.append({
            "path": relative,
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
            "mtime_ns": path.stat().st_mtime_ns,
            "newer_than_test_sources": (
                path.stat().st_mtime_ns >= newest_test_source_mtime_ns
            ),
            "leaf_testcases": len(cases),
        })
    expected = EXPECTED_LEAF_TESTS[package]
    passed = (
        leaf_tests == expected
        and failures == 0
        and errors == 0
        and skipped == 0
        and root_declared_failures == 0
        and root_declared_errors == 0
        and root_declared_skipped == 0
        and root_declared_disabled == 0
        and all(item["newer_than_test_sources"] for item in files)
    )
    return {
        "package": package,
        "expected_leaf_testcases": expected,
        "leaf_testcases": leaf_tests,
        "failures": failures,
        "errors": errors,
        "skipped": skipped,
        "root_declared_failures": root_declared_failures,
        "root_declared_errors": root_declared_errors,
        "root_declared_skipped": root_declared_skipped,
        "root_declared_disabled": root_declared_disabled,
        "newest_test_source_mtime_ns": newest_test_source_mtime_ns,
        "passed": passed,
        "files": files,
    }


def write_json(path, value):
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def validate_generated_at(value):
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("--generated-at must include an explicit timezone")


def newest_mtime(manifest_entries, predicate):
    selected = [
        path.stat().st_mtime_ns
        for relative, path in manifest_entries
        if predicate(relative)
    ]
    if not selected:
        raise ValueError("no source paths selected for freshness check")
    return max(selected)


def validate_gate2(workspace):
    path = workspace / "test_results" / "v2_correction_results.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    verification = report.get("verification", {})
    empty_world = verification.get("empty_world", {})
    single_wall = verification.get("single_wall", {})
    prompt_path = workspace / "mid360_vofod_tclv_codex_prompt_v2.md"
    valid = (
        sha256(path) == EXPECTED_GATE2_EVIDENCE_SHA256
        and
        report.get("prompt_comparison", {}).get("v2_sha256")
        == sha256(prompt_path)
        and verification.get("catkin_build", {}).get("result") == "PASS"
        and verification.get("xunit_summary")
        == "4 tests, 0 errors, 0 failures, 0 skipped"
        and empty_world.get("result") == "PASS"
        and empty_world.get("rays") == 820000
        and empty_world.get("csv_direction_checks") == 820000
        and single_wall.get("result") == "PASS"
        and single_wall.get("rays") == 820000
        and single_wall.get("csv_direction_checks") == 820000
    )
    return {
        "path": str(path.relative_to(workspace)),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
        "expected_sha256": EXPECTED_GATE2_EVIDENCE_SHA256,
        "validated": valid,
        "scope": "historical bounded Gate-2 sim_exact evidence; not rerun here",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace", default=str(Path(__file__).resolve().parents[3])
    )
    parser.add_argument("--output-dir", default="test_results")
    parser.add_argument(
        "--sdk-path",
        default="/home/uav/Livox-SDK2",
        help="Path to the pinned Livox-SDK2 checkout used for this build.",
    )
    parser.add_argument(
        "--generated-at",
        required=True,
        help="Fixed timezone-aware ISO-8601 timestamp for snapshot reproducibility.",
    )
    args = parser.parse_args()
    validate_generated_at(args.generated_at)

    workspace = Path(args.workspace).resolve()
    output_dir = Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = workspace / output_dir
    output_dir = output_dir.resolve()
    try:
        output_dir.relative_to(workspace)
    except ValueError as exc:
        raise ValueError("--output-dir must be inside --workspace") from exc
    output_dir.mkdir(parents=True, exist_ok=True)

    package_root = workspace / "src" / "mid360_spherical_capability_probe"
    manifest_source = package_root / "config" / "software_gate_manifest.txt"
    manifest_entries = load_manifest(workspace, manifest_source)
    manifest_relative_paths = {relative for relative, _ in manifest_entries}

    driver_root = workspace / "src" / "livox_ros_driver2_rayfork"
    sdk_root = Path(args.sdk_path).resolve()
    fork_head = git_head(driver_root)
    sdk_head = git_head(sdk_root)
    fork_overlay = git_status_paths(driver_root)
    sdk_status = git_status_paths(sdk_root)
    uncovered_overlay = []
    for entry in fork_overlay:
        relative = "src/livox_ros_driver2_rayfork/" + entry["path"]
        if relative not in manifest_relative_paths:
            uncovered_overlay.append(relative)

    driver_test_mtime = newest_mtime(
        manifest_entries,
        lambda relative: relative.startswith(
            "src/livox_ros_driver2_rayfork/"
        ) and not relative.endswith(("README.md", "UPSTREAM.md"))
        or relative.startswith("src/mid360_ray_msgs/"),
    )
    capability_test_mtime = newest_mtime(
        manifest_entries,
        lambda relative: relative.startswith(
            "src/mid360_spherical_capability_probe/"
        ) and not relative.endswith((
            "README.md", "seal_software_gate.py",
            "software_gate_manifest.txt",
        )),
    )
    preprocessor_test_mtime = newest_mtime(
        manifest_entries,
        lambda relative: (
            relative.startswith("src/mid360_ray_preprocessor/")
            and not relative.endswith("README.md")
        ) or relative.startswith("src/mid360_ray_msgs/"),
    )
    test_source_mtimes = {
        "livox_ros_driver2": driver_test_mtime,
        "mid360_spherical_capability_probe": capability_test_mtime,
        "mid360_ray_preprocessor": preprocessor_test_mtime,
    }
    test_groups = [
        parse_test_group(workspace, package, test_source_mtimes[package])
        for package in EXPECTED_LEAF_TESTS
    ]

    binaries = {
        "livox_ros_driver2_node": (
            workspace / "devel/.private/livox_ros_driver2/lib/"
            "livox_ros_driver2/livox_ros_driver2_node"
        ),
        "mid360_ray_preprocessor_node": (
            workspace / "devel/.private/mid360_ray_preprocessor/lib/"
            "mid360_ray_preprocessor/mid360_ray_preprocessor_node"
        ),
        "capability_probe": (
            workspace / "devel/.private/mid360_spherical_capability_probe/lib/"
            "mid360_spherical_capability_probe/"
            "mid360_spherical_capability_probe"
        ),
    }
    driver_binary_source_mtime = newest_mtime(
        manifest_entries,
        lambda relative: relative == "src/livox_ros_driver2_rayfork/CMakeLists.txt"
        or relative.endswith((".cpp", ".h"))
        and relative.startswith("src/livox_ros_driver2_rayfork/")
        or relative.startswith("src/mid360_ray_msgs/msg/"),
    )
    preprocessor_binary_source_mtime = newest_mtime(
        manifest_entries,
        lambda relative: (
            relative.startswith("src/mid360_ray_preprocessor/")
            and (
                relative.endswith((".cpp", ".hpp"))
                or relative.endswith("CMakeLists.txt")
            )
        ) or relative.startswith("src/mid360_ray_msgs/msg/"),
    )
    capability_runtime_sources = [
        package_root / "python/mid360_spherical_capability_probe/probe_core.py",
        package_root / "scripts/mid360_spherical_capability_probe",
    ]
    binary_requirements = {
        "livox_ros_driver2_node": driver_binary_source_mtime,
        "mid360_ray_preprocessor_node": preprocessor_binary_source_mtime,
        "capability_probe": max(
            path.stat().st_mtime_ns for path in capability_runtime_sources
        ),
    }
    binary_evidence = {}
    for name, path in binaries.items():
        exists = path.is_file()
        executable = exists and path.stat().st_mode & 0o111 != 0
        if name == "capability_probe":
            wrapper_text = (
                path.read_text(encoding="utf-8") if exists else ""
            )
            current = (
                exists
                and str(capability_runtime_sources[1]) in wrapper_text
                and "exec(compile(" in wrapper_text
            )
            freshness_semantics = (
                "catkin relay executes the exact current source-script path"
            )
        else:
            current = (
                exists
                and path.stat().st_mtime_ns >= binary_requirements[name]
            )
            freshness_semantics = "binary mtime is not older than compiled sources"
        binary_evidence[name] = {
            "path": str(path.relative_to(workspace)),
            "exists": exists,
            "executable": executable,
            "sha256": sha256(path) if exists else None,
            "mtime_ns": path.stat().st_mtime_ns if exists else None,
            "newest_runtime_source_mtime_ns": binary_requirements[name],
            "current_with_sources": current,
            "freshness_semantics": freshness_semantics,
        }

    cache_path = workspace / "build/livox_ros_driver2/CMakeCache.txt"
    link_path = workspace / (
        "build/livox_ros_driver2/CMakeFiles/"
        "livox_ros_driver2_node.dir/link.txt"
    )
    cache_text = cache_path.read_text(encoding="utf-8")
    link_text = link_path.read_text(encoding="utf-8")
    match = re.search(
        r"^LIVOX_LIDAR_SDK_LIBRARY:FILEPATH=(.+)$",
        cache_text,
        flags=re.MULTILINE,
    )
    if match is None:
        raise RuntimeError("Livox SDK library path missing from CMake cache")
    installed_sdk_library = Path(match.group(1)).resolve()
    checkout_sdk_library = sdk_root / "build/sdk_core/liblivox_lidar_sdk_static.a"
    installed_sdk_header = Path("/usr/local/include/livox_lidar_api.h")
    checkout_sdk_header = sdk_root / "include/livox_lidar_api.h"
    sdk_files = (
        installed_sdk_library,
        checkout_sdk_library,
        installed_sdk_header,
        checkout_sdk_header,
    )
    if not all(path.is_file() for path in sdk_files):
        raise FileNotFoundError("one or more pinned/installed SDK files are missing")
    sdk_binding_passed = (
        str(installed_sdk_library) in link_text
        and sha256(installed_sdk_library) == sha256(checkout_sdk_library)
        and sha256(installed_sdk_header) == sha256(checkout_sdk_header)
    )
    sdk_binding = {
        "passed": sdk_binding_passed,
        "evidence_scope": (
            "installed SDK files equal the checkout build-artifact snapshot; "
            "this seal does not independently rebuild SDK2 from its commit"
        ),
        "cmake_cache": {
            "path": str(cache_path.relative_to(workspace)),
            "sha256": sha256(cache_path),
        },
        "link_command": {
            "path": str(link_path.relative_to(workspace)),
            "sha256": sha256(link_path),
            "contains_installed_sdk_library": (
                str(installed_sdk_library) in link_text
            ),
        },
        "installed_library": {
            "path": str(installed_sdk_library),
            "sha256": sha256(installed_sdk_library),
        },
        "checkout_library": {
            "path": str(checkout_sdk_library),
            "sha256": sha256(checkout_sdk_library),
        },
        "installed_api_header": {
            "path": str(installed_sdk_header),
            "sha256": sha256(installed_sdk_header),
        },
        "checkout_api_header": {
            "path": str(checkout_sdk_header),
            "sha256": sha256(checkout_sdk_header),
        },
    }

    gate2 = validate_gate2(workspace)
    capability = CapabilityEvaluator().report()
    capability["runtime_revoked"] = None
    capability["runtime_status"] = "NOT_APPLICABLE_NO_PHYSICAL_RUN"
    capability["hardware_presence"] = "UNVERIFIED_NO_PHYSICAL_DEVICE"

    software_passed = (
        all(group["passed"] for group in test_groups)
        and fork_head == EXPECTED_FORK_HEAD
        and sdk_head == EXPECTED_SDK_HEAD
        and not sdk_status
        and not uncovered_overlay
        and sdk_binding_passed
        and all(
            item["exists"]
            and item["executable"]
            and item["current_with_sources"]
            for item in binary_evidence.values()
        )
        and gate2["validated"]
        and capability["ray_source_mode"] == "calibrated_fallback"
        and not capability["exact_no_return_direction_supported"]
    )
    if not software_passed:
        raise RuntimeError("Step-16 software evidence did not satisfy its seal")

    with tempfile.TemporaryDirectory(
        prefix=".stage16_seal_", dir=str(output_dir.parent)
    ) as temporary_directory:
        staging = Path(temporary_directory)
        capability_path = staging / ARTIFACT_NAMES[0]
        manifest_path = staging / ARTIFACT_NAMES[1]
        evidence_path = staging / ARTIFACT_NAMES[2]
        write_json(capability_path, capability)
        manifest_path.write_text(
            "".join(
                "%s  %s\n" % (sha256(path), relative)
                for relative, path in manifest_entries
            ),
            encoding="utf-8",
        )

        canonical_capability = output_dir / ARTIFACT_NAMES[0]
        canonical_manifest = output_dir / ARTIFACT_NAMES[1]
        evidence = {
            "schema_version": "tclv-step16-gate3-software-seal-v2",
            "generated_at": args.generated_at,
            "status": "SEALED_GATE3_INTERFACE_SOFTWARE_PASS_PHYSICAL_NOT_RUN",
            "step16_status": "PARTIAL",
            "gate3_interface_software": "PASS",
            "phase_d_physical_test": "NOT_RUN_NO_DEVICE",
            "physical_mid360_tested": False,
            "physical_firmware": "UNAVAILABLE",
            "hw_spherical_exact_claim": False,
            "ray_source_mode": "calibrated_fallback",
            "calibrated_fallback_generator_implemented": False,
            "sim_exact_status": "PASS_BOUND_HISTORICAL_GATE2_EVIDENCE",
            "sim_exact_evidence": gate2,
            "upstream": {
                "livox_ros_driver2_upstream_base_head": fork_head,
                "livox_ros_driver2_expected_base_head": EXPECTED_FORK_HEAD,
                "livox_ros_driver2_controlled_overlay_dirty": bool(fork_overlay),
                "livox_ros_driver2_overlay_status": fork_overlay,
                "overlay_paths_missing_from_manifest": uncovered_overlay,
                "livox_sdk2_head": sdk_head,
                "livox_sdk2_expected_head": EXPECTED_SDK_HEAD,
                "livox_sdk2_worktree_clean": not sdk_status,
            },
            "livox_sdk_build_binding": sdk_binding,
            "binaries": binary_evidence,
            "tests": test_groups,
            "artifacts": {
                "capability_report": {
                    "path": str(canonical_capability.relative_to(workspace)),
                    "bytes": capability_path.stat().st_size,
                    "sha256": sha256(capability_path),
                },
                "code_manifest": {
                    "path": str(canonical_manifest.relative_to(workspace)),
                    "entries": len(manifest_entries),
                    "bytes": manifest_path.stat().st_size,
                    "sha256": sha256(manifest_path),
                },
            },
            "snapshot_reproducibility_scope": (
                "byte-reproducible for identical source, binaries, xUnit XML, "
                "SDK files, and fixed generated_at; not a cross-rebuild claim"
            ),
            "allowed_claims": [
                "The controlled ROS1 interface adapter compiles in this workspace.",
                "Synthetic conversion and fail-closed decision contracts pass.",
                "Gate 3 interface-software acceptance is bounded PASS.",
            ],
            "forbidden_claims": [
                "A physical Mid-360 or any physical firmware was validated.",
                "hw_spherical_exact is available on real hardware.",
                "calibrated_fallback statistical coverage is implemented.",
                "line plus timestamp or simulation CSV reconstructs exact hardware rays.",
                "No-sync host receipt time is exact device or motion-compensation time.",
            ],
        }
        write_json(evidence_path, evidence)

        # Dependencies are replaced first and the self-describing evidence last.
        # A consumer must verify the hashes recorded by that final file.
        for artifact_name in ARTIFACT_NAMES:
            os.replace(staging / artifact_name, output_dir / artifact_name)

    print(json.dumps({
        "status": evidence["status"],
        "capability_report": str(output_dir / ARTIFACT_NAMES[0]),
        "code_manifest": str(output_dir / ARTIFACT_NAMES[1]),
        "evidence": str(output_dir / ARTIFACT_NAMES[2]),
        "leaf_testcases": {
            group["package"]: group["leaf_testcases"]
            for group in test_groups
        },
    }, sort_keys=True))


if __name__ == "__main__":
    main()
