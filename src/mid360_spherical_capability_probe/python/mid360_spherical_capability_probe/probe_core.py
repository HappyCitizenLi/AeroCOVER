"""ROS-independent, fail-closed hardware spherical capability decision core."""

from collections import OrderedDict
from dataclasses import dataclass
import math


SCHEMA_VERSION = "mid360-spherical-capability-probe-v1"
UNAVAILABLE_FIRMWARE = frozenset(("", "UNAVAILABLE", "UNVERIFIED_NO_HARDWARE"))
UNAVAILABLE_IDENTITY = frozenset(("", "UNAVAILABLE", "UNKNOWN"))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() == "true"


def _as_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def driver_diagnostics_runtime_safe(
        values, maximum_alignment_mismatch_count=None,
        expected_serial_number=None, expected_firmware=None,
        expected_restart_marker=None):
    """Return true only for a currently aligned, confirmed Mid-360 stream."""
    if not isinstance(values, dict):
        return False
    alignment_count = _as_int(
        values.get("frame_alignment_mismatch_count"), -1
    )
    alignment_safe = (
        _as_bool(values.get("frame_alignment_exact"))
        if maximum_alignment_mismatch_count is None
        else 0 <= alignment_count <= maximum_alignment_mismatch_count
    )
    return (
        str(values.get("device", "")) == "Mid360"
        and str(values.get("serial_number", "")) not in UNAVAILABLE_IDENTITY
        and str(values.get("firmware", "")) not in UNAVAILABLE_FIRMWARE
        and str(values.get("device_restart_marker", ""))
        not in UNAVAILABLE_IDENTITY
        and _as_bool(values.get("pcl_data_type_3_accepted"))
        and _as_bool(values.get("spherical_packet_confirmed"))
        and _as_bool(values.get("timestamp_metadata_valid"))
        and _as_bool(values.get("timestamp_synchronized"))
        and _as_bool(values.get(
            "diagnostic_header_stamp_matches_frame_stamp"
        ))
        and _as_bool(values.get("diagnostic_ray_frame_metadata_matches"))
        and _as_bool(values.get("single_device_stream"))
        and (
            expected_serial_number is None
            or str(values.get("serial_number", "")) == expected_serial_number
        )
        and (
            expected_firmware is None
            or str(values.get("firmware", "")) == expected_firmware
        )
        and (
            expected_restart_marker is None
            or str(values.get("device_restart_marker", ""))
            == expected_restart_marker
        )
        and alignment_safe
    )


def runtime_promotion_failure_reason(
        values, now_monotonic, last_diagnostics_monotonic,
        last_ray_monotonic, last_paired_frame_monotonic,
        diagnostics_timeout_sec, ray_timeout_sec,
        maximum_alignment_mismatch_count=None,
        expected_serial_number=None, expected_firmware=None,
        expected_restart_marker=None):
    """Return the first fail-closed reason blocking exact-mode promotion."""
    if (
        last_diagnostics_monotonic is None
        or now_monotonic - last_diagnostics_monotonic
        > diagnostics_timeout_sec
    ):
        return "driver_diagnostics_timeout"
    if (
        last_ray_monotonic is None
        or now_monotonic - last_ray_monotonic > ray_timeout_sec
    ):
        return "ray_bundle_timeout"
    if (
        last_paired_frame_monotonic is None
        or now_monotonic - last_paired_frame_monotonic
        > min(diagnostics_timeout_sec, ray_timeout_sec)
    ):
        return "paired_driver_ray_frame_timeout"
    if not driver_diagnostics_runtime_safe(
        values,
        maximum_alignment_mismatch_count,
        expected_serial_number,
        expected_firmware,
        expected_restart_marker,
    ):
        return "runtime_driver_capability_revoked"
    return None


def diagnostic_ray_frame_metadata_matches(values, bundle):
    """Bind a diagnostic record to one exact RayBundle beyond its stamp."""
    if not isinstance(values, dict) or bundle is None:
        return False
    try:
        return (
            int(values.get("frame_scan_id", -1)) == int(bundle.scan_id)
            and int(values.get("frame_pattern_start_index", -1))
            == int(bundle.pattern_start_index)
            and int(values.get("frame_ray_count", -1)) == len(bundle.rays)
            and int(values.get("frame_stamp_ns", -1))
            == int(bundle.header.stamp.to_nsec())
        )
    except (AttributeError, TypeError, ValueError):
        return False


class DiagnosticRayFramePairer:
    """Pair diagnostics and RayBundle payloads by one exact frame stamp."""

    def __init__(self, capacity=32):
        if int(capacity) <= 0:
            raise ValueError("frame pairer capacity must be positive")
        self.capacity = int(capacity)
        self._diagnostics = OrderedDict()
        self._rays = OrderedDict()
        self._pending_order = OrderedDict()
        self._processed = OrderedDict()

    def add_diagnostics(self, stamp_ns, marker, values):
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            return None
        self._diagnostics[stamp_ns] = (values, str(marker))
        self._note_pending(stamp_ns)
        return self._take(stamp_ns)

    def add_ray_bundle(self, stamp_ns, bundle):
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            return None
        self._rays[stamp_ns] = bundle
        self._note_pending(stamp_ns)
        return self._take(stamp_ns)

    def clear_pending(self):
        """Drop every unpaired cross-topic payload at an epoch boundary."""
        self._diagnostics.clear()
        self._rays.clear()
        self._pending_order.clear()

    def _note_pending(self, stamp_ns):
        self._pending_order[stamp_ns] = None
        self._pending_order.move_to_end(stamp_ns)
        while len(self._pending_order) > self.capacity:
            expired_stamp, _ = self._pending_order.popitem(last=False)
            self._diagnostics.pop(expired_stamp, None)
            self._rays.pop(expired_stamp, None)

    def _take(self, stamp_ns):
        if stamp_ns not in self._diagnostics or stamp_ns not in self._rays:
            return None
        values, marker = self._diagnostics.pop(stamp_ns)
        bundle = self._rays.pop(stamp_ns)
        self._pending_order.pop(stamp_ns, None)
        processed_key = (marker, stamp_ns)
        if processed_key in self._processed:
            return None
        self._processed[processed_key] = None
        while len(self._processed) > self.capacity:
            self._processed.popitem(last=False)
        return {
            "stamp_ns": stamp_ns,
            "marker": marker,
            "diagnostics": values,
            "ray_bundle": bundle,
        }


@dataclass(frozen=True)
class ProbeThresholds:
    minimum_packet_count: int = 10
    minimum_sample_count: int = 1000
    minimum_zero_depth_count: int = 1
    minimum_nontrivial_angle_ratio: float = 0.9
    minimum_temporal_continuity_ratio: float = 0.8
    maximum_direction_norm_error: float = 1.0e-4
    maximum_neighbor_angle_rad: float = 0.08726646259971647
    maximum_neighbor_time_gap_ns: int = 1000000
    minimum_zero_valid_neighbor_pairs: int = 1
    minimum_zero_depth_direction_span_rad: float = 0.001
    required_restart_trials: int = 2

    def validate(self):
        if self.minimum_packet_count <= 0 or self.minimum_sample_count <= 0:
            raise ValueError("packet/sample thresholds must be positive")
        if self.minimum_zero_depth_count <= 0:
            raise ValueError("zero-depth threshold must be positive")
        if not 0.0 <= self.minimum_nontrivial_angle_ratio <= 1.0:
            raise ValueError("nontrivial-angle ratio threshold is invalid")
        if not 0.0 <= self.minimum_temporal_continuity_ratio <= 1.0:
            raise ValueError("continuity ratio threshold is invalid")
        if self.maximum_direction_norm_error <= 0.0:
            raise ValueError("direction norm tolerance must be positive")
        if not 0.0 < self.maximum_neighbor_angle_rad <= math.pi:
            raise ValueError("neighbor angle threshold is invalid")
        if self.maximum_neighbor_time_gap_ns <= 0:
            raise ValueError("neighbor time-gap threshold must be positive")
        if self.minimum_zero_valid_neighbor_pairs <= 0:
            raise ValueError("zero/valid neighbor threshold must be positive")
        if not 0.0 < self.minimum_zero_depth_direction_span_rad <= math.pi:
            raise ValueError("zero-depth direction span threshold is invalid")
        if self.required_restart_trials < 2:
            raise ValueError("at least two restart trials are required")


class RestartEpochGate:
    """Separate trial epochs so pre-restart samples can never leak forward."""

    START = "start"
    CONTINUE = "continue"
    RESET = "reset"
    IGNORE = "ignore"

    def __init__(self):
        self.active_marker = None
        self.required_change_from = None
        self.waiting_for_new_boot = False
        self.retired_markers = set()

    def observe(self, marker):
        if marker is None:
            return self.IGNORE
        marker = str(marker)
        if marker in UNAVAILABLE_IDENTITY:
            return self.IGNORE
        if marker in self.retired_markers:
            return self.IGNORE
        if self.waiting_for_new_boot:
            if marker == self.required_change_from:
                return self.IGNORE
            self.waiting_for_new_boot = False
            self.active_marker = marker
            return self.START
        if self.active_marker is None:
            self.active_marker = marker
            return self.START
        if marker != self.active_marker:
            self.active_marker = marker
            return self.RESET
        return self.CONTINUE

    def finish(self, marker):
        if marker is None:
            raise ValueError("cannot finish a trial without a restart marker")
        marker = str(marker)
        if marker in UNAVAILABLE_IDENTITY:
            raise ValueError("cannot finish a trial without a restart marker")
        self.required_change_from = marker
        self.retired_markers.add(marker)
        self.active_marker = None
        self.waiting_for_new_boot = True


class TrialAccumulator:
    def __init__(self, trial_id, thresholds):
        if not isinstance(trial_id, str) or not trial_id.strip():
            raise ValueError("trial_id must be non-empty")
        thresholds.validate()
        self.trial_id = trial_id.strip()
        self.thresholds = thresholds
        self.driver = {}
        self._driver_counter_baseline = None
        self._bound_restart_marker = None
        self._restart_marker_changed = False
        self._bound_device_identity = None
        self._device_identity_changed = False
        self.sample_count = 0
        self.zero_depth_count = 0
        self.zero_depth_with_finite_angles = 0
        self.zero_depth_with_nontrivial_angles = 0
        self.continuity_checked_count = 0
        self.continuity_pass_count = 0
        self.zero_depth_direction_span_rad = 0.0
        self._first_zero_depth_direction = None
        self._previous_sample = None

    def update_driver_diagnostics(self, values):
        if not isinstance(values, dict):
            raise ValueError("driver diagnostics must be a mapping")
        normalized = {str(key): str(value) for key, value in values.items()}
        if self._driver_counter_baseline is None:
            self._driver_counter_baseline = {
                "packet_count": _as_int(normalized.get("packet_count")),
                "spherical_packet_count": _as_int(
                    normalized.get("spherical_packet_count")
                ),
                "sample_count": _as_int(normalized.get("sample_count")),
                "frame_alignment_mismatch_count": _as_int(
                    normalized.get("frame_alignment_mismatch_count")
                ),
            }
        marker = normalized.get("device_restart_marker")
        if marker is not None and marker not in UNAVAILABLE_IDENTITY:
            if self._bound_restart_marker is None:
                self._bound_restart_marker = marker
            elif marker != self._bound_restart_marker:
                self._restart_marker_changed = True
        identity = (
            normalized.get("device"),
            normalized.get("serial_number"),
            normalized.get("firmware"),
        )
        if (
            identity[0] == "Mid360"
            and identity[1] not in UNAVAILABLE_IDENTITY
            and identity[2] not in UNAVAILABLE_FIRMWARE
        ):
            if self._bound_device_identity is None:
                self._bound_device_identity = identity
            elif identity != self._bound_device_identity:
                self._device_identity_changed = True
        self.driver.update(normalized)

    def observe_sample(self, return_status, direction, stamp_ns):
        if len(direction) != 3:
            raise ValueError("direction must have three components")
        vector = tuple(float(value) for value in direction)
        timestamp = int(stamp_ns)
        self.sample_count += 1
        is_zero_depth = int(return_status) == 0
        finite = all(math.isfinite(value) for value in vector)
        norm = math.sqrt(sum(value * value for value in vector)) if finite else math.nan
        unit = finite and abs(norm - 1.0) <= self.thresholds.maximum_direction_norm_error
        if is_zero_depth:
            self.zero_depth_count += 1
            if unit:
                self.zero_depth_with_finite_angles += 1
            if unit and (
                abs(vector[0]) > 1.0e-8
                or abs(vector[1]) > 1.0e-8
                or abs(vector[2] - 1.0) > 1.0e-8
            ):
                self.zero_depth_with_nontrivial_angles += 1
            if unit:
                if self._first_zero_depth_direction is None:
                    self._first_zero_depth_direction = vector
                else:
                    first_dot = max(-1.0, min(1.0, sum(
                        vector[index] * self._first_zero_depth_direction[index]
                        for index in range(3)
                    )))
                    self.zero_depth_direction_span_rad = max(
                        self.zero_depth_direction_span_rad,
                        math.acos(first_dot),
                    )
        current = (int(return_status), vector, timestamp, unit)
        if self._previous_sample is not None:
            previous_status, previous_vector, previous_stamp, previous_unit = (
                self._previous_sample
            )
            zero_valid_pair = (
                (previous_status == 0 and int(return_status) == 1)
                or (previous_status == 1 and int(return_status) == 0)
            )
            if zero_valid_pair and previous_unit and unit:
                self.continuity_checked_count += 1
                dot = max(-1.0, min(1.0, sum(
                    vector[index] * previous_vector[index]
                    for index in range(3)
                )))
                time_gap = timestamp - previous_stamp
                if (
                    0 < time_gap <=
                    self.thresholds.maximum_neighbor_time_gap_ns
                    and math.acos(dot)
                    <= self.thresholds.maximum_neighbor_angle_rad
                ):
                    self.continuity_pass_count += 1
        self._previous_sample = current

    def finish(self):
        baseline = self._driver_counter_baseline or {
            "packet_count": 0,
            "spherical_packet_count": 0,
            "sample_count": 0,
            "frame_alignment_mismatch_count": 0,
        }
        raw_packet_count = max(
            0,
            _as_int(self.driver.get("packet_count"))
            - baseline["packet_count"],
        )
        packet_count = max(
            0,
            _as_int(self.driver.get("spherical_packet_count"))
            - baseline["spherical_packet_count"],
        )
        driver_sample_count = max(
            0,
            _as_int(self.driver.get("sample_count"))
            - baseline["sample_count"],
        )
        sample_count = self.sample_count
        alignment_mismatch_count = max(
            0,
            _as_int(self.driver.get("frame_alignment_mismatch_count"))
            - baseline["frame_alignment_mismatch_count"],
        )
        nontrivial_ratio = (
            self.zero_depth_with_nontrivial_angles / self.zero_depth_count
            if self.zero_depth_count else 0.0
        )
        continuity_ratio = (
            self.continuity_pass_count / self.continuity_checked_count
            if self.continuity_checked_count else 0.0
        )
        reasons = []
        device = self.driver.get("device", "UNAVAILABLE")
        serial_number = self.driver.get("serial_number", "UNAVAILABLE")
        firmware = self.driver.get("firmware", "UNAVAILABLE")
        restart_marker = self.driver.get(
            "device_restart_marker", "UNAVAILABLE"
        )
        accepted = _as_bool(self.driver.get("pcl_data_type_3_accepted"))
        spherical = _as_bool(self.driver.get("spherical_packet_confirmed"))
        timestamp_metadata_valid = _as_bool(
            self.driver.get("timestamp_metadata_valid")
        )
        timestamp_synchronized = _as_bool(
            self.driver.get("timestamp_synchronized")
        )
        diagnostic_stamp_matches = _as_bool(self.driver.get(
            "diagnostic_header_stamp_matches_frame_stamp"
        ))
        diagnostic_ray_metadata_matches = _as_bool(
            self.driver.get("diagnostic_ray_frame_metadata_matches")
        )
        frame_aligned = alignment_mismatch_count == 0
        single_device_stream = _as_bool(
            self.driver.get("single_device_stream")
        )
        checks = (
            (device == "Mid360", "device_not_confirmed_mid360"),
            (serial_number not in UNAVAILABLE_IDENTITY,
             "device_serial_unavailable"),
            (firmware not in UNAVAILABLE_FIRMWARE, "firmware_unavailable"),
            (restart_marker not in UNAVAILABLE_IDENTITY,
             "device_restart_marker_unavailable"),
            (not self._restart_marker_changed,
             "device_restart_marker_changed_within_trial"),
            (not self._device_identity_changed,
             "device_identity_changed_within_trial"),
            (accepted, "pcl_data_type_3_not_accepted"),
            (spherical, "spherical_packet_not_confirmed"),
            (timestamp_metadata_valid, "timestamp_metadata_invalid"),
            (timestamp_synchronized, "packet_timestamp_not_synchronized"),
            (diagnostic_stamp_matches,
             "diagnostic_ray_frame_stamp_mismatch"),
            (diagnostic_ray_metadata_matches,
             "diagnostic_ray_frame_metadata_mismatch"),
            (frame_aligned, "ray_pointcloud_frame_alignment_failed"),
            (single_device_stream, "multiple_lidar_streams_detected"),
            (packet_count >= self.thresholds.minimum_packet_count,
             "insufficient_packet_count"),
            (sample_count >= self.thresholds.minimum_sample_count,
             "insufficient_sample_count"),
            (self.zero_depth_count >= self.thresholds.minimum_zero_depth_count,
             "no_or_insufficient_zero_depth_samples"),
            (self.zero_depth_with_finite_angles == self.zero_depth_count,
             "zero_depth_direction_not_finite_unit"),
            (nontrivial_ratio >= self.thresholds.minimum_nontrivial_angle_ratio,
             "zero_depth_angles_trivial"),
            (self.zero_depth_direction_span_rad >=
             self.thresholds.minimum_zero_depth_direction_span_rad,
             "zero_depth_angles_not_changing"),
            (self.continuity_checked_count >=
             self.thresholds.minimum_zero_valid_neighbor_pairs,
             "insufficient_adjacent_zero_valid_pairs"),
            (continuity_ratio >= self.thresholds.minimum_temporal_continuity_ratio,
             "angle_temporal_continuity_failed"),
        )
        for passed, reason in checks:
            if not passed:
                reasons.append(reason)
        return {
            "trial_id": self.trial_id,
            "device": device,
            "serial_number": serial_number,
            "firmware": firmware,
            "device_restart_marker": restart_marker,
            "pcl_data_type_3_accepted": accepted,
            "spherical_packet_confirmed": spherical,
            "timestamp_metadata_valid": timestamp_metadata_valid,
            "timestamp_synchronized": timestamp_synchronized,
            "diagnostic_header_stamp_matches_frame_stamp": (
                diagnostic_stamp_matches
            ),
            "diagnostic_ray_frame_metadata_matches": (
                diagnostic_ray_metadata_matches
            ),
            "frame_alignment_exact": frame_aligned,
            "frame_alignment_mismatch_count": alignment_mismatch_count,
            "single_device_stream": single_device_stream,
            "packet_count": packet_count,
            "raw_packet_count": raw_packet_count,
            "sample_count": sample_count,
            "driver_sample_count": driver_sample_count,
            "zero_depth_count": self.zero_depth_count,
            "zero_depth_with_finite_angles": self.zero_depth_with_finite_angles,
            "zero_depth_with_nontrivial_angles": self.zero_depth_with_nontrivial_angles,
            "zero_depth_nontrivial_angle_ratio": nontrivial_ratio,
            "zero_depth_direction_span_rad": self.zero_depth_direction_span_rad,
            "angle_temporal_continuity_ratio": continuity_ratio,
            "zero_valid_neighbor_pair_count": self.continuity_checked_count,
            "zero_valid_neighbor_pair_pass_count": self.continuity_pass_count,
            "qualifies": not reasons,
            "failure_reasons": reasons,
        }


class CapabilityEvaluator:
    def __init__(self, thresholds=None):
        self.thresholds = thresholds or ProbeThresholds()
        self.thresholds.validate()
        self.trials = []

    def add_trial(self, trial):
        required = {
            "trial_id", "device", "serial_number", "firmware",
            "device_restart_marker", "pcl_data_type_3_accepted",
            "spherical_packet_confirmed", "timestamp_metadata_valid",
            "timestamp_synchronized",
            "diagnostic_header_stamp_matches_frame_stamp",
            "diagnostic_ray_frame_metadata_matches",
            "frame_alignment_exact",
            "single_device_stream",
            "packet_count", "sample_count",
            "zero_depth_count", "zero_depth_with_finite_angles",
            "zero_depth_with_nontrivial_angles",
            "zero_depth_direction_span_rad",
            "angle_temporal_continuity_ratio",
            "zero_valid_neighbor_pair_count", "qualifies",
            "failure_reasons",
        }
        if not isinstance(trial, dict) or not required.issubset(trial):
            raise ValueError("trial result is incomplete")
        if any(existing["trial_id"] == trial["trial_id"] for existing in self.trials):
            raise ValueError("duplicate trial_id")
        self.trials.append(dict(trial))

    def report(self):
        trial_count = len(self.trials)
        devices = {trial["device"] for trial in self.trials}
        serial_numbers = {trial["serial_number"] for trial in self.trials}
        firmwares = {trial["firmware"] for trial in self.trials}
        restart_markers = {
            trial["device_restart_marker"] for trial in self.trials
        }
        restart_reproducible = (
            trial_count >= self.thresholds.required_restart_trials
            and len(devices) == 1
            and len(serial_numbers) == 1
            and len(firmwares) == 1
            and len(restart_markers) >= self.thresholds.required_restart_trials
            and all(trial["qualifies"] for trial in self.trials)
        )
        exact = bool(restart_reproducible)
        packet_count = sum(_as_int(trial["packet_count"]) for trial in self.trials)
        sample_count = sum(_as_int(trial["sample_count"]) for trial in self.trials)
        zero_count = sum(_as_int(trial["zero_depth_count"]) for trial in self.trials)
        finite_count = sum(
            _as_int(trial["zero_depth_with_finite_angles"])
            for trial in self.trials
        )
        nontrivial_count = sum(
            _as_int(trial["zero_depth_with_nontrivial_angles"])
            for trial in self.trials
        )
        continuity = min(
            (_as_float(trial["angle_temporal_continuity_ratio"])
             for trial in self.trials),
            default=0.0,
        )
        direction_span = min(
            (_as_float(trial["zero_depth_direction_span_rad"])
             for trial in self.trials),
            default=0.0,
        )
        zero_valid_neighbor_pairs = sum(
            _as_int(trial["zero_valid_neighbor_pair_count"])
            for trial in self.trials
        )
        reasons = []
        if trial_count < self.thresholds.required_restart_trials:
            reasons.append("insufficient_independent_restart_trials")
        if len(devices) > 1:
            reasons.append("device_changed_between_trials")
        if len(serial_numbers) > 1:
            reasons.append("device_serial_changed_between_trials")
        if len(firmwares) > 1:
            reasons.append("firmware_changed_between_trials")
        if len(restart_markers) < self.thresholds.required_restart_trials:
            reasons.append("device_restart_not_observed_between_trials")
        if any(not trial["qualifies"] for trial in self.trials):
            reasons.append("one_or_more_trials_failed")
        return {
            "schema_version": SCHEMA_VERSION,
            "device": next(iter(devices)) if len(devices) == 1 else "UNAVAILABLE",
            "serial_number": (
                next(iter(serial_numbers))
                if len(serial_numbers) == 1 else "UNAVAILABLE"
            ),
            "firmware": next(iter(firmwares)) if len(firmwares) == 1 else "UNAVAILABLE",
            "device_restart_markers": sorted(restart_markers),
            "pcl_data_type_3_accepted": bool(self.trials) and all(
                trial["pcl_data_type_3_accepted"] for trial in self.trials
            ),
            "spherical_packet_confirmed": bool(self.trials) and all(
                trial["spherical_packet_confirmed"] for trial in self.trials
            ),
            "timestamp_metadata_valid": bool(self.trials) and all(
                trial["timestamp_metadata_valid"] for trial in self.trials
            ),
            "timestamp_synchronized": bool(self.trials) and all(
                trial["timestamp_synchronized"] for trial in self.trials
            ),
            "diagnostic_header_stamp_matches_frame_stamp": (
                bool(self.trials) and all(
                    trial["diagnostic_header_stamp_matches_frame_stamp"]
                    for trial in self.trials
                )
            ),
            "diagnostic_ray_frame_metadata_matches": (
                bool(self.trials) and all(
                    trial["diagnostic_ray_frame_metadata_matches"]
                    for trial in self.trials
                )
            ),
            "frame_alignment_exact": bool(self.trials) and all(
                trial["frame_alignment_exact"] for trial in self.trials
            ),
            "single_device_stream": bool(self.trials) and all(
                trial["single_device_stream"] for trial in self.trials
            ),
            "packet_count": packet_count,
            "sample_count": sample_count,
            "zero_depth_count": zero_count,
            "zero_depth_with_finite_angles": finite_count,
            "zero_depth_with_nontrivial_angles": nontrivial_count,
            "zero_depth_nontrivial_angle_ratio": (
                nontrivial_count / zero_count if zero_count else 0.0
            ),
            "zero_depth_direction_span_rad": direction_span,
            "angle_temporal_continuity_ratio": continuity,
            "zero_valid_neighbor_pair_count": zero_valid_neighbor_pairs,
            "restart_trial_count": trial_count,
            "required_restart_trials": self.thresholds.required_restart_trials,
            "restart_reproducible": restart_reproducible,
            "exact_no_return_direction_supported": exact,
            "ray_source_mode": (
                "hw_spherical_exact" if exact else "calibrated_fallback"
            ),
            "failure_reasons": reasons,
            "trials": list(self.trials),
        }
