# Target-level truth classification.  These values describe geometry at the
# CheckedRayBundle header stamp; they are never algorithm inputs.
uint8 UNSCANNED=0
uint8 VISIBLE=1
uint8 STATIC_OCCLUDED=2
uint8 TARGET_OCCLUDED=3
uint8 MIXED_OCCLUDED=4
uint8 INVALID=255

string target_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Evaluation-only presence/visibility flags. The evaluator configuration
# contains only targets active in the scored interval.
bool present
bool in_range
bool in_fov
bool line_of_sight

float32 true_visibility_fraction
uint8 true_occlusion_type
string true_occluder_id
# Modal blocker after restricting support to VALID_RETURN rays only.  Empty
# when the valid-only policy has no blocked ray.  This lets the P-valid-only
# ablation be evaluated without treating NO_RETURN evidence as truth input.
string true_valid_only_occluder_id

# Only direction/TF-valid rays whose source status is VALID_RETURN or
# NO_RETURN contribute to the emitted/unblocked truth counts.
uint32 true_emitted_ray_intersection_count
uint32 true_valid_return_intersection_count
uint32 true_no_return_intersection_count
uint32 true_actual_target_return_count

uint32 true_map_blocked_ray_count
uint32 true_target_blocked_ray_count
# Status-resolved blocker counts.  The two fields for each blocker class sum
# to the corresponding all-ray count above.
uint32 true_map_blocked_valid_return_count
uint32 true_map_blocked_no_return_count
uint32 true_target_blocked_valid_return_count
uint32 true_target_blocked_no_return_count
uint32 true_unblocked_ray_count
uint32 true_unblocked_valid_return_count
uint32 true_unblocked_no_return_count

# Geometry intersections excluded from emitted support by source status.
uint32 excluded_below_min_intersection_count
uint32 excluded_invalid_status_intersection_count
