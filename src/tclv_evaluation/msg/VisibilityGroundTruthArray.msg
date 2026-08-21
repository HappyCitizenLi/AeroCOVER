# Snapshot time/frame copied from the evaluated CheckedRayBundle.
std_msgs/Header header
uint32 scan_id
string geometry_contract_id
string ray_time_geometry_mode

uint32 input_ray_count
uint32 eligible_ray_count
# Exclusive bundle-level exclusion categories; invalid_ray_count is their sum.
uint32 invalid_ray_count
uint32 geometry_invalid_ray_count
uint32 below_min_status_ray_count
uint32 invalid_status_ray_count
uint32 target_candidate_ray_count

# Profiling is wall-clock CPU time for this message only.
float32 target_intersection_ms
float32 static_raycast_ms
float32 occlusion_classification_ms
float32 total_ms

tclv_evaluation/VisibilityGroundTruth[] targets
