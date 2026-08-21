# First-ray time and output/world frame of origin and direction below.
std_msgs/Header header

# Unmodified header of the input RayBundle.
std_msgs/Header source_header

uint32 scan_id
uint32 pattern_start_index

float32 min_range
float32 max_range

# Provenance and the TF interpretation used by the preprocessor.
string source_mode
string ray_time_geometry_mode

mid360_ray_msgs/CheckedRay[] rays
