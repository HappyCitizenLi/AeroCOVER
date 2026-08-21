# Companion identity for a legacy PointCloud2 and its complete RayBundle.
# The source producer creates this only after both payloads come from one scan.
std_msgs/Header header

uint32 scan_id
uint32 pattern_start_index
uint32 point_count
uint32 ray_count

time point_source_stamp
time ray_source_stamp
