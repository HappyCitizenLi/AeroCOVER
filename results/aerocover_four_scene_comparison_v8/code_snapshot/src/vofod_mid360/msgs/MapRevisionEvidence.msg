# Passive, truth-independent full-map evidence for one committed revision.
# Linear voxel index is x + map_size_x*(y + map_size_y*z).
uint8 SCHEMA_VERSION=2
uint8 BIT_ORDER_LINEAR_INDEX_LSB0=1
uint8 DECISION_MAP_VIEW_NOT_APPLICABLE=0
uint8 DECISION_MAP_VIEW_DELAYED_COMMITTED=1
uint8 DECISION_MAP_VIEW_IMMEDIATE_CANDIDATE=2

std_msgs/Header header
uint8 schema_version
uint32 scan_id
uint64 frame_token
# Enhanced frontend: exact candidate-raycast decision view and frozen base R.
# B0: NOT_APPLICABLE and zero.  map_revision is the accepted committed R+1.
uint8 decision_map_view
uint64 decision_base_map_revision
uint64 map_revision

geometry_msgs/Point map_origin
uint32 map_size_x
uint32 map_size_y
uint32 map_size_z
float32 voxel_size_m
float32 sure_obstacle_threshold

# Both arrays contain ceil(voxel_count/8) bytes.  For linear index i, byte is
# i/8 and the mask is 1<<(i%8).  Unused high bits of the final byte are zero.
uint8 bit_order
uint32 voxel_count
uint64 free_updated_this_revision_count
uint8[] sure_occupied_bits
uint8[] free_updated_this_revision_bits
