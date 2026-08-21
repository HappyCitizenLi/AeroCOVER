# Index in the synchronized source bundle (and in the raw cloud when the
# producer guarantees one-to-one indexing).
uint32 original_index

# Complete, unmodified emitted-ray sample from the source interface.
mid360_ray_msgs/Ray source

# Ray geometry in CheckedRayBundle.header.frame_id at this ray's timestamp.
geometry_msgs/Point origin
geometry_msgs/Vector3 direction

# direction_valid is false when the source direction was non-finite or zero.
# transform_valid is false when no world-frame transform was available.  The
# preprocessor currently drops an entire checked bundle on a TF failure, so a
# published checked ray normally has transform_valid=true.
bool direction_valid
bool transform_valid
