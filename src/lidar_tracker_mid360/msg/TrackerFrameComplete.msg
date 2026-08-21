# Source-time identity of the paired points/detections frame.  A message is
# emitted exactly once after both callbacks have finished and after every
# Tracks publish call made by those callbacks.
std_msgs/Header header

# Monotonic in publisher order, starting at zero for each tracker process.
uint64 completion_sequence

# Callback outcome values used by both status fields below.
uint8 STATUS_OK=0
uint8 STATUS_EMPTY=1
uint8 STATUS_INVALID_INPUT=2
uint8 STATUS_TRANSFORM_FAILED=3
uint8 STATUS_INTERNAL_ERROR=4
uint8 points_status
uint8 detections_status

# Number of Tracks publish calls completed by each callback.  The detection
# count is the full input batch size, including detections rejected as invalid.
uint32 points_tracks_publications
uint32 detections_tracks_publications
uint32 detections_received

# Cumulative barrier diagnostics at this completion.  These counters make
# duplicate, regressive, incomplete/overflowed and late-finish behavior
# externally auditable without changing the normal tracking output topics.
uint64 duplicate_inputs_dropped
uint64 regressive_inputs_dropped
uint64 incomplete_frames_dropped
uint64 late_finishes_dropped
uint32 pending_frames
