uint8 TENTATIVE=1
uint8 CONFIRMED=2
uint8 DELETING=3

std_msgs/Header header
uint32 track_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] covariance
float32 existence_probability
uint8 state
float32 age
uint32 num_positive_updates
float32 cumulative_effective_opportunity
float32 time_since_last_measurement
string deletion_reason

