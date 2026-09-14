std_msgs/Header header
# Original input-frame identity; not the state epoch. Each track's
# last_prediction is authoritative; header.stamp is the latest state epoch.
std_msgs/Header source_header
Track[] tracks
