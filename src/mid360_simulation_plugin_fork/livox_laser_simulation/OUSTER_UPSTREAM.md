# Ouster GPU publisher overlay

`src/mrs_3dlidar_plugin.cpp` is from ctu-mrs/mrs_gazebo_common_resources
commit `5ee221fb438515d428abb35f896176b6ef14a874`,
`src/sensor_and_model_plugins/3dlidar_plugin.cpp` (BSD-3-Clause, retained in source).
https://github.com/ctu-mrs/mrs_gazebo_common_resources/blob/5ee221fb438515d428abb35f896176b6ef14a874/src/sensor_and_model_plugins/3dlidar_plugin.cpp

Local changes: serialize admission; reject zero/repeated/regressive source scan
times before point conversion/noise; retain original acquisition times and all
organized rays. The two UInt16 placeholder fields use two-byte stores, avoiding
the upstream final-point out-of-bounds write without changing valid payload bytes.
The workspace's existing Gazebo plugin hook loads this overlay; /opt/ros is unchanged.
