# Controlled upstream boundary

This directory is a no-hardlink clone of `Livox-SDK/livox_ros_driver2` at
commit `6b9356cadf77084619ba406e6a0eb41163b08039` (release 1.2.5). The retained
upstream license is [LICENSE.txt](LICENSE.txt).

The local ROS1 overlay adds an optional hook before the upstream
`ProcessSphericalPoint()` conversion. When `publish_ray_bundle:=true` and the
device actually emits data type `3`, it preserves `depth/theta/phi`, publishes
the common `mid360_ray_msgs/RayBundle`, and publishes capability diagnostics.
The original PointCloud2, Livox CustomMsg, PCL, IMU, and ROS bag branches are
left in place. The upstream `build.sh` is intentionally not used because it
deletes workspace build/devel/install directories.

This fork does not itself claim that zero-depth angles from any physical
Mid-360 firmware are valid. The separate capability probe must establish that
fact over repeated hardware restarts; otherwise the system remains in
`calibrated_fallback`.

Phase 1 also publishes `mid360_ray_msgs/ScanIdentity` at the same cloud-frame
flush boundary and carries the accepted internal scan ID through the ROS1
PointCloud2 generation queue. Existing point fields and topics remain intact.
