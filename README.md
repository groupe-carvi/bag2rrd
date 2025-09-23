# bag2rrd (Rust)

**CLI utility** to convert **ROS1 .bag** files into **Rerun .rrd** files.

- Reading via [`rosbag = 0.6.3`]
- Writing via [`rerun = 0.25.1`]
- Built in Rust, CLI via `clap`

## Features (v0.3.0)

- **Images**: `sensor_msgs/Image`, `sensor_msgs/CompressedImage`
- **PointClouds**: `sensor_msgs/PointCloud2` (with optional RGB colors)
- **LaserScans**: `sensor_msgs/LaserScan` (as Points2D or LineStrips2D)
- **GPS**: `sensor_msgs/NavSatFix` (ENU-projected Points3D + optional path)
- **TF**: `/tf`, `/tf_static` (time-aware TF graph with interpolation)
- **Odometry**: `nav_msgs/Odometry` (as Transforms3D)
- **PoseStamped**: `geometry_msgs/PoseStamped` (as Transforms3D)
- **Path**: `nav_msgs/Path` (as LineStrips3D)
- **Parallel flushing**: Background workers for faster segmentation
- **Segmentation**: By image count or byte threshold

## Quick start
```bash
cargo run -- --help
```

## Examples

```bash
# Basic conversion (images only)
bag2rrd convert run01.bag run01.rrd

# With PointCloud2, LaserScan, GPS
bag2rrd convert run02.bag run02.rrd --scan-as-lines --gps-origin 46.7821,-71.2740,90 \
  --segment-size 300 --segment-bytes 200000000 --flush-workers 2

# Using TF to anchor odometry and pose into world
bag2rrd convert run03.bag run03.rrd --root-frame world \
  --map-frame base_link=/world/base robot=/world/robot \
  --tf-mode interpolate

# Logging a path from PoseStamped
bag2rrd convert run03.bag run03.rrd --topic-rename /slam/pose=/world/slam_pose

# Inspect bag contents
bag2rrd inspect run02.bag
```

## Testing

Download a test ROS bag file and try the commands:

```bash
# Download test data (not tracked by git)
./tests/download_test_bag.sh

# Inspect the bag file
cargo run -- inspect tests/data/race_1.bag

# Convert to RRD (images only in v0.1.0)
cargo run -- convert tests/data/race_1.bag output.rrd
```

## Development quality

```bash
cargo fmt -- --check
cargo clippy -- -D warnings
cargo test
```

## License

Apache-2.0
