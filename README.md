# bag2rrd (Rust)

**CLI utility** to convert **ROS1 .bag** files into **Rerun .rrd** files.

- Reading via [`rosbag = 0.6.3`]
- Writing via [`rerun = 0.25.1`]
- Built in Rust, CLI via `clap`

## Features (v0.4.0)

- **Images**: `sensor_msgs/Image`, `sensor_msgs/CompressedImage`
- **PointClouds**: `sensor_msgs/PointCloud2` (with optional RGB colors)
- **LaserScans**: `sensor_msgs/LaserScan` (as Points2D or LineStrips2D)
- **GPS**: `sensor_msgs/NavSatFix` (ENU-projected Points3D + optional path + geoid correction + status/service logging)
- **TF**: `/tf`, `/tf_static` (time-aware TF graph with interpolation)
- **Odometry**: `nav_msgs/Odometry` (as Transforms3D)
- **PoseStamped**: `geometry_msgs/PoseStamped` (as Transforms3D)
- **Path**: `nav_msgs/Path` (as LineStrips3D)
- **Parallel flushing**: Background workers for faster segmentation
- **Segmentation**: By image count or byte threshold
- **Schema inspection**: View supported ROS→Rerun mappings
- **Validation**: Basic RRD file structure validation
- **Metadata embedding**: Add custom key=value metadata to RRD files
- **Corruption tolerance**: Skip corrupted chunks in damaged bag files

## Library Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
bag2rrd = "0.4"
```

```rust
use bag2rrd::{convert_bag, ConvertOptions, inspect_bag, diagnose_bag, print_schema, validate_rrd, TfMode};

// Inspect a bag file
inspect_bag("input.bag")?;

// Diagnose bag file corruption
diagnose_bag("input.bag")?;

// Print supported ROS→Rerun mappings
print_schema()?;

// Convert a bag file
let options = ConvertOptions {
    bag_path: "input.bag".to_string(),
    output_path: "output.rrd".to_string(),
    include_topics: vec![],
    exclude_topics: vec![],
    start_time: None,
    end_time: None,
    dry_run: false,
    show_progress: true,
    segment_size: None,
    scan_as_lines: false,
    gps_origin: None,
    gps_path: true,
    segment_bytes: None,
    flush_workers: 2,
    root_frame: "world".to_string(),
    frame_mappings: vec![],
    topic_renames: vec![],
    tf_buffer_seconds: 30.0,
    tf_mode: TfMode::Nearest,
    metadata: vec![],
    gps_geoid: None,
    tolerate_corruption: false,
};

convert_bag(&options)?;

// Validate the output RRD file
validate_rrd("output.rrd")?;
```
```

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

# GPS with geoid correction and metadata
bag2rrd convert run04.bag run04.rrd --gps-geoid egm96-15.pgm \
  --metadata "vehicle=car123" --metadata "driver=test_driver"

# Inspect bag contents
bag2rrd inspect run02.bag

# Show supported ROS→Rerun mappings
bag2rrd schema

# Validate an RRD file
bag2rrd validate output.rrd
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
