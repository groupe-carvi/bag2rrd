//! bag2rrd - Convert ROS1 .bag files into Rerun .rrd files
//!
//! This library provides functionality to convert ROS1 bag files containing
//! sensor data (images, point clouds, laser scans, GPS, TF transforms,
//! odometry, poses, and paths) into Rerun .rrd files for visualization.
//!
//! # Features
//!
//! - **Images**: `sensor_msgs/Image`, `sensor_msgs/CompressedImage`
//! - **PointClouds**: `sensor_msgs/PointCloud2` with optional RGB colors
//! - **LaserScans**: `sensor_msgs/LaserScan` as Points2D or LineStrips2D
//! - **GPS**: `sensor_msgs/NavSatFix` with ENU projection and path tracking
//! - **TF**: `/tf`, `/tf_static` with time-aware transform resolution
//! - **Odometry**: `nav_msgs/Odometry` as Transforms3D
//! - **PoseStamped**: `geometry_msgs/PoseStamped` as Transforms3D
//! - **Path**: `nav_msgs/Path` as LineStrips3D
//! - **Parallel processing**: Background workers for efficient conversion
//! - **Segmentation**: By image count or byte thresholds
//!
//! # Example
//!
//! ```rust,no_run
//! use bag2rrd::{convert_bag, ConvertOptions, TfMode};
//!
//! let options = ConvertOptions {
//!     bag_path: "input.bag".to_string(),
//!     output_path: "output.rrd".to_string(),
//!     include_topics: vec![],
//!     exclude_topics: vec![],
//!     start_time: None,
//!     end_time: None,
//!     dry_run: false,
//!     show_progress: true,
//!     segment_size: None,
//!     scan_as_lines: false,
//!     gps_origin: None,
//!     gps_path: true,
//!     segment_bytes: None,
//!     flush_workers: 2,
//!     root_frame: "world".to_string(),
//!     frame_mappings: vec![],
//!     topic_renames: vec![],
//!     tf_buffer_seconds: 30.0,
//!     tf_mode: TfMode::Nearest,
//!     metadata: vec![],
//!     gps_geoid: None,
//! };
//!
//! convert_bag(&options)?;
//! # Ok::<(), anyhow::Error>(())
//! ```

pub mod cli;
pub mod convert;
pub mod mappings;
pub mod rosbags_io;
pub mod rrd_writer;
pub mod schema;
pub mod validate;

// Re-export main types for convenience
pub use convert::{convert_bag, ConvertOptions};
pub use mappings::tf::{TfGraph, TfMode, TfSample};
pub use rosbags_io::diagnose_bag;