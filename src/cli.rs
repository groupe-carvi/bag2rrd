use clap::{ArgAction, Parser, Subcommand};

#[derive(Parser, Debug)]
#[command(
    name = "bag2rrd",
    about = "Convert ROS1 bag files into Rerun RRD files",
    version
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// List topics, types, message counts and time span of a bag
    Inspect {
        /// Path to the .bag file
        bag: String,
    },

    /// Convert a bag into an .rrd file (images only in v0.1.0)
    Convert {
        /// Path to the .bag file
        bag: String,
        /// Output .rrd path
        out: String,
        /// Include only these topics (can be repeated)
        #[arg(long = "include", action = ArgAction::Append)]
        include: Vec<String>,
        /// Exclude these topics (can be repeated)
        #[arg(long = "exclude", action = ArgAction::Append)]
        exclude: Vec<String>,
        /// Start offset in seconds from the beginning of the bag
        #[arg(long = "start")]
        start: Option<f64>,
        /// End offset in seconds from the beginning of the bag
        #[arg(long = "end")]
        end: Option<f64>,
        /// Dry-run: show plan but do not write any RRD
        #[arg(long = "dry-run")]
        dry_run: bool,
        /// Show progress bar (enabled by default)
        #[arg(long = "progress", action = ArgAction::SetTrue, default_value_t = true)]
        progress: bool,
        /// Segment size (images kept) for parallel flush; if set, produce multiple .rrd files with suffixes
        #[arg(long = "segment-size")]
        segment_size: Option<usize>,
        /// Use LineStrips2D instead of Points2D for LaserScan
        #[arg(long = "scan-as-lines", default_value_t = false)]
        scan_as_lines: bool,
        /// GPS origin for ENU projection: "LAT,LON,ALT" (ellipsoidal meters)
        #[arg(long = "gps-origin")]
        gps_origin: Option<String>,
        /// Name of the GPS frame entity path (for future TF integration)
        #[arg(long = "gps-frame", default_value = "gps_link")]
        gps_frame: String,
        /// Log a polyline path for GPS track
        #[arg(long = "gps-path", default_value_t = true)]
        gps_path: bool,
        /// Segment size in bytes (approx) before flushing a new part (in addition to --segment-size)
        #[arg(long = "segment-bytes")]
        segment_bytes: Option<u64>,
        /// Number of parallel flush workers for segments (>=1)
        #[arg(long = "flush-workers", default_value_t = 2)]
        flush_workers: usize,
    },

    /// Show supported ROS→Rerun mappings (stubbed until v0.4.0)
    Schema {},

    /// Validate an .rrd file (stubbed until v0.4.0)
    Validate { rrd: String },
}
