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
#[allow(clippy::large_enum_variant)]
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
        /// Log a polyline path for GPS track
        #[arg(long = "gps-path", default_value_t = true)]
        gps_path: bool,
        /// Path to EGM96 geoid grid file (.pgm) for altitude correction
        #[arg(long = "gps-geoid")]
        gps_geoid: Option<String>,
        /// Segment size in bytes (approx) before flushing a new part (in addition to --segment-size)
        #[arg(long = "segment-bytes")]
        segment_bytes: Option<u64>,
        /// Number of parallel flush workers for segments (>=1)
        #[arg(long = "flush-workers", default_value_t = 2)]
        flush_workers: usize,
        /// Root frame name for logging transforms (default: "world")
        #[arg(long = "root-frame", default_value = "world")]
        root_frame: String,
        /// Map ROS frame names to Rerun entity paths: FRAME=/rr/path (repeatable)
        /// Example: --map-frame base_link=/world/base robot=/world/robot
        #[arg(long = "map-frame", action = clap::ArgAction::Append)]
        map_frame: Vec<String>,
        /// Rename a ROS topic to a specific Rerun entity path: ROS_TOPIC=/rr/path (repeatable)
        #[arg(long = "topic-rename", action = clap::ArgAction::Append)]
        topic_rename: Vec<String>,
        /// TF buffer duration in seconds to retain dynamic transforms
        #[arg(long = "tf-buffer-seconds", default_value_t = 30.0)]
        tf_buffer_seconds: f64,
        /// TF sampling mode when an exact timestamp is missing: nearest|interpolate|none
        #[arg(long = "tf-mode", default_value = "nearest")]
        tf_mode: String,
        /// Key=value metadata entries to embed in the RRD (repeatable)
        #[arg(long = "metadata", action = clap::ArgAction::Append)]
        metadata: Vec<String>,
        /// Tolerate bag file corruption by skipping corrupted chunks
        #[arg(long = "tolerate-corruption", default_value_t = false)]
        tolerate_corruption: bool,
    },

    /// Show supported ROS→Rerun mappings
    Schema {},

    /// Validate an .rrd file
    Validate { rrd: String },

    /// Diagnose bag file corruption and structure issues
    Diagnose {
        /// Path to the .bag file
        bag: String,
    },
}
