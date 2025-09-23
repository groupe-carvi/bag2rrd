use clap::{Parser, Subcommand, ArgAction};

#[derive(Parser, Debug)]
#[command(name = "bag2rrd", about = "Convert ROS1 bag files into Rerun RRD files", version)]
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
    },

    /// Show supported ROS→Rerun mappings (stubbed until v0.4.0)
    Schema {},

    /// Validate an .rrd file (stubbed until v0.4.0)
    Validate { rrd: String },
}
