use clap::{Parser, Subcommand};

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
    /// List topics, types, message counts and time span of a bag (implemented in v0.1.0)
    Inspect { bag: String },
    /// Convert a bag into an .rrd file (implemented progressively in v0.1.0+)
    Convert { bag: String, out: String },
    /// Show supported ROS→Rerun mappings (planned for v0.2.0/v0.3.0)
    Schema {},
    /// Validate an .rrd file (planned for v0.4.0)
    Validate { rrd: String },
}
