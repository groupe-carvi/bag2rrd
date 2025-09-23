//! bag2rrd — ROS1 bag to Rerun RRD converter (skeleton for v0.0.1)

mod cli;
mod convert;
mod mappings;
mod rosbags_io;
mod rrd_writer;

use anyhow::Result;
use clap::Parser;
use cli::{Cli, Commands};
use tracing_subscriber::{EnvFilter, fmt};

fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    fmt().with_env_filter(filter).init();
}

fn main() -> Result<()> {
    init_tracing();
    let cli = Cli::parse();
    match cli.command {
        Commands::Inspect { bag } => {
            println!(
                "inspect is not implemented yet (will be delivered in v0.1.0). bag={}",
                bag
            );
        }
        Commands::Convert { bag, out } => {
            println!(
                "convert is not implemented yet (will be delivered in v0.1.0+). bag={} out={}",
                bag, out
            );
        }
        Commands::Schema {} => {
            println!("schema is not implemented yet (will be delivered in v0.2.0/v0.3.0).");
        }
        Commands::Validate { rrd } => {
            println!(
                "validate is not implemented yet (will be delivered in v0.4.0). rrd={}",
                rrd
            );
        }
    }
    Ok(())
}
