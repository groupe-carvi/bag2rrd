use anyhow::Result;
use clap::Parser;
use tracing_subscriber::{EnvFilter, fmt};

mod cli;
mod convert;
mod mappings;
mod rosbags_io;
mod rrd_writer;
use cli::{Cli, Commands};

fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    fmt().with_env_filter(filter).init();
}

fn main() -> Result<()> {
    init_tracing();
    let cli = Cli::parse();
    match cli.command {
        Commands::Inspect { bag } => rosbags_io::inspect_bag(&bag),
        Commands::Convert {
            bag,
            out,
            include,
            exclude,
            start,
            end,
            dry_run,
            progress,
            segment_size,
            scan_as_lines,
            gps_origin,
            gps_frame,
            gps_path,
            segment_bytes,
            flush_workers,
        } => convert::convert_bag(
            &bag,
            &out,
            include,
            exclude,
            start,
            end,
            dry_run,
            progress,
            segment_size,
            scan_as_lines,
            gps_origin,
            gps_frame,
            gps_path,
            segment_bytes,
            flush_workers,
        ),
        Commands::Schema {} => {
            println!("Image/CompressedImage only in v0.1.0");
            Ok(())
        }
        Commands::Validate { rrd } => {
            println!("validate is planned for v0.4.0. input={}", rrd);
            Ok(())
        }
    }
}
