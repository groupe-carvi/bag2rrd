use anyhow::Result;
use clap::Parser;
use tracing_subscriber::{EnvFilter, fmt};

mod cli;
mod convert;
mod mappings;
mod rosbags_io;
mod rrd_writer;
use cli::{Cli, Commands};
use mappings::tf::parse_tf_mode;

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
            root_frame,
            map_frame,
            topic_rename,
            tf_buffer_seconds,
            tf_mode,
        } => {
            let options = convert::ConvertOptions {
                bag_path: bag,
                output_path: out,
                include_topics: include,
                exclude_topics: exclude,
                start_time: start,
                end_time: end,
                dry_run,
                show_progress: progress,
                segment_size,
                scan_as_lines,
                gps_origin,
                gps_frame,
                gps_path,
                segment_bytes,
                flush_workers,
                root_frame,
                frame_mappings: map_frame,
                topic_renames: topic_rename,
                tf_buffer_seconds,
                tf_mode: parse_tf_mode(&tf_mode)?,
            };
            convert::convert_bag(&options)
        }
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
