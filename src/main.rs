use anyhow::{anyhow, Result};
use clap::Parser;
use tracing_subscriber::{EnvFilter, fmt};

mod cli;
mod convert;
mod mappings;
mod rosbags_io;
mod rrd_writer;
mod schema;
mod validate;
use cli::{Cli, Commands};
use mappings::tf::parse_tf_mode;

fn parse_pointcloud_rotation(rotation_str: &str) -> Result<[f64; 3]> {
    let parts: Vec<&str> = rotation_str.split(',').collect();
    if parts.len() != 3 {
        return Err(anyhow!("The rotation must contain exactly 3 values separated by commas (roll,pitch,yaw)"));
    }
    
    let roll = parts[0].trim().parse::<f64>()
        .map_err(|_| anyhow!("Failed to parse roll: '{}'", parts[0]))?;
    let pitch = parts[1].trim().parse::<f64>()
        .map_err(|_| anyhow!("Failed to parse pitch: '{}'", parts[1]))?;
    let yaw = parts[2].trim().parse::<f64>()
        .map_err(|_| anyhow!("Failed to parse yaw: '{}'", parts[2]))?;
    
    Ok([roll, pitch, yaw])
}

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
            gps_path,
            segment_bytes,
            flush_workers,
            root_frame,
            map_frame,
            topic_rename,
            tf_buffer_seconds,
            tf_mode,
            metadata,
            gps_geoid,
            tolerate_corruption,
            pointcloud_rotation,
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
                gps_path,
                segment_bytes,
                flush_workers,
                root_frame,
                frame_mappings: map_frame,
                topic_renames: topic_rename,
                tf_buffer_seconds,
                tf_mode: parse_tf_mode(&tf_mode)?,
                metadata,
                gps_geoid,
                tolerate_corruption,
                pointcloud_rotation: match pointcloud_rotation {
                    Some(rotation_str) => Some(parse_pointcloud_rotation(&rotation_str)?),
                    None => None,
                },
            };
            convert::convert_bag(&options)
        }
        Commands::Schema {} => {
            schema::print_schema()
        }
        Commands::Validate { rrd } => {
            validate::validate_rrd(&rrd)
        }
        Commands::Diagnose { bag } => {
            rosbags_io::diagnose_bag(&bag)
        }
    }
}
