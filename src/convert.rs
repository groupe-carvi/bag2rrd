use anyhow::{Context, Result};
use flume::{Receiver, Sender};
use indicatif::{ProgressBar, ProgressStyle};
use rosbag::{ChunkRecord, MessageRecord, RosBag};
use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Instant;

use crate::mappings::tf::TfMode;

/// Options for converting a ROS bag file to Rerun RRD format
#[derive(Debug, Clone)]
pub struct ConvertOptions {
    /// Path to the input .bag file
    pub bag_path: String,
    /// Path to the output .rrd file
    pub output_path: String,
    /// Include only these topics (empty means include all)
    pub include_topics: Vec<String>,
    /// Exclude these topics
    pub exclude_topics: Vec<String>,
    /// Start time offset in seconds from bag start
    pub start_time: Option<f64>,
    /// End time offset in seconds from bag start
    pub end_time: Option<f64>,
    /// Dry run: show plan but don't write output
    pub dry_run: bool,
    /// Show progress bar
    pub show_progress: bool,
    /// Segment size (images kept) for parallel flush
    pub segment_size: Option<usize>,
    /// Use LineStrips2D instead of Points2D for LaserScan
    pub scan_as_lines: bool,
    /// GPS origin for ENU projection: "LAT,LON,ALT"
    pub gps_origin: Option<String>,
    /// Log a polyline path for GPS track
    pub gps_path: bool,
    /// Path to EGM96 geoid grid file for altitude correction
    pub gps_geoid: Option<String>,
    /// Segment size in bytes for parallel flush
    pub segment_bytes: Option<u64>,
    /// Number of parallel flush workers
    pub flush_workers: usize,
    /// Root frame name for transforms
    pub root_frame: String,
    /// Map ROS frame names to Rerun entity paths: FRAME=/rr/path
    pub frame_mappings: Vec<String>,
    /// Rename ROS topics to Rerun entity paths: ROS_TOPIC=/rr/path
    pub topic_renames: Vec<String>,
    /// TF buffer duration in seconds
    pub tf_buffer_seconds: f64,
    /// TF sampling mode
    pub tf_mode: TfMode,
    /// Key=value metadata entries to embed in the RRD
    pub metadata: Vec<String>,
}

#[derive(Debug)]
#[allow(dead_code)]
struct FlushJob {
    part_index: u32,
    tmp_path: PathBuf,
    final_path: PathBuf,
    raw_bytes_in_part: u64,
}

/// Convert a ROS bag file to Rerun RRD format
///
/// # Arguments
///
/// * `options` - Conversion options
///
/// # Returns
///
/// Returns `Ok(())` on success, or an error if conversion fails
///
/// # Example
///
/// ```rust,no_run
/// use bag2rrd::{convert_bag, ConvertOptions, TfMode};
///
/// let options = ConvertOptions {
///     bag_path: "input.bag".to_string(),
///     output_path: "output.rrd".to_string(),
///     include_topics: vec![],
///     exclude_topics: vec![],
///     start_time: None,
///     end_time: None,
///     dry_run: false,
///     show_progress: true,
///     segment_size: None,
///     scan_as_lines: false,
///     gps_origin: None,
///     gps_path: true,
///     segment_bytes: None,
///     flush_workers: 2,
///     root_frame: "world".to_string(),
///     frame_mappings: vec![],
///     topic_renames: vec![],
///     tf_buffer_seconds: 30.0,
///     tf_mode: TfMode::Nearest,
///     metadata: vec![],
///     gps_geoid: None,
/// };
///
/// convert_bag(&options)?;
/// # Ok::<(), anyhow::Error>(())
/// ```
pub fn convert_bag(options: &ConvertOptions) -> Result<()> {
    let bag_file = RosBag::new(&options.bag_path).with_context(|| format!("failed to open bag: {}", options.bag_path))?;

    let mut tf_graph = crate::mappings::tf::TfGraph::new();

    // filters
    let include_set: Option<HashSet<&str>> = if options.include_topics.is_empty() {
        None
    } else {
        Some(options.include_topics.iter().map(|s| s.as_str()).collect())
    };
    let exclude_set: HashSet<&str> = options.exclude_topics.iter().map(|s| s.as_str()).collect();

    // collect all chunks first since the iterator may not be restartable
    let chunks: Vec<_> = bag_file.chunk_records().collect::<Result<Vec<_>, _>>()?;

    // collect connections first
    let mut connections = std::collections::BTreeMap::new();
    for record in &chunks {
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::Connection(conn) = msg {
                    connections.insert(conn.id, (conn.topic.to_string(), conn.tp.to_string()));
                }
            }
        }
    }

    // segmentation validation
    if let Some(sz) = options.segment_size && sz == 0 {
        anyhow::bail!("segment-size must be > 0");
    }
    if let Some(sz) = options.segment_bytes && sz == 0 {
        anyhow::bail!("segment-bytes must be > 0");
    }
    if options.flush_workers == 0 {
        anyhow::bail!("flush-workers must be >= 1");
    }
    let segmentation_enabled = (options.segment_size.is_some() || options.segment_bytes.is_some()) && !options.dry_run;
    let seg_size = options.segment_size.unwrap_or(0) as u64;
    let seg_bytes = options.segment_bytes.unwrap_or(0);

    // Single-output recording (created lazily after first kept message for parity with segments)
    let mut rec: Option<rerun::RecordingStream> = None;

    // For segmentation derive base path components
    let (base_parent, base_stem, base_ext) = if segmentation_enabled {
        let p = Path::new(&options.output_path);
        let parent = p.parent().unwrap_or(Path::new(""));
        let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("out");
        let ext = p.extension().and_then(|s| s.to_str()).unwrap_or("rrd");
        (parent.to_path_buf(), stem.to_string(), ext.to_string())
    } else {
        (PathBuf::new(), String::new(), String::new())
    };

    let mut segment_index: u64 = 0; // 0-based
    let mut segment_images: u64 = 0; // images+compressed in current segment
    let mut segment_raw_bytes: u64 = 0;
    let mut current_tmp_path = PathBuf::new();
    let mut current_final_path = PathBuf::new();

    let open_new_segment = |segment_index: u64,
                            base_parent: &PathBuf,
                            base_stem: &str,
                            base_ext: &str,
                            bag: &str,
                            tmp_dir: &PathBuf,
                            current_tmp_path: &mut PathBuf,
                            current_final_path: &mut PathBuf|
     -> anyhow::Result<rerun::RecordingStream> {
        let final_path = base_parent.join(format!(
            "{}_part{:04}.{}",
            base_stem,
            segment_index + 1,
            base_ext
        ));
        let tmp_path = tmp_dir.join(format!(
            "bag2rrd_tmp_{}_{:04}.{}",
            bag.replace("/", "_"),
            segment_index + 1,
            base_ext
        ));
        *current_tmp_path = tmp_path.clone();
        *current_final_path = final_path.clone();
        let rec_id = format!("bag2rrd:{}:segment:{}", bag, segment_index + 1);
        eprintln!(
            "[bag2rrd][segment {}] opening tmp={}",
            segment_index + 1,
            tmp_path.display()
        );
        Ok(rerun::RecordingStreamBuilder::new(rec_id).save(tmp_path)?)
    };

    // Parallel flush setup
    let (flush_tx, flush_rx): (Sender<FlushJob>, Receiver<FlushJob>) = flume::unbounded();
    let (result_tx, result_rx): (
        Sender<anyhow::Result<FlushJob>>,
        Receiver<anyhow::Result<FlushJob>>,
    ) = flume::unbounded();
    let tmp_dir = std::env::var("BAG2RRD_SEGMENT_TMP_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir().join("bag2rrd_segments"));
    std::fs::create_dir_all(&tmp_dir)?;
    let workers: Vec<_> = (0..options.flush_workers)
        .map(|i| {
            let rx = flush_rx.clone();
            let tx = result_tx.clone();
            let tmp_dir = tmp_dir.clone();
            std::thread::spawn(move || flush_worker(i, rx, tx, tmp_dir))
        })
        .collect();

    // progress bar (unknown length)
    let pb = if options.show_progress {
        let pb = ProgressBar::new_spinner();
        pb.set_style(ProgressStyle::with_template("{spinner} {pos} msgs").unwrap());
        Some(pb)
    } else {
        None
    };

    let mut bag_start_ns = f64::INFINITY;
    let mut total_msgs: u64 = 0;
    let mut kept_msgs: u64 = 0;
    let mut topics: HashSet<String> = HashSet::new();

    // statistics and logging configuration
    #[derive(Default)]
    struct Stats {
        images: u64,
        compressed_images: u64,
        pointclouds: u64,
        laserscans: u64,
        gps_fixes: u64,
        skipped_type: u64,
        filtered_out: u64,
        raw_bytes: u64,
    }
    let mut stats = Stats::default();
    let log_every = std::env::var("BAG2RRD_LOG_EVERY")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .filter(|v| *v > 0);
    let verbose_types: Option<HashSet<String>> = std::env::var("BAG2RRD_LOG_TYPES")
        .ok()
        .map(|s| s.split(',').map(|t| t.trim().to_string()).collect());
    let second_pass_start = Instant::now();

    // First pass: collect bag start time and count messages
    for record in &chunks {
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::MessageData(msg_data) = msg {
                    bag_start_ns = bag_start_ns.min(msg_data.time as f64);
                    total_msgs += 1;
                }
            }
        }
    }

    let bag_start_s = if bag_start_ns.is_finite() {
        bag_start_ns / 1_000_000_000.0
    } else {
        0.0
    };

    // Second pass: process messages
    println!("Starting second pass...");
    for record in &chunks {
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::MessageData(msg_data) = msg {
                    if let Some((topic, tp)) = connections.get(&msg_data.conn_id) {
                        // Apply filters
                        if let Some(inc) = &include_set && !inc.contains(topic.as_str()) {
                            continue;
                        }
                        if exclude_set.contains(topic.as_str()) {
                            continue;
                        }

                        let ts_rel = (msg_data.time as f64 / 1_000_000_000.0) - bag_start_s;
                        if let Some(s) = options.start_time && ts_rel < s {
                            continue;
                        }
                        if let Some(e) = options.end_time && ts_rel > e {
                            continue;
                        }

                        topics.insert(topic.clone());
                        if options.dry_run {
                            kept_msgs += 1;
                            if let Some(pb) = &pb {
                                pb.inc(1);
                            }
                            continue;
                        }

                        // ensure recording stream exists (single or segment)
                        if rec.is_none() {
                            if segmentation_enabled {
                                rec = Some(open_new_segment(
                                    segment_index,
                                    &base_parent,
                                    &base_stem,
                                    &base_ext,
                                    &options.bag_path,
                                    &tmp_dir,
                                    &mut current_tmp_path,
                                    &mut current_final_path,
                                )?);
                            } else {
                                let rec_id = format!("bag2rrd:{}", options.bag_path);
                                rec = Some(rerun::RecordingStreamBuilder::new(rec_id).save(&options.output_path)?);
                            }

                            // Log metadata if provided
                            if let Some(ref rec_ref) = rec {
                                for metadata_entry in &options.metadata {
                                    if let Some((key, value)) = metadata_entry.split_once('=') {
                                        let metadata_path = format!("/metadata/{}", key.trim());
                                        rec_ref.log(metadata_path, &rerun::archetypes::TextLog::new(value.trim()))?;
                                    }
                                }
                            }
                        }

                        // dispatch by type
                        match tp.as_str() {
                            "sensor_msgs/Image" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::images::image_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.images += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled {
                                    segment_images += 1;
                                    segment_raw_bytes += msg_data.data.len() as u64;
                                }
                            }
                            "sensor_msgs/CompressedImage" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::images::compressed_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.compressed_images += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled {
                                    segment_images += 1;
                                    segment_raw_bytes += msg_data.data.len() as u64;
                                }
                            }
                            "sensor_msgs/PointCloud2" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::pointcloud::pointcloud2_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.pointclouds += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled {
                                    segment_images += 1;
                                    segment_raw_bytes += msg_data.data.len() as u64;
                                }
                            }
                            "sensor_msgs/LaserScan" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::laserscan::laserscan_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                        options.scan_as_lines,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.laserscans += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled {
                                    segment_images += 1;
                                    segment_raw_bytes += msg_data.data.len() as u64;
                                }
                            }
                            "sensor_msgs/NavSatFix" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::gps::navsatfix_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                        options.gps_origin.as_deref(),
                                        options.gps_path,
                                        options.gps_geoid.as_deref(),
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.gps_fixes += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled {
                                    segment_images += 1;
                                    segment_raw_bytes += msg_data.data.len() as u64;
                                }
                            }
                            "tf2_msgs/TFMessage" => {
                                if let Some(ref rec_ref) = rec {
                                    tf_graph.ingest_tf_msg(rec_ref, ts_rel, msg_data.data, options.tf_buffer_seconds, &options.root_frame, &options.frame_mappings)?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "tf/tfMessage" => {
                                if let Some(ref rec_ref) = rec {
                                    tf_graph.ingest_tf_msg(rec_ref, ts_rel, msg_data.data, options.tf_buffer_seconds, &options.root_frame, &options.frame_mappings)?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "tf2_msgs/TFMessageStatic" => {
                                if let Some(ref rec_ref) = rec {
                                    tf_graph.ingest_tf_static_msg(rec_ref, msg_data.data, &options.root_frame, &options.frame_mappings)?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "tf/tfMessageStatic" => {
                                if let Some(ref rec_ref) = rec {
                                    tf_graph.ingest_tf_static_msg(rec_ref, msg_data.data, &options.root_frame, &options.frame_mappings)?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "nav_msgs/Odometry" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::nav::odometry_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                        &options.root_frame,
                                        &options.frame_mappings,
                                        Some(&tf_graph),
                                        options.tf_mode,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "geometry_msgs/PoseStamped" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::nav::pose_stamped_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                        &options.root_frame,
                                        &options.topic_renames,
                                        &options.frame_mappings,
                                        Some(&tf_graph),
                                        options.tf_mode,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            "nav_msgs/Path" => {
                                if let Some(ref rec_ref) = rec {
                                    crate::mappings::nav::path_to_rerun(
                                        rec_ref,
                                        topic,
                                        ts_rel,
                                        msg_data.data,
                                        &options.root_frame,
                                        &options.topic_renames,
                                        &options.frame_mappings,
                                        Some(&tf_graph),
                                        options.tf_mode,
                                    )?;
                                }
                                kept_msgs += 1;
                                stats.raw_bytes += msg_data.data.len() as u64;
                            }
                            _ => {
                                stats.skipped_type += 1;
                            }
                        }

                        // Segment rotation
                        if segmentation_enabled
                            && ((seg_size > 0 && segment_images >= seg_size)
                                || (seg_bytes > 0 && segment_raw_bytes >= seg_bytes))
                        {
                            if let Some(_rec_full) = rec.take() {
                                eprintln!(
                                    "[bag2rrd][segment {}] submitting flush job (images={} raw_bytes={})",
                                    segment_index + 1,
                                    segment_images,
                                    segment_raw_bytes
                                );
                                let job = FlushJob {
                                    part_index: (segment_index + 1) as u32,
                                    tmp_path: current_tmp_path.clone(),
                                    final_path: current_final_path.clone(),
                                    raw_bytes_in_part: segment_raw_bytes,
                                };
                                flush_tx.send(job)?;
                                // prepare next
                                segment_index += 1;
                                segment_images = 0;
                                segment_raw_bytes = 0;
                                current_tmp_path.clear();
                                current_final_path.clear();
                            }
                        }
                        if let Some(pb) = &pb {
                            pb.inc(1);
                        }
                        if let Some(ref vt) = verbose_types && vt.contains(tp) {
                            eprintln!("[bag2rrd][msg] topic={topic} type={tp} t={:.6}", ts_rel);
                        }
                        if let Some(n) = log_every && kept_msgs % n == 0 {
                            eprintln!(
                                "[bag2rrd][progress] kept_msgs={} images={} compressed={} pointclouds={} laserscans={} gps_fixes={} skipped_type={} filtered={} elapsed={:?}",
                                kept_msgs,
                                stats.images,
                                stats.compressed_images,
                                stats.pointclouds,
                                stats.laserscans,
                                stats.gps_fixes,
                                stats.skipped_type,
                                stats.filtered_out,
                                second_pass_start.elapsed()
                            );
                        }
                    } else {
                        stats.filtered_out += 1;
                    }
                }
            }
        }
    }

    if let Some(pb) = &pb {
        pb.finish_and_clear();
    }
    println!("Second pass completed");

    println!(
        "Plan: {} messages, {} kept after filters, {} topics → output: {}",
        total_msgs,
        kept_msgs,
        topics.len(),
        options.output_path
    );

    if !options.dry_run {
        eprintln!(
            "[bag2rrd][stats] images={} compressed_images={} pointclouds={} laserscans={} gps_fixes={} skipped_types={} filtered_out={} kept_msgs={} total_msgs={} raw_bytes={}",
            stats.images,
            stats.compressed_images,
            stats.pointclouds,
            stats.laserscans,
            stats.gps_fixes,
            stats.skipped_type,
            stats.filtered_out,
            kept_msgs,
            total_msgs,
            stats.raw_bytes
        );
        if segmentation_enabled {
            // submit last open segment
            if let Some(_rec_last) = rec.take() {
                if segment_images > 0 {
                    // only if something was logged
                    eprintln!(
                        "[bag2rrd][segment {}] submitting final flush job (images={} raw_bytes={})",
                        segment_index + 1,
                        segment_images,
                        segment_raw_bytes
                    );
                    let job = FlushJob {
                        part_index: (segment_index + 1) as u32,
                        tmp_path: current_tmp_path.clone(),
                        final_path: current_final_path.clone(),
                        raw_bytes_in_part: segment_raw_bytes,
                    };
                    flush_tx.send(job)?;
                }
            }
            // Close the channel to signal workers to stop
            drop(flush_tx);
            // Wait for all workers to finish
            let mut completed_jobs = 0;
            let total_jobs = segment_index + if segment_images > 0 { 1 } else { 0 };
            while completed_jobs < total_jobs {
                match result_rx.recv() {
                    Ok(Ok(job)) => {
                        eprintln!(
                            "[bag2rrd][segment {}] completed file={}",
                            job.part_index,
                            job.final_path.display()
                        );
                        completed_jobs += 1;
                    }
                    Ok(Err(e)) => {
                        eprintln!("[bag2rrd][error] flush failed: {}", e);
                        completed_jobs += 1; // still count as completed
                    }
                    Err(_) => break, // channel closed
                }
            }
            // Join workers
            for worker in workers {
                let _ = worker.join();
            }
            let total_segments = total_jobs;
            eprintln!(
                "[bag2rrd] segmentation summary: segments={} segment_size={} segment_bytes={} total_images={} raw_bytes={} pattern='{}_part{{:04}}.{}'",
                total_segments,
                seg_size,
                seg_bytes,
                stats.images + stats.compressed_images,
                stats.raw_bytes,
                base_stem,
                base_ext
            );
        } else if let Some(rec_single) = rec.take() {
            eprintln!(
                "[bag2rrd][single] flushing recording (images={} raw_bytes={})",
                stats.images + stats.compressed_images,
                stats.raw_bytes
            );
            flush_recording(rec_single, &options.output_path, stats.raw_bytes, "[bag2rrd]");
            eprintln!("[bag2rrd] Saved RRD: {}", options.output_path);
        } else {
            // Could happen if no messages matched filters
            eprintln!("[bag2rrd] no messages kept; nothing to flush");
        }
    }

    Ok(())
}

fn flush_worker(
    _id: usize,
    rx: Receiver<FlushJob>,
    tx: Sender<anyhow::Result<FlushJob>>,
    _tmp_dir: PathBuf,
) {
    while let Ok(job) = rx.recv() {
        let res = (|| -> anyhow::Result<FlushJob> {
            // For now, since rerun::RecordingStream may not be Send, we'll assume the job contains the path to a temp file
            // and we just rename it. But in the spec, it's to finalize and save.
            // Since the spec says "finalize to a temporary file on the producer thread", but in this code, the producer is creating the recording directly.
            // To make it work, perhaps we need to change the approach.
            // For simplicity, since rerun saves directly, the FlushJob will contain the final path, and we just wait for the file to be stable.
            // But the spec shows tmp_path and final_path, with rename.
            // Since the recording is saved directly to final_path, perhaps tmp_path is not needed, or we can use it for something else.
            // To follow the spec, let's assume the producer saves to tmp_path, and worker renames to final_path.
            // But in the code, the open_new_segment saves to final_path.
            // I need to modify open_new_segment to save to tmp_path, and FlushJob to rename.
            // Yes, let's do that.

            // Monitor the file size until stable
            let mut last_size = 0u64;
            let mut stable_count = 0;
            let max_stable_checks = 10; // arbitrary
            while stable_count < max_stable_checks {
                std::thread::sleep(std::time::Duration::from_millis(100));
                if let Ok(meta) = std::fs::metadata(&job.tmp_path) {
                    let size = meta.len();
                    if size == last_size {
                        stable_count += 1;
                    } else {
                        stable_count = 0;
                        last_size = size;
                    }
                } else {
                    stable_count = 0;
                }
            }
            // Rename tmp to final
            std::fs::rename(&job.tmp_path, &job.final_path)?;
            Ok(job)
        })();
        let _ = tx.send(res);
    }
}

fn flush_recording(rec: rerun::RecordingStream, out_path: &str, raw_total: u64, prefix: &str) {
    use std::io::Write;
    let debug_timings = std::env::var("BAG2RRD_DEBUG_TIMINGS").ok().as_deref() == Some("1");
    let timeout_secs: u64 = std::env::var("BAG2RRD_FLUSH_TIMEOUT_SECS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    if debug_timings {
        eprintln!(
            "{prefix}[debug] flushing (timeout={}s, 0=forever) file={}",
            timeout_secs, out_path
        );
    }
    let t0 = if debug_timings {
        Some(std::time::Instant::now())
    } else {
        None
    };
    let stop_flag = Arc::new(AtomicBool::new(false));
    let path = out_path.to_string();
    let monitor_flag = Arc::clone(&stop_flag);
    let poll_ms: u64 = std::env::var("BAG2RRD_FLUSH_POLL_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(500);
    let prefix_owned = prefix.to_string();
    let monitor_handle = std::thread::spawn(move || {
        let mut last_size = 0u64;
        let mut last_change = Instant::now();
        while !monitor_flag.load(Ordering::Relaxed) {
            match std::fs::metadata(&path) {
                Ok(meta) => {
                    let size = meta.len();
                    if size != last_size {
                        let delta = size.saturating_sub(last_size);
                        let pct = if raw_total > 0 {
                            (size as f64 / raw_total as f64 * 100.0).min(100.0)
                        } else {
                            0.0
                        };
                        eprintln!(
                            "{}[flush] size={} (+{}) est_progress={:.1}%",
                            prefix_owned, size, delta, pct
                        );
                        last_size = size;
                        last_change = Instant::now();
                    } else if last_change.elapsed() > std::time::Duration::from_secs(5) {
                        eprintln!(
                            "{}[flush] no size change for 5s (size={})",
                            prefix_owned, size
                        );
                        last_change = Instant::now();
                    }
                }
                Err(e) => eprintln!("{}[flush] metadata error: {e}", prefix_owned),
            }
            std::thread::sleep(std::time::Duration::from_millis(poll_ms));
        }
        eprintln!("{}[flush] monitoring stop", prefix_owned);
    });

    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        drop(rec);
        let _ = tx.send(());
    });
    if timeout_secs == 0 {
        let _ = rx.recv();
    } else if rx
        .recv_timeout(std::time::Duration::from_secs(timeout_secs))
        .is_err()
    {
        eprintln!("{prefix}[warn] timeout waiting for flush; file may be incomplete");
    }
    stop_flag.store(true, Ordering::Relaxed);
    let _ = monitor_handle.join();
    std::io::stdout().flush().ok();
    std::io::stderr().flush().ok();
    if let Some(t0) = t0 && debug_timings {
        eprintln!("{prefix}[debug] flush completed in {:?}", t0.elapsed());
    }
}
