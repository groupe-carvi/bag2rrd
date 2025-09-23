use anyhow::{Context, Result};
use indicatif::{ProgressBar, ProgressStyle};
use std::collections::HashSet;
use rosbag::{ChunkRecord, MessageRecord, RosBag};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::time::Instant;
use std::path::{Path, PathBuf};

pub fn convert_bag(
    bag: &str,
    out: &str,
    include: Vec<String>,
    exclude: Vec<String>,
    start: Option<f64>,
    end: Option<f64>,
    dry_run: bool,
    progress: bool,
    segment_size: Option<usize>,
) -> Result<()> {
    let bag_file = RosBag::new(bag).with_context(|| format!("failed to open bag: {}", bag))?;

    // filters
    let include_set: Option<HashSet<&str>> = if include.is_empty() { None } else { Some(include.iter().map(|s| s.as_str()).collect()) };
    let exclude_set: HashSet<&str> = exclude.iter().map(|s| s.as_str()).collect();

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
    if let Some(sz) = segment_size { if sz == 0 { anyhow::bail!("segment-size must be > 0"); } }
    let segmentation_enabled = segment_size.is_some() && !dry_run;
    let seg_size = segment_size.unwrap_or(0) as u64;

    // Single-output recording (created lazily after first kept message for parity with segments)
    let mut rec: Option<rerun::RecordingStream> = None;

    // For segmentation derive base path components
    let (base_parent, base_stem, base_ext) = if segmentation_enabled {
        let p = Path::new(out);
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
    let mut current_segment_path = String::new();

    let open_new_segment = |segment_index: u64,
                            base_parent: &PathBuf,
                            base_stem: &str,
                            base_ext: &str,
                            bag: &str,
                            current_segment_path: &mut String|
     -> anyhow::Result<rerun::RecordingStream> {
        let seg_path = base_parent.join(format!("{}_part{:04}.{}", base_stem, segment_index + 1, base_ext));
        *current_segment_path = seg_path.to_string_lossy().to_string();
        let rec_id = format!("bag2rrd:{}:segment:{}", bag, segment_index + 1);
        eprintln!("[bag2rrd][segment {}] opening {}", segment_index + 1, current_segment_path);
        Ok(rerun::RecordingStreamBuilder::new(rec_id).save(&*current_segment_path)?)
    };

    // progress bar (unknown length)
    let pb = if progress { let pb = ProgressBar::new_spinner(); pb.set_style(ProgressStyle::with_template("{spinner} {pos} msgs").unwrap()); Some(pb) } else { None };

    let mut bag_start_ns = f64::INFINITY;
    let mut total_msgs: u64 = 0; let mut kept_msgs: u64 = 0; let mut topics: HashSet<String> = HashSet::new();

    // statistics and logging configuration
    #[derive(Default)]
    struct Stats { images: u64, compressed_images: u64, skipped_type: u64, filtered_out: u64, raw_bytes: u64 }
    let mut stats = Stats::default();
    let log_every = std::env::var("BAG2RRD_LOG_EVERY").ok().and_then(|s| s.parse::<u64>().ok()).filter(|v| *v > 0);
    let verbose_types: Option<HashSet<String>> = std::env::var("BAG2RRD_LOG_TYPES").ok().map(|s| s.split(',').map(|t| t.trim().to_string()).collect());
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

    let bag_start_s = if bag_start_ns.is_finite() { bag_start_ns / 1_000_000_000.0 } else { 0.0 };

    // Second pass: process messages
    println!("Starting second pass...");
    for record in &chunks {
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::MessageData(msg_data) = msg {
                    if let Some((topic, tp)) = connections.get(&msg_data.conn_id) {
                        // Apply filters
                        if let Some(inc) = &include_set { if !inc.contains(topic.as_str()) { continue; } }
                        if exclude_set.contains(topic.as_str()) { continue; }

                        let ts_rel = (msg_data.time as f64 / 1_000_000_000.0) - bag_start_s;
                        if let Some(s) = start { if ts_rel < s { continue; } }
                        if let Some(e) = end { if ts_rel > e { continue; } }

                        topics.insert(topic.clone());
                        if dry_run { kept_msgs += 1; if let Some(pb) = &pb { pb.inc(1); } continue; }

                        // ensure recording stream exists (single or segment)
                        if rec.is_none() {
                            if segmentation_enabled {
                                rec = Some(open_new_segment(segment_index, &base_parent, &base_stem, &base_ext, bag, &mut current_segment_path)?);
                            } else {
                                let rec_id = format!("bag2rrd:{}", bag);
                                rec = Some(rerun::RecordingStreamBuilder::new(rec_id).save(out)?);
                            }
                        }

                        // dispatch by type
                        match tp.as_str() {
                            "sensor_msgs/Image" => {
                                if let Some(ref rec_ref) = rec { crate::mappings::images::image_to_rerun(rec_ref, topic, ts_rel, msg_data.data)?; }
                                kept_msgs += 1; stats.images += 1; stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled { segment_images += 1; segment_raw_bytes += msg_data.data.len() as u64; }
                            }
                            "sensor_msgs/CompressedImage" => {
                                if let Some(ref rec_ref) = rec { crate::mappings::images::compressed_to_rerun(rec_ref, topic, ts_rel, msg_data.data)?; }
                                kept_msgs += 1; stats.compressed_images += 1; stats.raw_bytes += msg_data.data.len() as u64;
                                if segmentation_enabled { segment_images += 1; segment_raw_bytes += msg_data.data.len() as u64; }
                            }
                            _ => { stats.skipped_type += 1; }
                        }

                        // Segment rotation
                        if segmentation_enabled && seg_size > 0 && segment_images >= seg_size {
                            if let Some(rec_full) = rec.take() {
                                eprintln!("[bag2rrd][segment {}] flushing (images={} raw_bytes={})", segment_index + 1, segment_images, segment_raw_bytes);
                                flush_recording(rec_full, &current_segment_path, segment_raw_bytes, &format!("[bag2rrd][segment {}]", segment_index + 1));
                                eprintln!("[bag2rrd][segment {}] completed file={}", segment_index + 1, current_segment_path);
                                // prepare next
                                segment_index += 1; segment_images = 0; segment_raw_bytes = 0; current_segment_path.clear();
                            }
                        }
                        if let Some(pb) = &pb { pb.inc(1); }
                        if let Some(ref vt) = verbose_types { if vt.contains(tp) { eprintln!("[bag2rrd][msg] topic={topic} type={tp} t={:.6}", ts_rel); } }
                        if let Some(n) = log_every { if kept_msgs % n == 0 { eprintln!("[bag2rrd][progress] kept_msgs={} images={} compressed={} skipped_type={} filtered={} elapsed={:?}", kept_msgs, stats.images, stats.compressed_images, stats.skipped_type, stats.filtered_out, second_pass_start.elapsed()); } }
                    }
                    else { stats.filtered_out += 1; }
                }
            }
        }
    }

    if let Some(pb) = &pb { pb.finish_and_clear(); }
    println!("Second pass completed");

    println!("Plan: {} messages, {} kept after filters, {} topics → output: {}", total_msgs, kept_msgs, topics.len(), out);

    if !dry_run {
        eprintln!("[bag2rrd][stats] images={} compressed_images={} skipped_types={} filtered_out={} kept_msgs={} total_msgs={} raw_bytes={}", stats.images, stats.compressed_images, stats.skipped_type, stats.filtered_out, kept_msgs, total_msgs, stats.raw_bytes);
        if segmentation_enabled {
            // flush last open segment
            if let Some(rec_last) = rec.take() {
                if segment_images > 0 { // only if something was logged
                    eprintln!("[bag2rrd][segment {}] final flushing (images={} raw_bytes={})", segment_index + 1, segment_images, segment_raw_bytes);
                    flush_recording(rec_last, &current_segment_path, segment_raw_bytes, &format!("[bag2rrd][segment {}]", segment_index + 1));
                    eprintln!("[bag2rrd][segment {}] completed file={}", segment_index + 1, current_segment_path);
                }
            }
            let total_segments = segment_index + if segment_images > 0 {1} else {0};
            eprintln!("[bag2rrd] segmentation summary: segments={} segment_size={} total_images={} raw_bytes={} pattern='{}_part{{:04}}.{}'", total_segments, seg_size, stats.images + stats.compressed_images, stats.raw_bytes, base_stem, base_ext);
        } else {
            if let Some(rec_single) = rec.take() {
                eprintln!("[bag2rrd][single] flushing recording (images={} raw_bytes={})", stats.images + stats.compressed_images, stats.raw_bytes);
                flush_recording(rec_single, out, stats.raw_bytes, "[bag2rrd]");
                eprintln!("[bag2rrd] Saved RRD: {}", out);
            } else {
                // Could happen if no messages matched filters
                eprintln!("[bag2rrd] no messages kept; nothing to flush");
            }
        }
    }

    Ok(())
}

fn flush_recording(rec: rerun::RecordingStream, out_path: &str, raw_total: u64, prefix: &str) {
    use std::io::Write;
    let debug_timings = std::env::var("BAG2RRD_DEBUG_TIMINGS").ok().as_deref() == Some("1");
    let timeout_secs: u64 = std::env::var("BAG2RRD_FLUSH_TIMEOUT_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    if debug_timings { eprintln!("{prefix}[debug] flushing (timeout={}s, 0=forever) file={}", timeout_secs, out_path); }
    let t0 = if debug_timings { Some(std::time::Instant::now()) } else { None };
    let stop_flag = Arc::new(AtomicBool::new(false));
    let path = out_path.to_string();
    let monitor_flag = Arc::clone(&stop_flag);
    let poll_ms: u64 = std::env::var("BAG2RRD_FLUSH_POLL_MS").ok().and_then(|s| s.parse().ok()).unwrap_or(500);
    let prefix_owned = prefix.to_string();
    let monitor_handle = std::thread::spawn(move || {
        let mut last_size = 0u64; let mut last_change = Instant::now();
        while !monitor_flag.load(Ordering::Relaxed) {
            match std::fs::metadata(&path) {
                Ok(meta) => {
                    let size = meta.len();
                    if size != last_size {
                        let delta = size.saturating_sub(last_size);
                        let pct = if raw_total > 0 { (size as f64 / raw_total as f64 * 100.0).min(100.0) } else { 0.0 };
                        eprintln!("{}[flush] size={} (+{}) est_progress={:.1}%", prefix_owned, size, delta, pct);
                        last_size = size; last_change = Instant::now();
                    } else if last_change.elapsed() > std::time::Duration::from_secs(5) {
                        eprintln!("{}[flush] no size change for 5s (size={})", prefix_owned, size);
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
    std::thread::spawn(move || { drop(rec); let _ = tx.send(()); });
    if timeout_secs == 0 { let _ = rx.recv(); } else {
        if rx.recv_timeout(std::time::Duration::from_secs(timeout_secs)).is_err() {
            eprintln!("{prefix}[warn] timeout waiting for flush; file may be incomplete");
        }
    }
    stop_flag.store(true, Ordering::Relaxed);
    let _ = monitor_handle.join();
    std::io::stdout().flush().ok(); std::io::stderr().flush().ok();
    if let Some(t0) = t0 { if debug_timings { eprintln!("{prefix}[debug] flush completed in {:?}", t0.elapsed()); } }
}