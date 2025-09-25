use anyhow::{Context, Result};
use rosbag::{ChunkRecord, MessageRecord, RosBag};
use std::collections::BTreeMap;

/// Diagnose bag file issues
pub fn diagnose_bag(path: &str) -> Result<()> {
    tracing::info!("Starting bag diagnosis for: {}", path);

    // Try to open the bag
    tracing::debug!("Opening bag file...");
    let bag = match RosBag::new(path) {
        Ok(bag) => {
            tracing::info!("Bag file opened successfully");
            bag
        }
        Err(e) => {
            tracing::error!("Failed to open bag file: {}", e);
            return Err(e.into());
        }
    };

    // Count total chunks without reading them
    tracing::debug!("Counting chunks...");
    let mut total_chunks = 0;
    let mut successful_chunks = 0;

    for record in bag.chunk_records() {
        total_chunks += 1;

        if record.is_ok() {
            successful_chunks += 1;
        } else {
            tracing::error!("Failed to read chunk #{}: {:?}", total_chunks, record.err());
            break;
        }

        // Log progress every 1000 chunks
        if total_chunks % 1000 == 0 {
            tracing::info!("Processed {} chunks successfully", total_chunks);
        }
    }

    tracing::info!("Diagnosis complete:");
    tracing::info!("  Total chunks attempted: {}", total_chunks);
    tracing::info!("  Successful chunks: {}", successful_chunks);
    tracing::info!("  Failed chunks: {}", total_chunks - successful_chunks);

    if successful_chunks < total_chunks {
        tracing::error!("Bag file appears to be corrupted or truncated");
        tracing::info!("File is likely incomplete - missing data after chunk {}", successful_chunks);
    } else {
        tracing::info!("Bag file structure appears intact");
    }

    Ok(())
}

#[allow(clippy::collapsible_if)]
pub fn inspect_bag(path: &str) -> Result<()> {
    tracing::info!("Starting bag inspection for: {}", path);

    // 1) open bag
    tracing::debug!("Opening bag file...");
    let bag = RosBag::new(path).with_context(|| format!("failed to open bag: {}", path))?;
    tracing::info!("Bag file opened successfully");

    // 2) collect connections first
    tracing::debug!("Collecting connections...");
    let mut connections = BTreeMap::new();
    let mut chunk_count = 0;
    let mut connection_count = 0;

    for record in bag.chunk_records() {
        chunk_count += 1;
        tracing::debug!("Processing chunk record #{}", chunk_count);

        let record = match record.with_context(|| format!("failed to read chunk record #{}", chunk_count)) {
            Ok(record) => record,
            Err(e) => {
                tracing::error!("Error reading chunk #{}: {}", chunk_count, e);
                tracing::info!("Total chunks processed before error: {}", chunk_count - 1);
                return Err(e);
            }
        };

        if let ChunkRecord::Chunk(chunk) = record {
            tracing::debug!("Found chunk with {} messages", chunk.messages().count());

            for msg in chunk.messages() {
                let msg = msg.with_context(|| format!("failed to read message in chunk #{}", chunk_count))?;

                if let MessageRecord::Connection(conn) = msg {
                    connection_count += 1;
                    connections.insert(conn.id, (conn.topic.to_string(), conn.tp.to_string()));
                    tracing::debug!("Found connection: {} -> {}", conn.topic, conn.tp);
                }
            }
        }
    }

    tracing::info!("Collected {} connections from {} chunk records", connection_count, chunk_count);

    // 3) gather stats from messages
    tracing::debug!("Gathering message statistics...");
    #[derive(Default)]
    struct Stat {
        ty: String,
        count: u64,
        first: f64,
        last: f64,
    }
    let mut stats: BTreeMap<String, Stat> = BTreeMap::new();

    let mut bag_start_ns = f64::INFINITY;
    let mut global_first = f64::INFINITY;
    let mut global_last = 0.0_f64;
    let mut total: u64 = 0;
    let mut message_chunk_count = 0;

    for record in bag.chunk_records() {
        message_chunk_count += 1;
        tracing::debug!("Processing message chunk #{}", message_chunk_count);

        let record = record.with_context(|| format!("failed to read message chunk record #{}", message_chunk_count))?;

        if let ChunkRecord::Chunk(chunk) = record {
            tracing::debug!("Message chunk #{} has {} messages", message_chunk_count, chunk.messages().count());

            for msg in chunk.messages() {
                let msg = msg.with_context(|| format!("failed to read message in chunk #{}", message_chunk_count))?;

                if let MessageRecord::MessageData(msg_data) = msg {
                    total += 1;

                    if let Some((topic, tp)) = connections.get(&msg_data.conn_id) {
                        let ts_ns = msg_data.time as f64;
                        bag_start_ns = bag_start_ns.min(ts_ns);
                        let ts = ts_ns / 1_000_000_000.0; // convert to seconds
                        let entry = stats.entry(topic.clone()).or_insert_with(|| Stat {
                            ty: tp.clone(),
                            count: 0,
                            first: ts,
                            last: ts,
                        });
                        entry.count += 1;
                        entry.first = entry.first.min(ts);
                        entry.last = entry.last.max(ts);
                        global_first = global_first.min(ts);
                        global_last = global_last.max(ts);
                        total += 1;
                    }
                }
            }
        }
    }

    // Calculate relative times from bag start
    if bag_start_ns.is_finite() {
        let bag_start_s = bag_start_ns / 1_000_000_000.0;
        global_first -= bag_start_s;
        global_last -= bag_start_s;
        for stat in stats.values_mut() {
            stat.first -= bag_start_s;
            stat.last -= bag_start_s;
        }
    }

    let duration = if global_last.is_finite() {
        global_last - global_first
    } else {
        0.0
    };

    // 4) print summary
    println!("Bag: {}", path);
    println!(
        "Start (s): {:.6}, End (s): {:.6}, Duration (s): {:.6}, Total messages: {}\n",
        global_first, global_last, duration, total
    );

    // 5) table
    println!(
        "{:<35} {:<35} {:>7} {:>10} {:>10}",
        "Topic", "Type", "Count", "Start(s)", "End(s)"
    );
    println!("{}", "-".repeat(97));
    for (topic, st) in stats.iter() {
        println!(
            "{:<35} {:<35} {:>7} {:>10.6} {:>10.6}",
            topic, st.ty, st.count, st.first, st.last
        );
    }

    Ok(())
}
