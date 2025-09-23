use anyhow::{Context, Result};
use std::collections::BTreeMap;
use rosbag::{ChunkRecord, MessageRecord, RosBag};

pub fn inspect_bag(path: &str) -> Result<()> {
    // 1) open bag
    let bag = RosBag::new(path).with_context(|| format!("failed to open bag: {}", path))?;

    // 2) collect connections first
    let mut connections = BTreeMap::new();
    for record in bag.chunk_records() {
        let record = record?;
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::Connection(conn) = msg {
                    connections.insert(conn.id, (conn.topic.to_string(), conn.tp.to_string()));
                }
            }
        }
    }

    // 3) gather stats from messages
    #[derive(Default)]
    struct Stat { ty: String, count: u64, first: f64, last: f64 }
    let mut stats: BTreeMap<String, Stat> = BTreeMap::new();

    let mut bag_start_ns = f64::INFINITY;
    let mut global_first = f64::INFINITY; let mut global_last = 0.0_f64; let mut total: u64 = 0;

    for record in bag.chunk_records() {
        let record = record?;
        if let ChunkRecord::Chunk(chunk) = record {
            for msg in chunk.messages() {
                let msg = msg?;
                if let MessageRecord::MessageData(msg_data) = msg {
                    if let Some((topic, tp)) = connections.get(&msg_data.conn_id) {
                        let ts_ns = msg_data.time as f64;
                        bag_start_ns = bag_start_ns.min(ts_ns);
                        let ts = ts_ns / 1_000_000_000.0; // convert to seconds
                        let entry = stats.entry(topic.clone()).or_insert_with(|| Stat { ty: tp.clone(), count: 0, first: ts, last: ts });
                        entry.count += 1; entry.first = entry.first.min(ts); entry.last = entry.last.max(ts);
                        global_first = global_first.min(ts); global_last = global_last.max(ts); total += 1;
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

    let duration = if global_last.is_finite() { global_last - global_first } else { 0.0 };

    // 4) print summary
    println!("Bag: {}", path);
    println!("Start (s): {:.6}, End (s): {:.6}, Duration (s): {:.6}, Total messages: {}\n", global_first, global_last, duration, total);

    // 5) table
    println!("{:<35} {:<35} {:>7} {:>10} {:>10}", "Topic", "Type", "Count", "Start(s)", "End(s)");
    println!("{}", "-".repeat(97));
    for (topic, st) in stats.iter() {
        println!("{:<35} {:<35} {:>7} {:>10.6} {:>10.6}", topic, st.ty, st.count, st.first, st.last);
    }

    Ok(())
}