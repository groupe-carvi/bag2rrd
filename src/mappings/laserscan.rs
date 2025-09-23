//! LaserScan → Rerun Points2D or LineStrips2D (implemented in v0.2.0)

use anyhow::Result;

pub fn laserscan_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
    as_lines: bool,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    let points = parse_laserscan(payload)?;

    let rr_path = normalize_path(topic);
    if as_lines {
        // Create a single LineStrip2D with contiguous valid points
        let mut strips = vec![vec![]];
        for &pt in &points {
            if pt.0.is_finite() && pt.1.is_finite() {
                strips.last_mut().unwrap().push([pt.0, pt.1]);
            } else {
                // Start new strip on invalid
                if !strips.last().unwrap().is_empty() {
                    strips.push(vec![]);
                }
            }
        }
        if strips.last().unwrap().is_empty() {
            strips.pop();
        }
        if !strips.is_empty() {
            let line_strips = rerun::archetypes::LineStrips2D::new(strips);
            rec.log(rr_path, &line_strips)?;
        }
    } else {
        let valid_points: Vec<[f32; 2]> = points
            .into_iter()
            .filter(|p| p.0.is_finite() && p.1.is_finite())
            .map(|p| [p.0, p.1])
            .collect();
        let pts = rerun::archetypes::Points2D::new(valid_points);
        rec.log(rr_path, &pts)?;
    }

    Ok(())
}

pub fn parse_laserscan(payload: &[u8]) -> Result<Vec<(f32, f32)>> {
    let mut cursor = 0;

    // Skip header
    cursor = skip_header(payload, cursor)?;

    // angle_min (float32)
    let angle_min = read_f32_le(payload, &mut cursor)?;
    // angle_max (float32)
    let _angle_max = read_f32_le(payload, &mut cursor)?;
    // angle_increment (float32)
    let angle_increment = read_f32_le(payload, &mut cursor)?;
    // time_increment (float32) - skip
    cursor += 4;
    // scan_time (float32) - skip
    cursor += 4;
    // range_min (float32)
    let range_min = read_f32_le(payload, &mut cursor)?;
    // range_max (float32)
    let range_max = read_f32_le(payload, &mut cursor)?;

    // ranges length (uint32)
    let ranges_len = read_u32_le(payload, &mut cursor)? as usize;
    // intensities length (uint32) - skip
    let _intensities_len = read_u32_le(payload, &mut cursor)?;

    // ranges (float32[])
    let mut ranges = Vec::with_capacity(ranges_len);
    for _ in 0..ranges_len {
        let r = read_f32_le(payload, &mut cursor)?;
        ranges.push(r);
    }

    // Compute points
    let mut points = Vec::new();
    for (i, &r) in ranges.iter().enumerate() {
        if r.is_finite() && r >= range_min && r <= range_max {
            let angle = angle_min + i as f32 * angle_increment;
            let x = r * angle.cos();
            let y = r * angle.sin();
            points.push((x, y));
        } else {
            points.push((f32::NAN, f32::NAN)); // invalid
        }
    }

    Ok(points)
}

fn read_f32_le(payload: &[u8], cursor: &mut usize) -> Result<f32> {
    if *cursor + 4 > payload.len() {
        return Err(anyhow::anyhow!("payload too short"));
    }
    let val = f32::from_le_bytes([
        payload[*cursor],
        payload[*cursor + 1],
        payload[*cursor + 2],
        payload[*cursor + 3],
    ]);
    *cursor += 4;
    Ok(val)
}

fn read_u32_le(payload: &[u8], cursor: &mut usize) -> Result<u32> {
    if *cursor + 4 > payload.len() {
        return Err(anyhow::anyhow!("payload too short"));
    }
    let val = u32::from_le_bytes([
        payload[*cursor],
        payload[*cursor + 1],
        payload[*cursor + 2],
        payload[*cursor + 3],
    ]);
    *cursor += 4;
    Ok(val)
}

fn skip_header(payload: &[u8], mut cursor: usize) -> Result<usize> {
    // seq (uint32)
    cursor += 4;
    // stamp (uint32 + uint32)
    cursor += 8;
    // frame_id (string)
    let len = read_u32_le(payload, &mut cursor)? as usize;
    cursor += len;
    Ok(cursor)
}

fn normalize_path(topic: &str) -> String {
    topic.trim_start_matches('/').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_laserscan() {
        // Create a synthetic LaserScan buffer
        let mut data = Vec::new();

        // Header
        data.extend_from_slice(&0u32.to_le_bytes()); // seq
        data.extend_from_slice(&0u32.to_le_bytes()); // stamp sec
        data.extend_from_slice(&0u32.to_le_bytes()); // stamp nsec
        data.extend_from_slice(&0u32.to_le_bytes()); // frame_id len
        // angle_min
        data.extend_from_slice(&(-1.0f32).to_le_bytes());
        // angle_max
        data.extend_from_slice(&1.0f32.to_le_bytes());
        // angle_increment
        data.extend_from_slice(&0.1f32.to_le_bytes());
        // time_increment
        data.extend_from_slice(&0.0f32.to_le_bytes());
        // scan_time
        data.extend_from_slice(&0.0f32.to_le_bytes());
        // range_min
        data.extend_from_slice(&0.1f32.to_le_bytes());
        // range_max
        data.extend_from_slice(&10.0f32.to_le_bytes());
        // ranges len
        let ranges_len = 10;
        data.extend_from_slice(&(ranges_len as u32).to_le_bytes());
        // intensities len
        data.extend_from_slice(&0u32.to_le_bytes());
        // ranges
        for i in 0..ranges_len {
            let r = if i % 2 == 0 { 1.0f32 } else { f32::NAN };
            data.extend_from_slice(&r.to_le_bytes());
        }

        let points = parse_laserscan(&data).unwrap();

        assert_eq!(points.len(), 10);
        // Check some points
        assert!(points[0].0.is_finite() && points[0].1.is_finite());
        assert!(points[1].0.is_nan() || points[1].1.is_nan());
    }
}
