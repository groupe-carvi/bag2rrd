use anyhow::{Context, Result};
use image::{DynamicImage, ImageFormat};

pub fn image_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    match parse_ros_image(payload) {
        Ok((width, height, encoding, data)) => {
            let rr_path = normalize_path(topic);
            match encoding.as_str() {
                "rgb8" => {
                    let img = rerun::archetypes::Image::from_rgb24(
                        data.to_vec(),
                        [width as u32, height as u32],
                    );
                    rec.log(rr_path, &img)?;
                }
                "bgr8" => {
                    let mut buf = data.to_vec();
                    for px in buf.chunks_exact_mut(3) {
                        px.swap(0, 2);
                    } // BGR→RGB
                    let img =
                        rerun::archetypes::Image::from_rgb24(buf, [width as u32, height as u32]);
                    rec.log(rr_path, &img)?;
                }
                "rgba8" => {
                    let mut rgb = Vec::with_capacity(width * height * 3);
                    for px in data.chunks_exact(4) {
                        rgb.extend_from_slice(&px[..3]);
                    }
                    let img =
                        rerun::archetypes::Image::from_rgb24(rgb, [width as u32, height as u32]);
                    rec.log(rr_path, &img)?;
                }
                "mono8" => {
                    // Convert mono to RGB for now
                    let mut rgb = Vec::with_capacity(width * height * 3);
                    for &gray in data {
                        rgb.extend_from_slice(&[gray, gray, gray]);
                    }
                    let img =
                        rerun::archetypes::Image::from_rgb24(rgb, [width as u32, height as u32]);
                    rec.log(rr_path, &img)?;
                }
                "mono16" => {
                    // For v0.1.0, scale down to 8-bit with a warning
                    tracing::warn!("mono16 not natively supported in v0.1.0; scaling to 8-bit");
                    let mut rgb = Vec::with_capacity(width * height * 3);
                    for chunk in data.chunks_exact(2) {
                        let v = u16::from_le_bytes([chunk[0], chunk[1]]);
                        let gray = (v >> 8) as u8;
                        rgb.extend_from_slice(&[gray, gray, gray]);
                    }
                    let img =
                        rerun::archetypes::Image::from_rgb24(rgb, [width as u32, height as u32]);
                    rec.log(rr_path, &img)?;
                }
                other => {
                    tracing::warn!(%other, "unsupported encoding; skipping");
                }
            }
        }
        Err(e) => {
            tracing::warn!("Failed to parse ROS image message: {}; skipping", e);
        }
    }
    Ok(())
}

pub fn compressed_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);
    match parse_ros_compressed(payload) {
        Ok((fmt, bytes)) => {
            let fmt_lc = fmt.to_ascii_lowercase();

            let dyn_img: DynamicImage = if fmt_lc.contains("png") {
                image::load_from_memory_with_format(bytes, ImageFormat::Png)
                    .context("decode png")?
            } else if fmt_lc.contains("jpg") || fmt_lc.contains("jpeg") {
                image::load_from_memory_with_format(bytes, ImageFormat::Jpeg)
                    .context("decode jpeg")?
            } else {
                tracing::warn!(format=%fmt, "unsupported compressed image format; skipping");
                return Ok(());
            };

            let rgb8 = dyn_img.to_rgb8();
            let width = rgb8.width();
            let height = rgb8.height();
            let rr_path = normalize_path(topic);
            let img = rerun::archetypes::Image::from_rgb24(rgb8.into_raw(), [width, height]);
            rec.log(rr_path, &img)?;
        }
        Err(e) => {
            tracing::warn!(
                "Failed to parse ROS compressed image message: {}; skipping",
                e
            );
        }
    }
    Ok(())
}

fn normalize_path(topic: &str) -> String {
    if topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("/{}", topic)
    }
}

// ROS message parsing helpers
fn parse_ros_image(payload: &[u8]) -> Result<(usize, usize, String, &[u8])> {
    // Debug: log first 20 bytes
    tracing::debug!(
        "Parsing ROS image, payload length: {}, first 20 bytes: {:?}",
        payload.len(),
        &payload[..payload.len().min(20)]
    );

    let mut cursor = 0;

    // Parse header (std_msgs/Header)
    // seq (uint32)
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for header seq"));
    }
    let _seq = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]);
    cursor += 4;

    // stamp (time): secs (uint32), nsecs (uint32)
    if payload.len() < cursor + 8 {
        return Err(anyhow::anyhow!("payload too short for header stamp"));
    }
    let _secs = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]);
    let _nsecs = u32::from_le_bytes([
        payload[cursor + 4],
        payload[cursor + 5],
        payload[cursor + 6],
        payload[cursor + 7],
    ]);
    cursor += 8;

    // frame_id (string): length (uint32) + chars
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for frame_id length"));
    }
    let frame_id_len = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]) as usize;
    cursor += 4;

    if payload.len() < cursor + frame_id_len {
        return Err(anyhow::anyhow!("payload too short for frame_id"));
    }
    let _frame_id = String::from_utf8_lossy(&payload[cursor..cursor + frame_id_len]).to_string();
    cursor += frame_id_len;

    // height (uint32)
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for height"));
    }
    let height = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]) as usize;
    cursor += 4;

    // width (uint32)
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for width"));
    }
    let width = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]) as usize;
    cursor += 4;

    // encoding (string): length (uint32) + chars
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for encoding length"));
    }
    let encoding_len = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]) as usize;
    cursor += 4;

    if payload.len() < cursor + encoding_len {
        return Err(anyhow::anyhow!("payload too short for encoding"));
    }
    let encoding = String::from_utf8_lossy(&payload[cursor..cursor + encoding_len]).to_string();
    cursor += encoding_len;

    // is_bigendian (uint8)
    if payload.len() < cursor + 1 {
        return Err(anyhow::anyhow!("payload too short for is_bigendian"));
    }
    let _is_bigendian = payload[cursor];
    cursor += 1;

    // step (uint32)
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for step"));
    }
    let _step = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]);
    cursor += 4;

    // data (uint8[]): length (uint32) + bytes
    if payload.len() < cursor + 4 {
        return Err(anyhow::anyhow!("payload too short for data length"));
    }
    let data_len = u32::from_le_bytes([
        payload[cursor],
        payload[cursor + 1],
        payload[cursor + 2],
        payload[cursor + 3],
    ]) as usize;
    cursor += 4;

    if payload.len() < cursor + data_len {
        return Err(anyhow::anyhow!("payload too short for data"));
    }
    let data = &payload[cursor..cursor + data_len];

    // Validate dimensions
    if height == 0 || width == 0 || height > 10000 || width > 10000 {
        return Err(anyhow::anyhow!(
            "invalid image dimensions: {}x{}",
            width,
            height
        ));
    }

    tracing::debug!(
        "Successfully parsed image: {}x{} {}, data size: {}",
        width,
        height,
        encoding,
        data.len()
    );
    Ok((width, height, encoding, data))
}

fn parse_ros_compressed(payload: &[u8]) -> Result<(String, &[u8])> {
    let mut cursor = 0;

    // Find format (string: uint32 length + chars)
    if payload.len() < 4 {
        return Err(anyhow::anyhow!("payload too short for format length"));
    }
    let fmt_len = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as usize;
    cursor += 4;

    if payload.len() < cursor + fmt_len {
        return Err(anyhow::anyhow!("payload too short for format"));
    }
    let format = String::from_utf8_lossy(&payload[cursor..cursor + fmt_len]).to_string();
    cursor += fmt_len;

    // The rest should be data
    if cursor >= payload.len() {
        return Err(anyhow::anyhow!("no data found"));
    }
    let data = &payload[cursor..];

    Ok((format, data))
}
