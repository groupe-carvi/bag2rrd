//! PointCloud2 → Rerun Points3D (implemented in v0.2.0)

use anyhow::Result;
use rerun::components::Position3D;

pub fn pointcloud2_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    let (positions, colors) = parse_pointcloud2(payload)?;

    let rr_path = normalize_path(topic);
    let pts = rerun::archetypes::Points3D::new(positions);
    let pts = if let Some(colors) = colors {
        pts.with_colors(colors)
    } else {
        pts
    };
    rec.log(rr_path, &pts)?;

    Ok(())
}

#[allow(clippy::type_complexity, clippy::collapsible_if)]
pub fn parse_pointcloud2(payload: &[u8]) -> Result<(Vec<Position3D>, Option<Vec<[u8; 3]>>)> {
    let mut cursor = 0;

    // Parse header (std_msgs/Header) - skip for now
    cursor = skip_header(payload, cursor)?;

    // height (uint32)
    let height = read_u32_le(payload, &mut cursor)?;
    // width (uint32)
    let width = read_u32_le(payload, &mut cursor)?;

    // fields (array of PointField)
    let fields = parse_fields(payload, &mut cursor)?;

    // is_bigendian (bool)
    let is_bigendian = read_bool(payload, &mut cursor)?;
    if is_bigendian {
        tracing::warn!("Big-endian PointCloud2 not supported; skipping");
        return Ok((vec![], None));
    }

    // point_step (uint32)
    let point_step = read_u32_le(payload, &mut cursor)? as usize;
    // row_step (uint32)
    let _row_step = read_u32_le(payload, &mut cursor)?;

    // data length (uint32)
    let data_len = read_u32_le(payload, &mut cursor)? as usize;
    if payload.len() < cursor + data_len {
        return Err(anyhow::anyhow!("payload too short for data"));
    }
    let data = &payload[cursor..cursor + data_len];
    // cursor += data_len; // not needed

    // is_dense (bool) - skip

    // Find x, y, z offsets
    let x_off = fields
        .iter()
        .find(|f| f.name == "x")
        .map(|f| f.offset as usize);
    let y_off = fields
        .iter()
        .find(|f| f.name == "y")
        .map(|f| f.offset as usize);
    let z_off = fields
        .iter()
        .find(|f| f.name == "z")
        .map(|f| f.offset as usize);

    if x_off.is_none() || y_off.is_none() || z_off.is_none() {
        tracing::warn!("PointCloud2 missing x/y/z fields; skipping");
        return Ok((vec![], None));
    }

    let x_off = x_off.unwrap();
    let y_off = y_off.unwrap();
    let z_off = z_off.unwrap();

    // Color offset
    let color_off = fields
        .iter()
        .find(|f| f.name == "rgb" || f.name == "rgba")
        .map(|f| f.offset as usize);

    let mut positions = Vec::new();
    let mut colors = if color_off.is_some() {
        Some(Vec::new())
    } else {
        None
    };

    for i in 0..(height * width) {
        let point_start = i as usize * point_step;
        if point_start + point_step > data.len() {
            break;
        }
        let point = &data[point_start..point_start + point_step];

        // Read x, y, z as f32
        let x = read_f32_le_at(point, x_off)?;
        let y = read_f32_le_at(point, y_off)?;
        let z = read_f32_le_at(point, z_off)?;

        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            continue; // skip NaN/Inf
        }

        positions.push(Position3D::new(x, y, z));

        if let Some(colors_vec) = &mut colors {
            if let Some(off) = color_off {
                let color = read_color_at(point, off)?;
                colors_vec.push(color);
            }
        }
    }

    Ok((positions, colors))
}

#[derive(Debug)]
#[allow(dead_code)]
struct PointField {
    name: String,
    offset: u32,
    datatype: u8,
    count: u32,
}

fn parse_fields(payload: &[u8], cursor: &mut usize) -> Result<Vec<PointField>> {
    // array length (uint32)
    let len = read_u32_le(payload, cursor)? as usize;
    let mut fields = Vec::with_capacity(len);
    for _ in 0..len {
        // name (string)
        let name = read_string(payload, cursor)?;
        // offset (uint32)
        let offset = read_u32_le(payload, cursor)?;
        // datatype (uint8)
        let datatype = read_u8(payload, cursor)?;
        // count (uint32)
        let count = read_u32_le(payload, cursor)?;
        fields.push(PointField {
            name,
            offset,
            datatype,
            count,
        });
    }
    Ok(fields)
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

fn read_u8(payload: &[u8], cursor: &mut usize) -> Result<u8> {
    if *cursor + 1 > payload.len() {
        return Err(anyhow::anyhow!("payload too short"));
    }
    let val = payload[*cursor];
    *cursor += 1;
    Ok(val)
}

fn read_bool(payload: &[u8], cursor: &mut usize) -> Result<bool> {
    let val = read_u8(payload, cursor)?;
    Ok(val != 0)
}

fn read_string(payload: &[u8], cursor: &mut usize) -> Result<String> {
    let len = read_u32_le(payload, cursor)? as usize;
    if *cursor + len > payload.len() {
        return Err(anyhow::anyhow!("payload too short for string"));
    }
    let s = String::from_utf8_lossy(&payload[*cursor..*cursor + len]).to_string();
    *cursor += len;
    Ok(s)
}

fn read_f32_le_at(data: &[u8], off: usize) -> Result<f32> {
    if off + 4 > data.len() {
        return Err(anyhow::anyhow!("data too short"));
    }
    let bytes = [data[off], data[off + 1], data[off + 2], data[off + 3]];
    Ok(f32::from_le_bytes(bytes))
}

fn read_color_at(data: &[u8], off: usize) -> Result<[u8; 3]> {
    if off + 4 > data.len() {
        return Err(anyhow::anyhow!("data too short"));
    }
    // Assume float32 packed RGB or RGBA
    let packed = f32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]);
    let packed_u32 = packed.to_bits();
    let r = ((packed_u32 >> 16) & 0xFF) as u8;
    let g = ((packed_u32 >> 8) & 0xFF) as u8;
    let b = (packed_u32 & 0xFF) as u8;
    Ok([r, g, b])
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
    fn test_parse_pointcloud2() {
        // Create a synthetic PC2 buffer with x,y,z,rgb fields
        let mut data = Vec::new();

        // Header
        data.extend_from_slice(&0u32.to_le_bytes()); // seq
        data.extend_from_slice(&0u32.to_le_bytes()); // stamp sec
        data.extend_from_slice(&0u32.to_le_bytes()); // stamp nsec
        data.extend_from_slice(&0u32.to_le_bytes()); // frame_id len
        // height
        data.extend_from_slice(&1u32.to_le_bytes());
        // width
        data.extend_from_slice(&4u32.to_le_bytes());
        // fields: x,y,z,rgb
        let fields = vec![
            ("x", 0u32, 7u8, 1u32), // FLOAT32
            ("y", 4u32, 7, 1),
            ("z", 8u32, 7, 1),
            ("rgb", 12u32, 7, 1),
        ];
        data.extend_from_slice(&(fields.len() as u32).to_le_bytes());
        for (name, offset, datatype, count) in fields {
            data.extend_from_slice(&(name.len() as u32).to_le_bytes());
            data.extend_from_slice(name.as_bytes());
            data.extend_from_slice(&offset.to_le_bytes());
            data.extend_from_slice(&datatype.to_le_bytes());
            data.extend_from_slice(&count.to_le_bytes());
        }
        // is_bigendian
        data.extend_from_slice(&0u8.to_le_bytes());
        // point_step
        data.extend_from_slice(&16u32.to_le_bytes());
        // row_step
        data.extend_from_slice(&64u32.to_le_bytes());
        // data len
        let points_data_len = 4 * 16;
        data.extend_from_slice(&(points_data_len as u32).to_le_bytes());
        // points data: 4 points with x,y,z,rgb
        for i in 0..4 {
            let x = i as f32;
            let y = (i + 1) as f32;
            let z = (i + 2) as f32;
            let rgb = ((i as u32) << 16) | ((i as u32) << 8) | i as u32; // packed
            let rgb_f32 = f32::from_bits(rgb);
            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&rgb_f32.to_le_bytes());
        }
        // is_dense
        data.extend_from_slice(&1u8.to_le_bytes());

        let (positions, colors) = parse_pointcloud2(&data).unwrap();

        assert_eq!(positions.len(), 4);
        assert_eq!(colors.as_ref().unwrap().len(), 4);
        assert_eq!(positions[0], Position3D::new(0.0, 1.0, 2.0));
        assert_eq!(colors.as_ref().unwrap()[0], [0, 0, 0]);
    }
}
