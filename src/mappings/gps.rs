//! NavSatFix → Rerun Points3D + LineStrips3D (implemented in v0.2.0)

use anyhow::Result;
use once_cell::sync::Lazy;
use std::sync::Mutex;

static GPS_STATE: Lazy<Mutex<GpsState>> = Lazy::new(|| Mutex::new(GpsState::default()));

#[derive(Default)]
struct GpsState {
    origin: Option<nalgebra::Point3<f64>>,
    path_points: Vec<[f32; 3]>,
}

pub fn navsatfix_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
    gps_origin: Option<&str>,
    gps_path: bool,
    geoid_path: Option<&str>,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

            let (lat, lon, mut alt, status, service) = parse_navsatfix(payload)?;

    // Apply geoid correction if grid file provided
    if let Some(geoid_path) = geoid_path {
        if let Ok(correction) = get_geoid_correction(geoid_path, lat, lon) {
            alt += correction;
            tracing::debug!("Applied geoid correction: {}m at ({}, {})", correction, lat, lon);
        } else {
            tracing::warn!("Failed to apply geoid correction from {}", geoid_path);
        }
    }

    if status.status < 0 {
        tracing::warn!("GPS fix status < 0; skipping");
        return Ok(());
    }

    let mut state = GPS_STATE.lock().unwrap();

    // Set origin if not set
    if state.origin.is_none() {
        let origin = if let Some(origin_str) = gps_origin {
            parse_origin(origin_str)?
        } else {
            nalgebra::Point3::new(lat, lon, alt)
        };
        state.origin = Some(origin);
    }

    let origin = state.origin.as_ref().unwrap();
    let enu = wgs84_to_enu(lat, lon, alt, origin.x, origin.y, origin.z)?;

    let pos_arr = [enu.0 as f32, enu.1 as f32, enu.2 as f32];

    // Log points
    let rr_path_points = format!("{}/points", normalize_path(topic).trim_end_matches('/'));
    let pts = rerun::archetypes::Points3D::new(vec![pos_arr]);
    rec.log(rr_path_points, &pts)?;

    // Log GPS status and service as scalars
    let rr_path_status = format!("{}/status", normalize_path(topic).trim_end_matches('/'));
    rec.log(rr_path_status, &rerun::archetypes::Scalars::new(vec![status.status as f64]))?;

    // Log service as categorical if possible, otherwise as scalar
    let service_names = get_service_names(service);
    if !service_names.is_empty() {
        let rr_path_service = format!("{}/service", normalize_path(topic).trim_end_matches('/'));
        rec.log(rr_path_service, &rerun::archetypes::TextLog::new(service_names))?;
    }

    // Log path
    if gps_path {
        state.path_points.push(pos_arr);
        let rr_path_path = format!("{}/path", normalize_path(topic).trim_end_matches('/'));
        let line_strips = rerun::archetypes::LineStrips3D::new(vec![state.path_points.clone()]);
        rec.log(rr_path_path, &line_strips)?;
    }

    Ok(())
}

fn parse_navsatfix(payload: &[u8]) -> Result<(f64, f64, f64, Status, u16)> {
    let mut cursor = 0;

    // Skip header
    cursor = skip_header(payload, cursor)?;

    // status (NavSatStatus) - contains status and service
    let (status, service) = parse_status(payload, &mut cursor)?;

    // latitude (float64)
    let lat = read_f64_le(payload, &mut cursor)?;
    // longitude (float64)
    let lon = read_f64_le(payload, &mut cursor)?;
    // altitude (float64)
    let alt = read_f64_le(payload, &mut cursor)?;

    Ok((lat, lon, alt, status, service))
}

#[derive(Debug)]
struct Status {
    status: i8,
}

fn parse_status(payload: &[u8], cursor: &mut usize) -> Result<(Status, u16)> {
    // status (int8)
    let status = read_i8(payload, cursor)?;
    // service (uint16)
    let service = read_u16_le(payload, cursor)?;
    Ok((Status { status }, service))
}

fn parse_origin(s: &str) -> Result<nalgebra::Point3<f64>> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 {
        return Err(anyhow::anyhow!("Invalid GPS origin format"));
    }
    let lat = parts[0].trim().parse()?;
    let lon = parts[1].trim().parse()?;
    let alt = parts[2].trim().parse()?;
    Ok(nalgebra::Point3::new(lat, lon, alt))
}

pub fn wgs84_to_enu(
    lat: f64,
    lon: f64,
    alt: f64,
    lat0: f64,
    lon0: f64,
    alt0: f64,
) -> Result<(f64, f64, f64)> {
    // Simple ENU transformation using ECEF intermediate
    // WGS84 ellipsoid parameters
    let a = 6378137.0; // semi-major axis
    let e2 = 0.00669437999014; // eccentricity squared

    // Convert to ECEF
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let n = a / (1.0 - e2 * lat_rad.sin().powi(2)).sqrt();
    let x = (n + alt) * lat_rad.cos() * lon_rad.cos();
    let y = (n + alt) * lat_rad.cos() * lon_rad.sin();
    let z = (n * (1.0 - e2) + alt) * lat_rad.sin();

    // Origin to ECEF
    let lat0_rad = lat0.to_radians();
    let lon0_rad = lon0.to_radians();
    let n0 = a / (1.0 - e2 * lat0_rad.sin().powi(2)).sqrt();
    let x0 = (n0 + alt0) * lat0_rad.cos() * lon0_rad.cos();
    let y0 = (n0 + alt0) * lat0_rad.cos() * lon0_rad.sin();
    let z0 = (n0 * (1.0 - e2) + alt0) * lat0_rad.sin();

    // ECEF to ENU
    let dx = x - x0;
    let dy = y - y0;
    let dz = z - z0;

    let e = -lon0_rad.sin() * dx + lon0_rad.cos() * dy;
    let n = -lon0_rad.sin() * lat0_rad.cos() * dx - lon0_rad.cos() * lat0_rad.cos() * dy
        + lat0_rad.sin() * dz;
    let u = lon0_rad.cos() * lat0_rad.sin() * dx
        + lon0_rad.sin() * lat0_rad.sin() * dy
        + lat0_rad.cos() * dz;

    Ok((e, n, u))
}

fn read_f64_le(payload: &[u8], cursor: &mut usize) -> Result<f64> {
    if *cursor + 8 > payload.len() {
        return Err(anyhow::anyhow!("payload too short"));
    }
    let val = f64::from_le_bytes([
        payload[*cursor],
        payload[*cursor + 1],
        payload[*cursor + 2],
        payload[*cursor + 3],
        payload[*cursor + 4],
        payload[*cursor + 5],
        payload[*cursor + 6],
        payload[*cursor + 7],
    ]);
    *cursor += 8;
    Ok(val)
}

fn read_i8(payload: &[u8], cursor: &mut usize) -> Result<i8> {
    if *cursor + 1 > payload.len() {
        return Err(anyhow::anyhow!("payload too short"));
    }
    let val = payload[*cursor] as i8;
    *cursor += 1;
    Ok(val)
}

fn read_u16_le(payload: &[u8], cursor: &mut usize) -> Result<u16> {
    if *cursor + 2 > payload.len() {
        return Err(anyhow::anyhow!("Buffer too short for u16"));
    }
    let val = u16::from_le_bytes([payload[*cursor], payload[*cursor + 1]]);
    *cursor += 2;
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

fn normalize_path(topic: &str) -> String {
    if topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("/{}", topic)
    }
}

/// Get human-readable service names from bitmask
fn get_service_names(service: u16) -> String {
    let mut names = Vec::new();
    if service & 1 != 0 { names.push("GPS"); }
    if service & 2 != 0 { names.push("GLONASS"); }
    if service & 4 != 0 { names.push("COMPASS"); }
    if service & 8 != 0 { names.push("GALILEO"); }
    names.join(", ")
}

/// Get geoid correction in meters for given latitude/longitude
/// Uses EGM96 grid file in PGM format
fn get_geoid_correction(geoid_path: &str, lat: f64, lon: f64) -> Result<f64> {
    // For now, implement basic PGM parsing
    // EGM96 is a 15' grid, so we need to interpolate
    // This is a simplified implementation - in production, use a proper geoid library

    use std::fs::File;
    use std::io::{BufRead, BufReader};

    let file = File::open(geoid_path)?;
    let reader = BufReader::new(file);

    let mut lines = reader.lines();

    // Parse PGM header
    let magic = lines.next().ok_or_else(|| anyhow::anyhow!("Invalid PGM file"))??;
    if magic != "P2" && magic != "P5" {
        return Err(anyhow::anyhow!("Not a valid PGM file"));
    }

    // Skip comments
    let mut width = 0;
    let mut height = 0;
    let mut _max_val = 0;

    for line in lines.by_ref() {
        let line = line?;
        let line = line.trim();
        if line.starts_with('#') || line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        if width == 0 {
            if parts.len() >= 2 {
                width = parts[0].parse()?;
                height = parts[1].parse()?;
                break;
            }
        }
    }

    // Read max value
    for line in lines.by_ref() {
        let line = line?;
        let line = line.trim();
        if line.starts_with('#') || line.is_empty() {
            continue;
        }
        _max_val = line.parse()?;
        break;
    }

    // For simplicity, assume EGM96 grid parameters
    // EGM96 covers -90 to 90 lat, -180 to 180 lon, 15' resolution
    let lat_min = -90.0;
    let lon_min = -180.0;
    let lat_step = 15.0 / 60.0; // 15 arcminutes
    let lon_step = 15.0 / 60.0;

    // Calculate grid indices
    let lat_idx = ((lat - lat_min) / lat_step) as usize;
    let lon_idx = ((lon - lon_min) / lon_step) as usize;

    if lat_idx >= height || lon_idx >= width {
        return Err(anyhow::anyhow!("Coordinates out of grid bounds"));
    }

    // For P2 (ASCII), read the data
    if magic == "P2" {
        let mut data = Vec::new();
        for line in lines {
            let line = line?;
            let parts: Vec<i32> = line.split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            data.extend(parts);
        }

        if data.len() < width * height {
            return Err(anyhow::anyhow!("Incomplete PGM data"));
        }

        let index = lat_idx * width + lon_idx;
        let raw_value = data[index] as f64;

        // Convert to meters (EGM96 values are in cm, stored as offset from -1000)
        let correction = (raw_value - 1000.0) / 100.0;

        Ok(correction)
    } else {
        // P5 (binary) - not implemented yet
        Err(anyhow::anyhow!("Binary PGM not supported yet"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wgs84_to_enu() {
        // Test ENU conversion
        let lat = 46.7821;
        let lon = -71.2740;
        let alt = 90.0;
        let lat0 = 46.7821;
        let lon0 = -71.2740;
        let alt0 = 90.0;

        let enu = wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0).unwrap();
        // At same point, should be close to 0
        assert!(enu.0.abs() < 1e-3);
        assert!(enu.1.abs() < 1e-3);
        assert!(enu.2.abs() < 1e-3);
    }

    #[test]
    fn test_geoid_correction() {
        // This would require a test PGM file
        // For now, just test the function signature
        // let correction = get_geoid_correction("test.pgm", 45.0, -75.0);
        // assert!(correction.is_ok());
    }

    #[test]
    fn test_get_service_names() {
        assert_eq!(get_service_names(0), "");
        assert_eq!(get_service_names(1), "GPS");
        assert_eq!(get_service_names(2), "GLONASS");
        assert_eq!(get_service_names(4), "COMPASS");
        assert_eq!(get_service_names(8), "GALILEO");
        assert_eq!(get_service_names(1 | 2), "GPS, GLONASS");
        assert_eq!(get_service_names(1 | 4 | 8), "GPS, COMPASS, GALILEO");
    }
}
