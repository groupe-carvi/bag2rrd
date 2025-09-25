use anyhow::Result;
// Manual ROS message parsing for sensor_msgs/Imu
pub fn imu_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
) -> anyhow::Result<()> {
    // Parse IMU message manually
    let imu_data = parse_ros_imu(payload)?;
    
    // Set timestamp
    rec.set_timestamp_secs_since_epoch("ros_time", ts);
    
    let entity_path = format!("/{}/imu", topic.trim_start_matches('/'));
    
    // Log orientation as Transform3D
    if is_valid_quaternion(&imu_data.orientation) {
        rec.log(
            format!("{}/orientation", entity_path),
            &rerun::archetypes::Transform3D::from_rotation(
                rerun::datatypes::Quaternion::from_xyzw([
                    imu_data.orientation.x as f32,
                    imu_data.orientation.y as f32,
                    imu_data.orientation.z as f32,
                    imu_data.orientation.w as f32,
                ])
            )
        )?;
    }
    
    // Log angular velocity as arrows
    rec.log(
        format!("{}/angular_velocity", entity_path),
        &rerun::archetypes::Arrows3D::from_vectors([[
            imu_data.angular_velocity.x as f32,
            imu_data.angular_velocity.y as f32,
            imu_data.angular_velocity.z as f32,
        ]])
        .with_colors([rerun::Color::from_rgb(255, 165, 0)]) // Orange
    )?;
    
    // Log linear acceleration as arrows
    rec.log(
        format!("{}/linear_acceleration", entity_path),
        &rerun::archetypes::Arrows3D::from_vectors([[
            imu_data.linear_acceleration.x as f32,
            imu_data.linear_acceleration.y as f32,
            imu_data.linear_acceleration.z as f32,
        ]])
        .with_colors([rerun::Color::from_rgb(255, 0, 0)]) // Red
    )?;
    
    // Log magnitude scalars
    let angular_magnitude = (
        imu_data.angular_velocity.x.powi(2) +
        imu_data.angular_velocity.y.powi(2) +
        imu_data.angular_velocity.z.powi(2)
    ).sqrt();
    
    let linear_magnitude = (
        imu_data.linear_acceleration.x.powi(2) +
        imu_data.linear_acceleration.y.powi(2) +
        imu_data.linear_acceleration.z.powi(2)
    ).sqrt();
    
    rec.log(
        format!("{}/angular_velocity_magnitude", entity_path),
        &rerun::archetypes::Scalars::new(vec![angular_magnitude])
    )?;
    
    rec.log(
        format!("{}/linear_acceleration_magnitude", entity_path),
        &rerun::archetypes::Scalars::new(vec![linear_magnitude])
    )?;
    
    tracing::debug!(
        "Added IMU data to entity: {} with orientation: {:?}, angular_velocity: {:?}, linear_acceleration: {:?}",
        entity_path,
        imu_data.orientation,
        imu_data.angular_velocity,
        imu_data.linear_acceleration
    );
    
    Ok(())
}

// IMU data structure
#[derive(Debug)]
struct ImuData {
    orientation: Quaternion,
    angular_velocity: Vector3,
    linear_acceleration: Vector3,
}

#[derive(Debug)]
struct Quaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Debug)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

// Check if quaternion is valid (not all zeros and normalized)
fn is_valid_quaternion(q: &Quaternion) -> bool {
    let norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    
    // Check if quaternion is not zero and reasonably normalized (allow some tolerance)
    norm_sq > 0.01 && (norm_sq - 1.0).abs() < 0.1
}

// ROS message parsing helper
fn parse_ros_imu(payload: &[u8]) -> Result<ImuData> {
    let mut cursor = 0;

    // Parse header (std_msgs/Header) - skip for now
    cursor = skip_header(payload, cursor)?;

    // Parse orientation (geometry_msgs/Quaternion)
    if payload.len() < cursor + 32 {
        return Err(anyhow::anyhow!("payload too short for orientation"));
    }
    let orientation = Quaternion {
        x: f64::from_le_bytes([
            payload[cursor], payload[cursor + 1], payload[cursor + 2], payload[cursor + 3],
            payload[cursor + 4], payload[cursor + 5], payload[cursor + 6], payload[cursor + 7],
        ]),
        y: f64::from_le_bytes([
            payload[cursor + 8], payload[cursor + 9], payload[cursor + 10], payload[cursor + 11],
            payload[cursor + 12], payload[cursor + 13], payload[cursor + 14], payload[cursor + 15],
        ]),
        z: f64::from_le_bytes([
            payload[cursor + 16], payload[cursor + 17], payload[cursor + 18], payload[cursor + 19],
            payload[cursor + 20], payload[cursor + 21], payload[cursor + 22], payload[cursor + 23],
        ]),
        w: f64::from_le_bytes([
            payload[cursor + 24], payload[cursor + 25], payload[cursor + 26], payload[cursor + 27],
            payload[cursor + 28], payload[cursor + 29], payload[cursor + 30], payload[cursor + 31],
        ]),
    };
    cursor += 32;

    // Skip orientation_covariance (9 * f64 = 72 bytes)
    cursor += 72;

    // Parse angular_velocity (geometry_msgs/Vector3)
    if payload.len() < cursor + 24 {
        return Err(anyhow::anyhow!("payload too short for angular_velocity"));
    }
    let angular_velocity = Vector3 {
        x: f64::from_le_bytes([
            payload[cursor], payload[cursor + 1], payload[cursor + 2], payload[cursor + 3],
            payload[cursor + 4], payload[cursor + 5], payload[cursor + 6], payload[cursor + 7],
        ]),
        y: f64::from_le_bytes([
            payload[cursor + 8], payload[cursor + 9], payload[cursor + 10], payload[cursor + 11],
            payload[cursor + 12], payload[cursor + 13], payload[cursor + 14], payload[cursor + 15],
        ]),
        z: f64::from_le_bytes([
            payload[cursor + 16], payload[cursor + 17], payload[cursor + 18], payload[cursor + 19],
            payload[cursor + 20], payload[cursor + 21], payload[cursor + 22], payload[cursor + 23],
        ]),
    };
    cursor += 24;

    // Skip angular_velocity_covariance (9 * f64 = 72 bytes)
    cursor += 72;

    // Parse linear_acceleration (geometry_msgs/Vector3)
    if payload.len() < cursor + 24 {
        return Err(anyhow::anyhow!("payload too short for linear_acceleration"));
    }
    let linear_acceleration = Vector3 {
        x: f64::from_le_bytes([
            payload[cursor], payload[cursor + 1], payload[cursor + 2], payload[cursor + 3],
            payload[cursor + 4], payload[cursor + 5], payload[cursor + 6], payload[cursor + 7],
        ]),
        y: f64::from_le_bytes([
            payload[cursor + 8], payload[cursor + 9], payload[cursor + 10], payload[cursor + 11],
            payload[cursor + 12], payload[cursor + 13], payload[cursor + 14], payload[cursor + 15],
        ]),
        z: f64::from_le_bytes([
            payload[cursor + 16], payload[cursor + 17], payload[cursor + 18], payload[cursor + 19],
            payload[cursor + 20], payload[cursor + 21], payload[cursor + 22], payload[cursor + 23],
        ]),
    };

    Ok(ImuData {
        orientation,
        angular_velocity,
        linear_acceleration,
    })
}

fn skip_header(payload: &[u8], mut cursor: usize) -> Result<usize> {
    // seq (uint32)
    cursor += 4;

    // stamp (time): secs (uint32), nsecs (uint32)
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
    cursor += 4 + frame_id_len;

    Ok(cursor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_quaternion() {
        let valid_q = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        assert!(is_valid_quaternion(&valid_q));

        let zero_q = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        assert!(!is_valid_quaternion(&zero_q));
        
        let not_normalized_q = Quaternion {
            x: 0.5,
            y: 0.5,
            z: 0.5,
            w: 2.0, // Too large, not normalized
        };
        assert!(!is_valid_quaternion(&not_normalized_q));
    }

    #[test]
    fn test_quaternion_normalization() {
        let almost_normalized_q = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.1,
            w: 0.995, // sqrt(1-0.1^2) ≈ 0.995, within tolerance
        };
        assert!(is_valid_quaternion(&almost_normalized_q));
    }
}