//! Odometry/Pose/Path → Rerun Transforms3D/LineStrips3D (implemented in v0.3.0)

use anyhow::Result;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

#[allow(clippy::too_many_arguments)]
pub fn odometry_to_rerun(
    rec: &rerun::RecordingStream,
    #[allow(unused_variables)] _topic: &str,
    ts: f64,
    payload: &[u8],
    root_frame: &str,
    #[allow(unused_variables)] map_frame: &[String],
    tf_graph: Option<&crate::mappings::tf::TfGraph>,
    tf_mode: crate::mappings::tf::TfMode,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    let odom = parse_odometry(payload)?;
    let parent_frame = odom.header.frame_id;
    let child_frame = odom.child_frame_id;
    let pose = odom.pose.pose;

    // Create transform from parent to child
    let iso = pose_to_isometry(&pose);

    // Log the transform
    let parent_path = map_frame_to_path(&parent_frame, root_frame, map_frame);
    let child_path = map_frame_to_path(&child_frame, root_frame, map_frame);
    log_transform(rec, &parent_path, &child_path, &iso, ts)?;

    // If TF is available, resolve to root
    if let Some(tf) = tf_graph && let Some(root_iso) = tf.resolve(root_frame, &parent_frame, ts, tf_mode) {
        let combined_iso = root_iso * iso;
        let root_path = format!("/{root_frame}");
        log_transform(rec, &root_path, &child_path, &combined_iso, ts)?;
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
#[allow(clippy::too_many_arguments)]
pub fn pose_stamped_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
    root_frame: &str,
    topic_renames: &[String],
    #[allow(unused_variables)] map_frame: &[String],
    tf_graph: Option<&crate::mappings::tf::TfGraph>,
    tf_mode: crate::mappings::tf::TfMode,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    let pose_stamped = parse_pose_stamped(payload, &mut 0)?;
    let frame_id = pose_stamped.header.frame_id;

    let entity_path = map_topic_to_path(topic, topic_renames).unwrap_or_else(|| format!("/{root_frame}/poses/{topic}"));

    let iso = pose_to_isometry(&pose_stamped.pose);

    // If TF available, resolve to root
    let final_iso = if let Some(tf) = tf_graph {
        if let Some(root_iso) = tf.resolve(root_frame, &frame_id, ts, tf_mode) {
            root_iso * iso
        } else {
            iso
        }
    } else {
        iso
    };

    let root_path = format!("/{root_frame}");
    log_transform(rec, &root_path, &entity_path, &final_iso, ts)?;

    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub fn path_to_rerun(
    rec: &rerun::RecordingStream,
    topic: &str,
    ts: f64,
    payload: &[u8],
    root_frame: &str,
    topic_renames: &[String],
    #[allow(unused_variables)] map_frame: &[String],
    tf_graph: Option<&crate::mappings::tf::TfGraph>,
    tf_mode: crate::mappings::tf::TfMode,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);

    let path = parse_path(payload)?;
    let entity_path = map_topic_to_path(topic, topic_renames).unwrap_or_else(|| format!("/{root_frame}/paths/{topic}"));

    let mut points = Vec::new();
    for pose_stamped in &path.poses {
        let frame_id = pose_stamped.header.frame_id.clone();
        let iso = pose_to_isometry(&pose_stamped.pose);
        let final_iso = if let Some(tf) = tf_graph {
            if let Some(root_iso) = tf.resolve(root_frame, &frame_id, pose_stamped.header.stamp, tf_mode) {
                root_iso * iso
            } else {
                iso
            }
        } else {
            iso
        };
        let trans = final_iso.translation.vector;
        points.push([trans.x as f32, trans.y as f32, trans.z as f32]);
    }

    if !points.is_empty() {
        let line_strips = rerun::archetypes::LineStrips3D::new(vec![points]);
        rec.log(entity_path, &line_strips)?;
    }

    Ok(())
}

fn log_transform(
    rec: &rerun::RecordingStream,
    #[allow(unused_variables)] parent_path: &str,
    child_path: &str,
    iso: &Isometry3<f64>,
    #[allow(unused_variables)] ts: f64,
) -> Result<()> {
    let trans = iso.translation.vector;
    let quat = iso.rotation.quaternion();
    let transform = rerun::archetypes::Transform3D::from_translation_rotation(
        [trans.x as f32, trans.y as f32, trans.z as f32],
        rerun::datatypes::Quaternion::from_xyzw([quat.i as f32, quat.j as f32, quat.k as f32, quat.w as f32]),
    );
    rec.log(child_path, &transform)?;
    Ok(())
}

fn map_frame_to_path(frame: &str, root_frame: &str, map_frame: &[String]) -> String {
    for mapping in map_frame {
        if let Some((ros_frame, rr_path)) = mapping.split_once('=') && ros_frame == frame {
            return rr_path.to_string();
        }
    }
    format!("/{root_frame}/{frame}")
}

fn map_topic_to_path(topic: &str, topic_renames: &[String]) -> Option<String> {
    for rename in topic_renames {
        if let Some((ros_topic, rr_path)) = rename.split_once('=') && ros_topic == topic {
            return Some(rr_path.to_string());
        }
    }
    None
}

fn pose_to_isometry(pose: &Pose) -> Isometry3<f64> {
    let trans = Translation3::new(pose.position.x, pose.position.y, pose.position.z);
    let quat = UnitQuaternion::from_quaternion(Quaternion::new(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    Isometry3::from_parts(trans, quat)
}

// Parsing structs and functions
#[derive(Debug)]
struct Header {
    stamp: f64,
    frame_id: String,
}

#[derive(Debug)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug)]
struct RosQuaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Debug)]
struct Pose {
    position: Vector3,
    orientation: RosQuaternion,
}

#[derive(Debug)]
struct Odometry {
    header: Header,
    child_frame_id: String,
    pose: PoseWithCovariance,
}

#[derive(Debug)]
struct PoseWithCovariance {
    pose: Pose,
}

#[derive(Debug)]
struct PoseStamped {
    header: Header,
    pose: Pose,
}

#[derive(Debug)]
struct Path {
    #[allow(dead_code)] header: Header,
    poses: Vec<PoseStamped>,
}

fn parse_odometry(payload: &[u8]) -> Result<Odometry> {
    let mut cursor = 0;
    let header = parse_header(payload, &mut cursor)?;
    let child_frame_id = parse_string(payload, &mut cursor)?;
    let pose = parse_pose_with_covariance(payload, &mut cursor)?;
    Ok(Odometry { header, child_frame_id, pose })
}

fn parse_pose_stamped(payload: &[u8], cursor: &mut usize) -> Result<PoseStamped> {
    let header = parse_header(payload, cursor)?;
    let pose = parse_pose(payload, cursor)?;
    Ok(PoseStamped { header, pose })
}

fn parse_path(payload: &[u8]) -> Result<Path> {
    let mut cursor = 0;
    let header = parse_header(payload, &mut cursor)?;
    let len = read_u32_le(payload, &mut cursor)? as usize;
    let mut poses = Vec::with_capacity(len);
    for _ in 0..len {
        poses.push(parse_pose_stamped(payload, &mut cursor)?);
    }
    Ok(Path { header, poses })
}

fn parse_header(payload: &[u8], cursor: &mut usize) -> Result<Header> {
    *cursor += 4; // seq
    let stamp = read_f64_le(payload, cursor)?; // time
    let frame_id = parse_string(payload, cursor)?;
    Ok(Header { stamp, frame_id })
}

fn parse_pose_with_covariance(payload: &[u8], cursor: &mut usize) -> Result<PoseWithCovariance> {
    let pose = parse_pose(payload, cursor)?;
    *cursor += 36 * 8; // covariance matrix
    Ok(PoseWithCovariance { pose })
}

fn parse_pose(payload: &[u8], cursor: &mut usize) -> Result<Pose> {
    let position = parse_vector3(payload, cursor)?;
    let orientation = parse_quaternion(payload, cursor)?;
    Ok(Pose { position, orientation })
}

fn parse_vector3(payload: &[u8], cursor: &mut usize) -> Result<Vector3> {
    let x = read_f64_le(payload, cursor)?;
    let y = read_f64_le(payload, cursor)?;
    let z = read_f64_le(payload, cursor)?;
    Ok(Vector3 { x, y, z })
}

fn parse_quaternion(payload: &[u8], cursor: &mut usize) -> Result<RosQuaternion> {
    let x = read_f64_le(payload, cursor)?;
    let y = read_f64_le(payload, cursor)?;
    let z = read_f64_le(payload, cursor)?;
    let w = read_f64_le(payload, cursor)?;
    Ok(RosQuaternion { x, y, z, w })
}

fn parse_string(payload: &[u8], cursor: &mut usize) -> Result<String> {
    let len = read_u32_le(payload, cursor)? as usize;
    let bytes = &payload[*cursor..*cursor + len];
    *cursor += len;
    Ok(String::from_utf8_lossy(bytes).to_string())
}

fn read_u32_le(payload: &[u8], cursor: &mut usize) -> Result<u32> {
    if *cursor + 4 > payload.len() {
        return Err(anyhow::anyhow!("Unexpected end of payload"));
    }
    let val = u32::from_le_bytes(payload[*cursor..*cursor + 4].try_into().unwrap());
    *cursor += 4;
    Ok(val)
}

fn read_f64_le(payload: &[u8], cursor: &mut usize) -> Result<f64> {
    if *cursor + 8 > payload.len() {
        return Err(anyhow::anyhow!("Unexpected end of payload"));
    }
    let val = f64::from_le_bytes(payload[*cursor..*cursor + 8].try_into().unwrap());
    *cursor += 8;
    Ok(val)
}
