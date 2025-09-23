//! /tf and /tf_static → Rerun Transforms3D (implemented in v0.3.0)

use anyhow::{anyhow, Result};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::collections::{BTreeMap, HashMap, HashSet};

#[derive(Clone, Copy, Debug)]
pub struct TfSample {
    pub t: f64,
    pub trans: [f64; 3],
    pub quat: [f64; 4], // [x, y, z, w]
}

#[derive(Clone, Debug)]
pub struct TfGraph {
    // key: (parent, child) in ROS names
    dynamic: BTreeMap<(String, String), Vec<TfSample>>, // sorted by t
    static_edges: BTreeMap<(String, String), TfSample>,
    // For cycle detection in static graph
    static_graph: HashMap<String, HashSet<String>>, // parent -> children
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TfMode {
    Nearest,
    Interpolate,
    None,
}

impl TfGraph {
    pub fn new() -> Self {
        Self {
            dynamic: BTreeMap::new(),
            static_edges: BTreeMap::new(),
            static_graph: HashMap::new(),
        }
    }

    /// Ingest a /tf message
    pub fn ingest_tf_msg(&mut self, rec: &rerun::RecordingStream, ts: f64, payload: &[u8], buffer_seconds: f64, root_frame: &str, map_frame: &[String]) -> Result<()> {
        let transforms = parse_tf_message(payload)?;
        for tf in transforms {
            let parent = tf.header.frame_id;
            let child = tf.child_frame_id;
            let trans = tf.transform.translation;
            let rot = tf.transform.rotation;
            // Normalize quaternion
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(rot.w, rot.x, rot.y, rot.z));
            let quat_normalized = quat.quaternion();
            let sample = TfSample {
                t: ts,
                trans: [trans.x, trans.y, trans.z],
                quat: [quat_normalized.i, quat_normalized.j, quat_normalized.k, quat_normalized.w],
            };
            self.dynamic.entry((parent.clone(), child.clone())).or_default().push(sample);

            // Log the transform
            let parent_path = map_frame_to_path(&parent, root_frame, map_frame);
            let child_path = map_frame_to_path(&child, root_frame, map_frame);
            log_transform(rec, &parent_path, &child_path, &sample_to_isometry(&sample), ts)?;
        }
        // Prune old samples based on latest ts
        self.prune_dynamic(ts, buffer_seconds);
        Ok(())
    }

    /// Ingest a /tf_static message
    pub fn ingest_tf_static_msg(&mut self, rec: &rerun::RecordingStream, payload: &[u8], root_frame: &str, map_frame: &[String]) -> Result<()> {
        let transforms = parse_tf_message(payload)?;
        for tf in transforms {
            let parent = tf.header.frame_id;
            let child = tf.child_frame_id;
            let trans = tf.transform.translation;
            let rot = tf.transform.rotation;
            // Normalize quaternion
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(rot.w, rot.x, rot.y, rot.z));
            let quat_normalized = quat.quaternion();
            let sample = TfSample {
                t: 0.0,
                trans: [trans.x, trans.y, trans.z],
                quat: [quat_normalized.i, quat_normalized.j, quat_normalized.k, quat_normalized.w],
            };
            // Check for cycle
            if self.would_create_cycle(&parent, &child) {
                tracing::warn!("Static TF edge {parent} -> {child} would create a cycle, skipping");
                continue;
            }
            self.static_edges.insert((parent.clone(), child.clone()), sample);
            self.static_graph.entry(parent.clone()).or_default().insert(child.clone());

            // Log the static transform
            let parent_path = map_frame_to_path(&parent, root_frame, map_frame);
            let child_path = map_frame_to_path(&child, root_frame, map_frame);
            log_transform(rec, &parent_path, &child_path, &sample_to_isometry(&sample), 0.0)?;
        }
        Ok(())
    }

    fn would_create_cycle(&self, parent: &str, child: &str) -> bool {
        // Simple cycle detection: check if child can reach parent
        let mut visited = HashSet::new();
        let mut stack = vec![child.to_string()];
        while let Some(node) = stack.pop() {
            if !visited.insert(node.clone()) {
                continue;
            }
            if node == parent {
                return true;
            }
            if let Some(children) = self.static_graph.get(&node) {
                for child in children {
                    stack.push(child.clone());
                }
            }
        }
        false
    }

    fn prune_dynamic(&mut self, latest_ts: f64, buffer_seconds: f64) {
        let cutoff = latest_ts - buffer_seconds;
        for samples in self.dynamic.values_mut() {
            samples.retain(|s| s.t >= cutoff);
        }
    }

    /// Resolve transform from source_frame to target_frame at time at_time
    pub fn resolve(&self, target_frame: &str, source_frame: &str, at_time: f64, mode: TfMode) -> Option<Isometry3<f64>> {
        // Find path from source to target
        let path = self.find_path(source_frame, target_frame)?;
        // Compose transforms along the path
        let mut iso = Isometry3::identity();
        for (parent, child) in path {
            let edge_iso = self.get_edge_transform(&parent, &child, at_time, mode)?;
            iso = edge_iso * iso; // Compose: parent_to_child * current
        }
        Some(iso)
    }

    fn find_path(&self, source: &str, target: &str) -> Option<Vec<(String, String)>> {
        // BFS to find path from source to target
        let mut visited = HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        let mut parent_map: HashMap<String, (String, String)> = HashMap::new();
        queue.push_back(source.to_string());
        visited.insert(source.to_string());
        while let Some(current) = queue.pop_front() {
            if current == target {
                // Reconstruct path
                let mut path = Vec::new();
                let mut node = current;
                while let Some((p, c)) = parent_map.get(&node) {
                    path.push((p.clone(), c.clone()));
                    node = p.clone();
                }
                path.reverse();
                return Some(path);
            }
            // Find neighbors: parents and children
            for (p, c) in self.static_edges.keys() {
                if p == &current && !visited.contains(c) {
                    visited.insert(c.clone());
                    parent_map.insert(c.clone(), (p.clone(), c.clone()));
                    queue.push_back(c.clone());
                }
                if c == &current && !visited.contains(p) {
                    visited.insert(p.clone());
                    parent_map.insert(p.clone(), (current.clone(), p.clone()));
                    queue.push_back(p.clone());
                }
            }
            // Also dynamic edges
            for (p, c) in self.dynamic.keys() {
                if p == &current && !visited.contains(c) {
                    visited.insert(c.clone());
                    parent_map.insert(c.clone(), (p.clone(), c.clone()));
                    queue.push_back(c.clone());
                }
                if c == &current && !visited.contains(p) {
                    visited.insert(p.clone());
                    parent_map.insert(p.clone(), (current.clone(), p.clone()));
                    queue.push_back(p.clone());
                }
            }
        }
        None
    }

    fn get_edge_transform(&self, parent: &str, child: &str, at_time: f64, mode: TfMode) -> Option<Isometry3<f64>> {
        if let Some(sample) = self.static_edges.get(&(parent.to_string(), child.to_string())) {
            return Some(sample_to_isometry(sample));
        }
        if let Some(sample) = self.static_edges.get(&(child.to_string(), parent.to_string())) {
            // Inverse transform
            let iso = sample_to_isometry(sample);
            return Some(iso.inverse());
        }
        if let Some(samples) = self.dynamic.get(&(parent.to_string(), child.to_string())) {
            return self.interpolate_samples(samples, at_time, mode);
        }
        if let Some(samples) = self.dynamic.get(&(child.to_string(), parent.to_string())) {
            // Inverse
            let iso = self.interpolate_samples(samples, at_time, mode)?;
            return Some(iso.inverse());
        }
        None
    }

    fn interpolate_samples(&self, samples: &[TfSample], at_time: f64, mode: TfMode) -> Option<Isometry3<f64>> {
        match mode {
            TfMode::None => samples.iter().find(|s| (s.t - at_time).abs() < 1e-9).map(sample_to_isometry),
            TfMode::Nearest => {
                let mut best: Option<&TfSample> = None;
                let mut best_diff = f64::INFINITY;
                for s in samples {
                    let diff = (s.t - at_time).abs();
                    if diff < best_diff {
                        best_diff = diff;
                        best = Some(s);
                    }
                }
                best.map(sample_to_isometry)
            }
            TfMode::Interpolate => {
                let mut before: Option<&TfSample> = None;
                let mut after: Option<&TfSample> = None;
                for s in samples {
                    if s.t <= at_time && (before.is_none() || s.t > before.unwrap().t) {
                        before = Some(s);
                    }
                    if s.t >= at_time && (after.is_none() || s.t < after.unwrap().t) {
                        after = Some(s);
                    }
                }
                match (before, after) {
                    (Some(b), Some(a)) if (a.t - b.t).abs() > 1e-9 => {
                        let t = (at_time - b.t) / (a.t - b.t);
                        let trans = [
                            b.trans[0] + t * (a.trans[0] - b.trans[0]),
                            b.trans[1] + t * (a.trans[1] - b.trans[1]),
                            b.trans[2] + t * (a.trans[2] - b.trans[2]),
                        ];
                        let quat_b = UnitQuaternion::from_quaternion(Quaternion::new(b.quat[3], b.quat[0], b.quat[1], b.quat[2]));
                        let quat_a = UnitQuaternion::from_quaternion(Quaternion::new(a.quat[3], a.quat[0], a.quat[1], a.quat[2]));
                        let quat = quat_b.slerp(&quat_a, t);
                        let quat_arr = quat.quaternion();
                        let sample = TfSample { t: at_time, trans, quat: [quat_arr.i, quat_arr.j, quat_arr.k, quat_arr.w] };
                        Some(sample_to_isometry(&sample))
                    }
                    (Some(b), _) => Some(sample_to_isometry(b)),
                    (_, Some(a)) => Some(sample_to_isometry(a)),
                    _ => None,
                }
            }
        }
    }
}

fn sample_to_isometry(sample: &TfSample) -> Isometry3<f64> {
    let trans = Translation3::new(sample.trans[0], sample.trans[1], sample.trans[2]);
    let quat = UnitQuaternion::from_quaternion(Quaternion::new(sample.quat[3], sample.quat[0], sample.quat[1], sample.quat[2]));
    Isometry3::from_parts(trans, quat)
}

fn log_transform(
    rec: &rerun::RecordingStream,
    #[allow(unused_variables)] parent_path: &str,
    child_path: &str,
    iso: &Isometry3<f64>,
    ts: f64,
) -> Result<()> {
    rec.set_timestamp_secs_since_epoch("ros_time", ts);
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

pub fn parse_tf_mode(s: &str) -> Result<TfMode> {
    match s {
        "nearest" => Ok(TfMode::Nearest),
        "interpolate" => Ok(TfMode::Interpolate),
        "none" => Ok(TfMode::None),
        _ => Err(anyhow!("Invalid tf-mode: {}", s)),
    }
}

// ROS message structs
#[derive(Debug)]
struct Header {
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
struct Transform {
    translation: Vector3,
    rotation: RosQuaternion,
}

#[derive(Debug)]
struct TransformStamped {
    header: Header,
    child_frame_id: String,
    transform: Transform,
}

fn parse_tf_message(payload: &[u8]) -> Result<Vec<TransformStamped>> {
    let mut cursor = 0;
    // Skip ROS header if any, but for TF, it's an array of TransformStamped
    // TF message is tf2_msgs/TFMessage which is std_msgs/Header + TransformStamped[]
    // But in practice, it's just the array.
    // Assume it's a sequence of TransformStamped
    let mut transforms = Vec::new();
    while cursor < payload.len() {
        let tf = parse_transform_stamped(payload, &mut cursor)?;
        transforms.push(tf);
    }
    Ok(transforms)
}

fn parse_transform_stamped(payload: &[u8], cursor: &mut usize) -> Result<TransformStamped> {
    // TransformStamped: header, child_frame_id, transform
    let header = parse_header(payload, cursor)?;
    let child_frame_id = parse_string(payload, cursor)?;
    let transform = parse_transform(payload, cursor)?;
    Ok(TransformStamped { header, child_frame_id, transform })
}

fn parse_header(payload: &[u8], cursor: &mut usize) -> Result<Header> {
    // Header: seq (u32), stamp (time), frame_id (string)
    *cursor += 4; // seq
    *cursor += 8; // stamp
    let frame_id = parse_string(payload, cursor)?;
    Ok(Header { frame_id })
}

fn parse_string(payload: &[u8], cursor: &mut usize) -> Result<String> {
    let len = read_u32_le(payload, cursor)? as usize;
    let bytes = &payload[*cursor..*cursor + len];
    *cursor += len;
    Ok(String::from_utf8_lossy(bytes).to_string())
}

fn parse_transform(payload: &[u8], cursor: &mut usize) -> Result<Transform> {
    let translation = parse_vector3(payload, cursor)?;
    let rotation = parse_quaternion(payload, cursor)?;
    Ok(Transform { translation, rotation })
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

fn read_u32_le(payload: &[u8], cursor: &mut usize) -> Result<u32> {
    if *cursor + 4 > payload.len() {
        return Err(anyhow!("Unexpected end of payload"));
    }
    let val = u32::from_le_bytes(payload[*cursor..*cursor + 4].try_into().unwrap());
    *cursor += 4;
    Ok(val)
}

fn read_f64_le(payload: &[u8], cursor: &mut usize) -> Result<f64> {
    if *cursor + 8 > payload.len() {
        return Err(anyhow!("Unexpected end of payload"));
    }
    let val = f64::from_le_bytes(payload[*cursor..*cursor + 8].try_into().unwrap());
    *cursor += 8;
    Ok(val)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_normalization() {
        let (rec, _) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
        let mut graph = TfGraph::new();
        // Create a TF message with non-normalized quaternion
        let payload = create_tf_payload();
        graph.ingest_tf_msg(&rec, 0.0, &payload, 30.0, "world", &[]).unwrap();
        // Check that quaternions are normalized
        for samples in graph.dynamic.values() {
            for sample in samples {
                let norm = (sample.quat[0]*sample.quat[0] + sample.quat[1]*sample.quat[1] + sample.quat[2]*sample.quat[2] + sample.quat[3]*sample.quat[3]).sqrt();
                assert!((norm - 1.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn test_resolve_simple_chain() {
        let (rec, _) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
        let mut graph = TfGraph::new();
        // Add static edges A -> B, B -> C
        let payload_ab = create_tf_static_payload("A", "B", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        graph.ingest_tf_static_msg(&rec, &payload_ab, "world", &[]).unwrap();
        let payload_bc = create_tf_static_payload("B", "C", [0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        graph.ingest_tf_static_msg(&rec, &payload_bc, "world", &[]).unwrap();
        // Resolve A to C
        let iso = graph.resolve("C", "A", 0.0, TfMode::Nearest).unwrap();
        let trans = iso.translation.vector;
        assert!((trans.x - 1.0).abs() < 1e-6);
        assert!((trans.y - 1.0).abs() < 1e-6);
        assert!((trans.z - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_cycle_detection() {
        let (rec, _) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
        let mut graph = TfGraph::new();
        let payload_ab = create_tf_static_payload("A", "B", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        graph.ingest_tf_static_msg(&rec, &payload_ab, "world", &[]).unwrap();
        let payload_ba = create_tf_static_payload("B", "A", [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        // Should not add cycle
        assert!(graph.ingest_tf_static_msg(&rec, &payload_ba, "world", &[]).is_ok());
        assert!(!graph.static_edges.contains_key(&("B".to_string(), "A".to_string())));
    }

    fn create_tf_payload() -> Vec<u8> {
        // Simplified: just one transform
        let mut data = Vec::new();
        // TransformStamped
        // header: seq=0, stamp=0, frame_id="parent"
        data.extend_from_slice(&0u32.to_le_bytes()); // seq
        data.extend_from_slice(&0u64.to_le_bytes()); // stamp
        let frame_id = b"parent";
        data.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
        data.extend_from_slice(frame_id);
        // child_frame_id="child"
        let child = b"child";
        data.extend_from_slice(&(child.len() as u32).to_le_bytes());
        data.extend_from_slice(child);
        // transform: translation [1,0,0], rotation [0,0,0,1]
        data.extend_from_slice(&1.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&1.0f64.to_le_bytes());
        data
    }

    fn create_tf_static_payload(parent: &str, child: &str, trans: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
        let mut data = Vec::new();
        // header: seq=0, stamp=0, frame_id=parent
        data.extend_from_slice(&0u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&(parent.len() as u32).to_le_bytes());
        data.extend_from_slice(parent.as_bytes());
        // child_frame_id=child
        data.extend_from_slice(&(child.len() as u32).to_le_bytes());
        data.extend_from_slice(child.as_bytes());
        // transform
        for &t in &trans {
            data.extend_from_slice(&t.to_le_bytes());
        }
        for &q in &quat {
            data.extend_from_slice(&q.to_le_bytes());
        }
        data
    }
}
