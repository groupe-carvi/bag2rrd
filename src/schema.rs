//! Schema command - Print supported ROS → Rerun mappings

use anyhow::Result;

/// Print all supported ROS → Rerun mappings with version introduced
pub fn print_schema() -> Result<()> {
    println!("Supported ROS → Rerun mappings:");
    println!("---------------------------------------------------------------");

    let mappings = vec![
        ("sensor_msgs/Image", "Image/DepthImage", "v0.1.0"),
        ("sensor_msgs/CompressedImage", "Image", "v0.1.0"),
        ("sensor_msgs/PointCloud2", "Points3D", "v0.2.0"),
        ("sensor_msgs/LaserScan", "Points2D/LineStrips2D", "v0.2.0"),
        ("sensor_msgs/NavSatFix", "Points3D (+path optional)", "v0.2.0"),
        ("/tf, /tf_static", "Transforms3D", "v0.3.0"),
        ("nav_msgs/Odometry", "Transforms3D", "v0.3.0"),
        ("geometry_msgs/PoseStamped", "Transforms3D", "v0.3.0"),
        ("nav_msgs/Path", "LineStrips3D", "v0.3.0"),
    ];

    for (ros_type, rerun_archetype, version) in mappings {
        println!("{:<30} → {:<25} {}", ros_type, rerun_archetype, version);
    }

    Ok(())
}