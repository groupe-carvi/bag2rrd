use std::fs;
use std::path::Path;
use std::process::Command;

#[cfg(feature = "integration_tests")]
#[test]
fn test_inspect_command() {
    // Use the downloaded test bag file
    let test_bag_path = "tests/data/race_1.bag";

    // Check if test bag exists
    if !Path::new(test_bag_path).exists() {
        panic!("Test bag file not found. Run tests/download_test_bag.sh first");
    }

    // Run inspect command
    let output = Command::new("cargo")
        .args(&["run", "--", "inspect", test_bag_path])
        .output()
        .expect("Failed to run inspect command");

    // Check that it succeeded
    assert!(output.status.success(), "Inspect command failed");

    // Check output format
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Topic"),
        "Output should contain topic information"
    );
    assert!(
        stdout.contains("Type"),
        "Output should contain type information"
    );
    assert!(
        stdout.contains("Count"),
        "Output should contain count information"
    );
}

#[cfg(feature = "integration_tests")]
#[test]
fn test_convert_command() {
    // Use the downloaded test bag file
    let test_bag_path = "tests/data/race_1.bag";
    let test_rrd_path = "tests/data/test.rrd";

    // Check if test bag exists
    if !Path::new(test_bag_path).exists() {
        panic!("Test bag file not found. Run tests/download_test_bag.sh first");
    }

    // Run convert command
    let output = Command::new("cargo")
        .args(&["run", "--", "convert", test_bag_path, test_rrd_path])
        .output()
        .expect("Failed to run convert command");

    // Check that it succeeded
    assert!(output.status.success(), "Convert command failed");

    // Check that RRD file was created
    assert!(
        Path::new(test_rrd_path).exists(),
        "RRD file should be created"
    );

    // Cleanup
    let _ = fs::remove_file(test_rrd_path);
}

#[cfg(feature = "integration_tests")]
#[test]
fn test_convert_with_filters() {
    // Use the downloaded test bag file
    let test_bag_path = "tests/data/race_1.bag";
    let test_rrd_path = "tests/data/test.rrd";

    // Check if test bag exists
    if !Path::new(test_bag_path).exists() {
        panic!("Test bag file not found. Run tests/download_test_bag.sh first");
    }

    // Run convert command with filters
    let output = Command::new("cargo")
        .args(&[
            "run",
            "--",
            "convert",
            test_bag_path,
            test_rrd_path,
            "--include",
            "/camera/image_raw",
            "--start",
            "0.0",
            "--end",
            "10.0",
            "--progress",
        ])
        .output()
        .expect("Failed to run convert command with filters");

    // Check that it succeeded
    assert!(
        output.status.success(),
        "Convert command with filters failed"
    );

    // Cleanup
    let _ = fs::remove_file(test_rrd_path);
}

#[cfg(feature = "integration_tests")]
#[test]
fn test_dry_run() {
    // Use the downloaded test bag file
    let test_bag_path = "tests/data/race_1.bag";
    let test_rrd_path = "tests/data/test.rrd";

    // Check if test bag exists
    if !Path::new(test_bag_path).exists() {
        panic!("Test bag file not found. Run tests/download_test_bag.sh first");
    }

    // Clean up any existing file
    let _ = fs::remove_file(test_rrd_path);

    // Run convert command with dry-run
    let output = Command::new("cargo")
        .args(&[
            "run",
            "--",
            "convert",
            test_bag_path,
            test_rrd_path,
            "--dry-run",
        ])
        .output()
        .expect("Failed to run convert command with dry-run");

    // Check that it succeeded
    assert!(
        output.status.success(),
        "Convert command with dry-run failed"
    );

    // Check that RRD file was NOT created
    assert!(
        !Path::new(test_rrd_path).exists(),
        "RRD file should not be created in dry-run"
    );
}

#[cfg(feature = "integration_tests")]
#[test]
fn test_schema_command() {
    // Run schema command
    let output = Command::new("cargo")
        .args(&["run", "--", "schema"])
        .output()
        .expect("Failed to run schema command");

    // Check that it succeeded
    assert!(output.status.success(), "Schema command failed");

    // Check output contains expected mappings
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("sensor_msgs/Image"),
        "Output should contain Image mapping"
    );
    assert!(
        stdout.contains("sensor_msgs/NavSatFix"),
        "Output should contain NavSatFix mapping"
    );
    assert!(
        stdout.contains("v0.1.0"),
        "Output should contain version information"
    );
}

#[cfg(feature = "integration_tests")]
#[test]
fn test_validate_command_nonexistent_file() {
    // Run validate command on nonexistent file
    let output = Command::new("cargo")
        .args(&["run", "--", "validate", "nonexistent.rrd"])
        .output()
        .expect("Failed to run validate command");

    // Check that it failed
    assert!(!output.status.success(), "Validate command should fail on nonexistent file");

    // Check error message in both stdout and stderr
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined_output = format!("{}{}", stdout, stderr);
    assert!(
        combined_output.contains("FAILED") || combined_output.contains("does not exist"),
        "Error message should indicate file not found. stdout: {}, stderr: {}",
        stdout, stderr
    );
}
