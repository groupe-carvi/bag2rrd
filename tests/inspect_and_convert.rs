use std::fs;
use std::path::Path;
use std::process::Command;

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
    assert!(stdout.contains("Topic"), "Output should contain topic information");
    assert!(stdout.contains("Type"), "Output should contain type information");
    assert!(stdout.contains("Count"), "Output should contain count information");
}

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
    assert!(Path::new(test_rrd_path).exists(), "RRD file should be created");

    // Cleanup
    let _ = fs::remove_file(test_rrd_path);
}

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
            "run", "--",
            "convert",
            test_bag_path,
            test_rrd_path,
            "--include", "/camera/image_raw",
            "--start", "0.0",
            "--end", "10.0",
            "--progress"
        ])
        .output()
        .expect("Failed to run convert command with filters");

    // Check that it succeeded
    assert!(output.status.success(), "Convert command with filters failed");

    // Cleanup
    let _ = fs::remove_file(test_rrd_path);
}

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
            "run", "--",
            "convert",
            test_bag_path,
            test_rrd_path,
            "--dry-run"
        ])
        .output()
        .expect("Failed to run convert command with dry-run");

    // Check that it succeeded
    assert!(output.status.success(), "Convert command with dry-run failed");

    // Check that RRD file was NOT created
    assert!(!Path::new(test_rrd_path).exists(), "RRD file should not be created in dry-run");
}