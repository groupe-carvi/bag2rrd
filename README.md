# bag2rrd (Rust)

**CLI utility** to convert **ROS1 .bag** files into **Rerun .rrd** files.

- Reading via [`rosbag = 0.6.3`]
- Writing via [`rerun = 0.25.1`]
- Built in Rust, CLI via `clap`

## Quick start
```bash
cargo run -- --help
```

## Testing

Download a test ROS bag file and try the commands:

```bash
# Download test data (not tracked by git)
./tests/download_test_bag.sh

# Inspect the bag file
cargo run -- inspect tests/data/race_1.bag

# Convert to RRD (images only in v0.1.0)
cargo run -- convert tests/data/race_1.bag output.rrd
```

## Development quality

```bash
cargo fmt -- --check
cargo clippy -- -D warnings
cargo test
```

## License

Apache-2.0
