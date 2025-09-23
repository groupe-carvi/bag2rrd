# bag2rrd (Rust)

**CLI utility** to convert **ROS1 .bag** files into **Rerun .rrd** files.

- Reading via [`rosbag = 0.6.3`]
- Writing via [`rerun = 0.25.1`]
- Built in Rust, CLI via `clap`

## Quick start
```bash
cargo run -- --help
```

## Development quality

```bash
cargo fmt -- --check
cargo clippy -- -D warnings
cargo test
```

## License

Apache-2.0
