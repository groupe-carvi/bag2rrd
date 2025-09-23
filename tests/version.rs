#[test]
fn version_is_semver_like() {
    let ver = env!("CARGO_PKG_VERSION");
    assert!(ver.split('.').count() >= 2);
}
