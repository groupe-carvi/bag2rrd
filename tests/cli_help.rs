use assert_cmd::Command;

#[test]
fn cli_help_runs() {
    let mut cmd = Command::cargo_bin("bag2rrd").unwrap();
    cmd.arg("--help").assert().success();
}
