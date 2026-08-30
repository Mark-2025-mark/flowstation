use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    // Only resolve the native TETRA codec link path when the Asterisk SIP/RTP
    // bridge is compiled in. The default build excludes net_asterisk entirely,
    // so it must not depend on libtetra-codec being installed.
    if std::env::var("CARGO_FEATURE_ASTERISK").is_ok() {
        // Keep the .so findable after Install without forcing LD_LIBRARY_PATH on systemd.
        println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/local/lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/lib");
        if let Ok(output) = Command::new("pkg-config").args(["--libs", "tetra-codec"]).output()
            && output.status.success()
        {
            let flags = String::from_utf8_lossy(&output.stdout);
            for flag in flags.split_whitespace() {
                if let Some(path) = flag.strip_prefix("-L") {
                    println!("cargo:rustc-link-search=native={path}");
                    println!("cargo:rustc-link-arg=-Wl,-rpath,{path}");
                }
            }
        }
    }
}
