//! Gives the Windows program its icon and file details (shown in Explorer and the taskbar).

fn main() {
    println!("cargo:rerun-if-changed=../../assets/icon.ico");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        embed_resource();
    }
}

/// Windows builds are made on Windows, where the resource compiler is available.
#[cfg(windows)]
fn embed_resource() {
    let mut resource = winresource::WindowsResource::new();
    resource
        .set_icon("../../assets/icon.ico")
        .set("ProductName", "EPCC")
        .set("FileDescription", "EPCC - Easy Point Cloud Converter")
        .set("LegalCopyright", "MIT OR Apache-2.0");
    resource.compile().expect("could not embed the Windows icon");
}

#[cfg(not(windows))]
fn embed_resource() {}
