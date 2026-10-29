//! Gives the Windows command-line program the EPCC icon and file details.

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
        .set("FileDescription", "EPCC command line")
        .set("LegalCopyright", "MIT OR Apache-2.0");
    resource.compile().expect("could not embed the Windows icon");
}

#[cfg(not(windows))]
fn embed_resource() {}
