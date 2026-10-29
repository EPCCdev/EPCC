#!/usr/bin/env bash
# Builds the portable download for the system this runs on, into dist/:
#
#   Windows  EPCC-windows-x64.zip    EPCC.exe, command-line/epcc.exe
#   macOS    EPCC-macos.zip          EPCC.app, command-line/epcc  (Apple silicon and Intel)
#   Linux    EPCC-linux-x64.tar.gz   EPCC,     command-line/epcc
#
# The files sit at the top of the archive, so unpacking gives one folder with the program
# right inside it. The release download links rely on these archive names.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"
version="$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)"

case "$(uname -s)" in
    MINGW* | MSYS* | CYGWIN*) os=windows ;;
    Darwin) os=macos ;;
    *) os=linux ;;
esac

rm -rf dist
stage=dist/EPCC
mkdir -p "$stage/command-line"

if [[ $os == macos ]]; then
    targets=(aarch64-apple-darwin x86_64-apple-darwin)
    for target in "${targets[@]}"; do
        ./scripts/build-release.sh --target "$target"
    done
    # One program for both processor types.
    lipo -create -output "$stage/command-line/epcc" target/aarch64-apple-darwin/release/epcc \
        target/x86_64-apple-darwin/release/epcc
    contents="$stage/EPCC.app/Contents"
    mkdir -p "$contents/MacOS" "$contents/Resources"
    cp assets/icon.icns "$contents/Resources/EPCC.icns"
    lipo -create -output "$contents/MacOS/EPCC" target/aarch64-apple-darwin/release/epcc-gui \
        target/x86_64-apple-darwin/release/epcc-gui
    cat > "$contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key><string>EPCC</string>
    <key>CFBundleDisplayName</key><string>EPCC</string>
    <key>CFBundleIdentifier</key><string>io.github.epccdev.epcc</string>
    <key>CFBundleExecutable</key><string>EPCC</string>
    <key>CFBundleIconFile</key><string>EPCC</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>$version</string>
    <key>CFBundleVersion</key><string>$version</string>
    <key>LSMinimumSystemVersion</key><string>11.0</string>
    <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
EOF
    # A free ad-hoc signature: without any, macOS calls a downloaded app damaged and offers no
    # way to open it; with it, the user can allow the app once.
    codesign --force --deep --sign - "$stage/EPCC.app"
    codesign --force --sign - "$stage/command-line/epcc"
    start="double-click EPCC.app"
else
    ./scripts/build-release.sh
    exe=""
    [[ $os == windows ]] && exe=".exe"
    cp "target/release/epcc-gui$exe" "$stage/EPCC$exe"
    cp "target/release/epcc$exe" "$stage/command-line/epcc$exe"
    if [[ $os == windows ]]; then start="double-click EPCC.exe"; else start="double-click EPCC (or run ./EPCC)"; fi
fi

cp LICENSE-MIT LICENSE-APACHE "$stage/"
cat > "$stage/Read me.txt" <<EOF
EPCC - Easy Point Cloud Converter, version $version

To start, $start.
Instructions, updates and source code: https://github.com/EPCCdev/EPCC

The "command-line" folder holds epcc, the same converter for scripts; run "epcc --help".
Settings are saved in epcc-settings.toml next to the program.

Free software under the MIT or Apache 2.0 licence, see LICENSE-MIT and LICENSE-APACHE.
EOF

# Everything in the folder, without a leading "./", which some unpackers show as a folder.
entries=()
for entry in "$stage"/*; do entries+=("$(basename "$entry")"); done

case $os in
    windows)
        archive="dist/EPCC-windows-x64.zip"
        if command -v 7z > /dev/null; then
            (cd "$stage" && 7z a -tzip -bso0 "../$(basename "$archive")" "${entries[@]}")
        else
            # Windows' own tar writes zip files.
            "$SYSTEMROOT/System32/tar.exe" -a -c -f "$archive" -C "$stage" "${entries[@]}"
        fi
        ;;
    macos)
        archive="dist/EPCC-macos.zip"
        (cd "$stage" && zip -qry "../$(basename "$archive")" "${entries[@]}")
        ;;
    linux)
        archive="dist/EPCC-linux-x64.tar.gz"
        tar czf "$archive" -C "$stage" "${entries[@]}"
        ;;
esac
echo "$archive"
