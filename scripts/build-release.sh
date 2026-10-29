#!/usr/bin/env bash
# Builds optimised binaries without local folder names inside them.
#
# Rust embeds source paths for panic messages. Without remapping they would contain the
# builder's home folder (and so, often, their user name).
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
cargo_home="${CARGO_HOME:-$HOME/.cargo}"

# When several prefixes match, rustc applies the last one, so the most specific comes last.
export RUSTFLAGS="${RUSTFLAGS:-} --remap-path-prefix=$HOME=/home --remap-path-prefix=$cargo_home=/cargo --remap-path-prefix=$repo=/epcc"
if [[ "${OS:-}" == "Windows_NT" ]]; then
    # Windows tools may report the same folders with backslashes and a drive letter.
    win() { cygpath -w "$1"; }
    export RUSTFLAGS="$RUSTFLAGS --remap-path-prefix=$(win "$HOME")=/home --remap-path-prefix=$(win "$cargo_home")=/cargo --remap-path-prefix=$(win "$repo")=/epcc"
fi

cd "$repo"
cargo build --release --workspace "$@"
