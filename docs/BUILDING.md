# Building EPCC from source

Most people do not need this: the [download buttons](../README.md) give you a ready program.

## Requirements

* A current stable [Rust](https://rustup.rs) toolchain.
* On Linux, the window system libraries, for example on Debian or Ubuntu:

  ```
  sudo apt-get install libxkbcommon-dev libwayland-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev
  ```

## Building

```
cargo build --release
```

builds `target/release/epcc-gui` (the window) and `target/release/epcc` (the command line).
`cargo test` runs the tests.

For programs you hand to others, use the scripts instead. They remove local folder names (which
usually contain your user name) from the binaries:

* `scripts/build-release.sh` builds them;
* `scripts/package.sh` builds them and packs the portable download for the system it runs on
  into `dist/`, exactly as the download buttons deliver it.

### Program icon

The icon is drawn in `assets/icon.svg`. `python scripts/make_icons.py` (Python 3 with numpy)
renders it into `assets/icon.ico` (built into the Windows programs), `assets/icon.icns` (the
macOS app) and `assets/icon-128.rgba` (the window icon). Run it after changing the SVG and commit
the results; building needs no Python.

## How the code is organised

```
crates/epcc-core   the library: point model, formats, reduction, conversion pipeline, settings
crates/epcc-cli    the epcc command
crates/epcc-gui    the window (egui)
docs/              this file and the pictures of the README
scripts/           release builds and packaging
```

Every reader turns its file into a stream of point chunks in one shared representation
(`model.rs`); every writer consumes such a stream. A conversion runs reader, filters and writer
on separate threads connected by small queues (`convert.rs`), and outputs are written under a
temporary name and renamed when complete, so a failed or cancelled run leaves nothing behind.
Formats that write several files (Potree) write a folder the same way (`output.rs`).

To add a format, add a module under `crates/epcc-core/src/format/` with a reader and/or writer
and a variant to `Format`; the compiler lists every place that needs to know about it.

Formats that store an octree (COPC and Potree) build it with `octree/`, which works out of core:
the points are staged in temporary files next to the output, split into pieces that fit in
memory, and built piece by piece. The temporary files take about twice the uncompressed size of the
cloud and are removed when the conversion ends.

Thinning to a minimum point spacing (`reduce/spacing.rs`) keeps the points it has kept in a grid
of blocks; when they outgrow the memory limit, the blocks used longest ago go to a temporary file
next to the output and come back when later points need them. Points are still decided in file
order, so the result is the same as with everything in memory.

RCS has an octree layout of its own but works out of core the same way, with the same temporary
point files: the points are staged, sorted into one file per leaf region, and the leaves built a
few million points at a time. The temporary files take about twice the uncompressed size of the
cloud plus the size of the output.
