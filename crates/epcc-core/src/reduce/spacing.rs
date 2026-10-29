//! Minimum point spacing, out of core.
//!
//! The kept points are stored in a hash grid of cubes with edge `spacing`, so a new point only
//! needs to be compared with the points in its own and the 26 neighbouring cubes. The cubes are
//! grouped into blocks of 32 × 32 × 32. When the kept points outgrow the memory limit, the
//! blocks used longest ago move to a temporary file next to the output.
//!
//! Each chunk of points is decided in two steps, which together give exactly the result of
//! deciding point after point in file order:
//! 1. Every point is compared with the points kept from earlier chunks, which do not change
//!    meanwhile. This runs on all cores, and the blocks the chunk needs are brought back from
//!    disk as many at a time as fit into memory, each at most once per chunk.
//! 2. The points that pass are decided one by one, in file order, against the points kept
//!    from this chunk so far.
//!
//! Files that store their points scan by scan, line by line or tile by tile need few blocks
//! per chunk, mostly still in memory. Points in random order need nearly every block for every
//! chunk once the kept points no longer fit into memory, which makes each chunk read the whole
//! temporary file.

use std::fs::{self, File, OpenOptions};
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use super::PointFilter;
use crate::error::IoContext;
use crate::model::PointChunk;
use crate::octree::staging::TempFolder;
use crate::{Error, Result};

/// How much is held in memory and how the grid is cut into blocks.
#[derive(Clone, Copy, Debug)]
struct Limits {
    /// Kept points held in memory, at about 60 bytes each including the allocator's share.
    /// Exceeded by at most the points kept from one chunk.
    memory_points: usize,
    /// Blocks have `1 << block_shift` cubes along each edge.
    block_shift: u32,
    /// A block on disk in this many pieces is written anew in one piece when it is stored again.
    max_spans: usize,
    /// The temporary file is rewritten without its unused parts once these are larger than
    /// the used ones and than this many bytes.
    compaction_bytes: u64,
}

const LIMITS: Limits = Limits { memory_points: 8 << 20, block_shift: 5, max_spans: 8, compaction_bytes: 64 << 20 };

/// Bytes per kept point on disk: its cube within the block as `u32`, then its offset from the
/// cube's corner as three `f32`, all little-endian.
const RECORD_BYTES: usize = 16;

/// Keeps a point only if no kept point lies closer than `spacing`. The result has no two points
/// closer than `spacing`, and every dropped point has a kept point within `spacing`.
///
/// Points are decided in file order: of two points closer than `spacing`, the one that comes
/// first is kept. Memory stays bounded however many points are kept, see the module
/// documentation.
pub struct MinSpacing {
    distance: Distance,
    /// Grid coordinates are relative to the first point, which keeps them small.
    origin: Option<[f64; 3]>,
    grid: KeptGrid,
}

#[derive(Clone, Copy, Debug)]
struct Distance {
    spacing: f64,
    squared: f64,
}

type Cell = [i32; 3];
type Located = (Cell, [f64; 3]);

const NEIGHBOURS: [[i32; 3]; 27] = neighbours_own_cell_first();

const fn neighbours_own_cell_first() -> [[i32; 3]; 27] {
    let mut cells = [[0; 3]; 27];
    let mut count = 1; // index 0 is the own cell
    let mut dx = -1;
    while dx <= 1 {
        let mut dy = -1;
        while dy <= 1 {
            let mut dz = -1;
            while dz <= 1 {
                if dx != 0 || dy != 0 || dz != 0 {
                    cells[count] = [dx, dy, dz];
                    count += 1;
                }
                dz += 1;
            }
            dy += 1;
        }
        dx += 1;
    }
    cells
}

impl MinSpacing {
    /// A filter whose temporary file, if it needs one, goes to the system's temporary folder.
    pub fn new(spacing: f64) -> Self {
        Self::with_work_folder(spacing, &std::env::temp_dir())
    }

    /// A filter whose temporary file, if it needs one, goes to a new folder inside `folder`.
    /// The folder is removed when the filter is dropped.
    pub fn with_work_folder(spacing: f64, folder: &Path) -> Self {
        Self::with_limits(spacing, folder, LIMITS)
    }

    fn with_limits(spacing: f64, folder: &Path, limits: Limits) -> Self {
        MinSpacing {
            distance: Distance { spacing, squared: spacing * spacing },
            origin: None,
            grid: KeptGrid::new(folder, limits),
        }
    }

    /// The cell holding `position`, and the position relative to that cell's corner.
    fn locate(&self, origin: [f64; 3], position: [f64; 3]) -> Result<Located> {
        let spacing = self.distance.spacing;
        let mut cell = [0i32; 3];
        let mut offset = [0f64; 3];
        for axis in 0..3 {
            let scaled = (position[axis] - origin[axis]) / spacing;
            let index = scaled.floor();
            if index.abs() >= f64::from(i32::MAX) {
                return Err(Error::InvalidSetting(format!(
                    "The cloud is too large for a point spacing of {spacing} m. Use a larger spacing."
                )));
            }
            cell[axis] = index as i32;
            offset[axis] = (scaled - index) * spacing;
        }
        Ok((cell, offset))
    }
}

impl PointFilter for MinSpacing {
    fn apply(&mut self, chunk: &mut PointChunk) -> Result<()> {
        let Some(&first) = chunk.positions.first() else {
            return Ok(());
        };
        let origin = *self.origin.get_or_insert(first);
        let located =
            chunk.positions.par_iter().map(|&position| self.locate(origin, position)).collect::<Result<Vec<_>>>()?;
        let distance = self.distance;
        let grid = &mut self.grid;
        let mut keep = vec![false; located.len()];

        let all_in_memory = if grid.fits_in_memory(located.len()) {
            keep.par_iter_mut()
                .zip(&located)
                .for_each(|(keep, &(cell, offset))| *keep = !grid.has_point_near(cell, offset, distance));
            true
        } else {
            grid.check_against_kept(&located, &mut keep, distance)?
        };

        if all_in_memory {
            // The second step against the same grid, which simply grows.
            for (kept, &(cell, offset)) in keep.iter_mut().zip(&located) {
                if !*kept {
                    continue;
                }
                if grid.has_point_near(cell, offset, distance) {
                    *kept = false; // too close to a point kept earlier in this chunk
                } else {
                    grid.remember(cell, offset);
                }
            }
            grid.trim()?;
        } else {
            // Some blocks around the points went back to disk during the first step.
            let mut this_chunk = KeptGrid::new(&grid.work_folder, Limits { memory_points: usize::MAX, ..grid.limits });
            let mut new_points = Vec::new();
            for (kept, &(cell, offset)) in keep.iter_mut().zip(&located) {
                if !*kept {
                    continue;
                }
                if this_chunk.has_point_near(cell, offset, distance) {
                    *kept = false;
                } else {
                    this_chunk.remember(cell, offset);
                    new_points.push((cell, offset));
                }
            }
            drop(this_chunk);
            grid.add(&new_points)?;
        }
        chunk.retain(&keep);
        Ok(())
    }
}

/// The kept points, in blocks that are either in memory or in the temporary file.
struct KeptGrid {
    limits: Limits,
    /// `(1 << block_shift) - 1`, which turns a cell into its position within its block.
    mask: i32,
    /// Every block that has kept points.
    blocks: FxHashMap<Cell, Block>,
    /// Kept points in the blocks that are in memory.
    resident_points: usize,
    /// Advances whenever blocks may move to disk; tells which blocks were used last.
    tick: u64,
    work_folder: PathBuf,
    /// Created when the first block moves to disk.
    spill: Option<Spill>,
    #[cfg(test)]
    stats: Stats,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default)]
struct Stats {
    stored: u64,
    loaded: u64,
    compactions: u64,
}

struct Block {
    count: u32,
    last_used: u64,
    /// `None` while the block is only on disk.
    points: Option<Box<BlockPoints>>,
    /// Where the block's first `stored` points are in the temporary file, in order.
    spans: Vec<Span>,
    stored: u32,
}

#[derive(Clone, Copy, Debug)]
struct Span {
    offset: u64,
    points: u32,
}

#[derive(Default)]
struct BlockPoints {
    /// The last kept point of each occupied cell, by the cell's position in the block.
    cells: FxHashMap<u32, u32>,
    kept: Vec<KeptPoint>,
}

/// A kept point, stored relative to its cell's corner, which f32 resolves finely enough.
struct KeptPoint {
    offset: [f32; 3],
    /// The point kept before it in the same cell, or `NONE`.
    next: u32,
}

const NONE: u32 = u32::MAX;

impl BlockPoints {
    fn add(&mut self, cell: u32, offset: [f32; 3]) {
        let index = self.kept.len() as u32;
        let next = self.cells.insert(cell, index).unwrap_or(NONE);
        self.kept.push(KeptPoint { offset, next });
    }

    /// Whether a point of `cell`, which lies `step` cells from the new point's cell, is closer
    /// than the spacing to the new point at `offset` from its cell's corner.
    fn has_point_near(&self, cell: u32, step: [i32; 3], offset: [f64; 3], distance: Distance) -> bool {
        let mut next = self.cells.get(&cell).copied().unwrap_or(NONE);
        while next != NONE {
            let kept = &self.kept[next as usize];
            let mut distance_squared = 0.0;
            for axis in 0..3 {
                // Both positions relative to the new point's cell corner.
                let other = f64::from(step[axis]) * distance.spacing + f64::from(kept.offset[axis]);
                distance_squared += (other - offset[axis]).powi(2);
            }
            if distance_squared < distance.squared {
                return true;
            }
            next = kept.next;
        }
        false
    }

    /// The cell of every kept point, in the order of `kept`.
    fn cells_by_point(&self) -> Vec<u32> {
        let mut cells = vec![0; self.kept.len()];
        for (&cell, &last) in &self.cells {
            let mut next = last;
            while next != NONE {
                cells[next as usize] = cell;
                next = self.kept[next as usize].next;
            }
        }
        cells
    }
}

impl KeptGrid {
    fn new(work_folder: &Path, limits: Limits) -> Self {
        assert!((2..=10).contains(&limits.block_shift), "blocks need 4 to 1024 cells per edge");
        KeptGrid {
            limits,
            mask: (1 << limits.block_shift) - 1,
            blocks: FxHashMap::default(),
            resident_points: 0,
            tick: 0,
            work_folder: work_folder.to_path_buf(),
            spill: None,
            #[cfg(test)]
            stats: Stats::default(),
        }
    }

    fn block_of(&self, cell: Cell) -> Cell {
        cell.map(|c| c >> self.limits.block_shift)
    }

    /// Packs a cell's position within its block into one number.
    fn pack(&self, local: [i32; 3]) -> u32 {
        let shift = self.limits.block_shift;
        (local[0] | local[1] << shift | local[2] << (2 * shift)) as u32
    }

    fn cell_in_block(&self, cell: Cell) -> u32 {
        self.pack(cell.map(|c| c & self.mask))
    }

    /// The points of the block `key`, if it has any and is in memory.
    fn points(&self, key: Cell) -> Option<&BlockPoints> {
        self.blocks.get(&key)?.points.as_deref()
    }

    /// Whether a kept point in the blocks in memory lies closer than the spacing to the point at
    /// `offset` from the corner of `cell`.
    fn has_point_near(&self, cell: Cell, offset: [f64; 3], distance: Distance) -> bool {
        let local = cell.map(|c| c & self.mask);
        if local.iter().all(|&l| l > 0 && l < self.mask) {
            // All 27 cells are in the same block.
            let Some(points) = self.points(self.block_of(cell)) else {
                return false;
            };
            NEIGHBOURS.iter().any(|step| {
                let neighbour = [local[0] + step[0], local[1] + step[1], local[2] + step[2]];
                points.has_point_near(self.pack(neighbour), *step, offset, distance)
            })
        } else {
            NEIGHBOURS.iter().any(|step| {
                let neighbour = [cell[0] + step[0], cell[1] + step[1], cell[2] + step[2]];
                self.points(self.block_of(neighbour))
                    .is_some_and(|points| points.has_point_near(self.cell_in_block(neighbour), *step, offset, distance))
            })
        }
    }

    /// Whether every block is in memory and stays within the limit with `new_points` more.
    fn fits_in_memory(&self, new_points: usize) -> bool {
        self.spill.is_none() && self.resident_points.saturating_add(new_points) <= self.limits.memory_points
    }

    /// Adds a kept point. Its block must be in memory if it has points.
    fn remember(&mut self, cell: Cell, offset: [f64; 3]) {
        let key = self.block_of(cell);
        let local = self.cell_in_block(cell);
        let tick = self.tick;
        let block = self.blocks.entry(key).or_insert_with(|| Block {
            count: 0,
            last_used: tick,
            points: Some(Box::default()),
            spans: Vec::new(),
            stored: 0,
        });
        let points = block.points.as_mut().expect("the block of a kept point is in memory");
        points.add(local, offset.map(|v| v as f32));
        block.count += 1;
        block.last_used = tick;
        self.resident_points += 1;
    }

    /// Sets `keep[i]` to whether no kept point lies closer than the spacing to `located[i]`.
    /// The blocks the points need come back from disk in groups that fit into memory. Returns
    /// whether all of them are in memory afterwards.
    fn check_against_kept(&mut self, located: &[Located], keep: &mut [bool], distance: Distance) -> Result<bool> {
        self.tick += 1;
        let shift = self.limits.block_shift;
        // The blocks holding a point's 27 cells: at most two along each axis.
        let ranges: Vec<[Cell; 2]> = located
            .par_iter()
            .map(|&(cell, _)| [cell.map(|c| (c - 1) >> shift), cell.map(|c| (c + 1) >> shift)])
            .collect();
        let mut seen = FxHashSet::default();
        let mut on_disk = Vec::new();
        let mut previous = None;
        for &range in &ranges {
            if previous.replace(range) == Some(range) {
                continue;
            }
            for key in blocks_in(range) {
                if seen.insert(key)
                    && let Some(block) = self.blocks.get_mut(&key)
                {
                    block.last_used = self.tick;
                    if block.points.is_none() {
                        on_disk.push(key);
                    }
                }
            }
        }

        let this = &*self;
        keep.par_iter_mut()
            .zip(located)
            .for_each(|(keep, &(cell, offset))| *keep = !this.has_point_near(cell, offset, distance));
        if on_disk.is_empty() {
            return Ok(true);
        }

        // Neighbouring blocks tend to land in the same group.
        on_disk.sort_unstable();
        let half = (self.limits.memory_points / 2).max(1);
        let mut groups: Vec<(Vec<Cell>, usize)> = Vec::new();
        let mut group_of = FxHashMap::default();
        for key in on_disk {
            let count = self.blocks[&key].count as usize;
            match groups.last_mut() {
                Some((keys, points)) if *points + count <= half => {
                    keys.push(key);
                    *points += count;
                }
                _ => groups.push((vec![key], count)),
            }
            group_of.insert(key, groups.len() - 1);
        }
        for (index, (keys, points)) in groups.iter().enumerate() {
            self.tick += 1;
            if self.resident_points + points > self.limits.memory_points {
                self.store_until((self.limits.memory_points / 4 * 3).saturating_sub(*points))?;
            }
            for key in keys {
                self.load(*key)?;
            }
            let this = &*self;
            keep.par_iter_mut().zip(located).zip(&ranges).for_each(|((keep, &(cell, offset)), range)| {
                if *keep && blocks_in(*range).any(|key| group_of.get(&key) == Some(&index)) {
                    *keep = !this.has_point_near(cell, offset, distance);
                }
            });
        }
        Ok(false)
    }

    /// Adds the points kept from one chunk, in file order. Blocks on disk get them on disk.
    fn add(&mut self, new_points: &[Located]) -> Result<()> {
        self.tick += 1;
        // By block, and in file order within each block.
        let mut order: Vec<(Cell, usize)> =
            new_points.iter().enumerate().map(|(index, &(cell, _))| (self.block_of(cell), index)).collect();
        order.sort_unstable();
        let mut records = Vec::new();
        for run in order.chunk_by(|a, b| a.0 == b.0) {
            let key = run[0].0;
            let on_disk = self.blocks.get(&key).is_some_and(|block| block.points.is_none());
            if !on_disk {
                for &(_, index) in run {
                    self.remember(new_points[index].0, new_points[index].1);
                }
                continue;
            }
            let spill = self.spill.as_ref().expect("a block not in memory is on disk");
            let offset = spill.len + records.len() as u64;
            for &(_, index) in run {
                let (cell, position) = new_points[index];
                encode(self.cell_in_block(cell), position.map(|v| v as f32), &mut records);
            }
            let block = self.blocks.get_mut(&key).expect("checked above");
            block.spans.push(Span { offset, points: run.len() as u32 });
            block.count += run.len() as u32;
            block.stored = block.count;
        }
        if let Some(spill) = &mut self.spill {
            spill.append(&records)?;
        }
        self.trim()
    }

    /// Moves blocks to disk if more kept points than the limit are in memory.
    fn trim(&mut self) -> Result<()> {
        if self.resident_points > self.limits.memory_points {
            self.tick += 1;
            self.store_until(self.limits.memory_points / 4 * 3)?;
        }
        Ok(())
    }

    /// Moves the blocks not used since the last tick to disk, the one used longest ago first,
    /// until at most `target` kept points are in memory.
    fn store_until(&mut self, target: usize) -> Result<()> {
        let mut candidates: Vec<(u64, Cell)> = self
            .blocks
            .iter()
            .filter(|(_, block)| block.points.is_some() && block.last_used < self.tick)
            .map(|(key, block)| (block.last_used, *key))
            .collect();
        candidates.sort_unstable();

        if self.spill.is_none() {
            self.spill = Some(Spill::create(&self.work_folder)?);
        }
        let spill = self.spill.as_mut().expect("just created");
        let mut records = Vec::new();
        for (_, key) in candidates {
            if self.resident_points <= target {
                break;
            }
            let block = self.blocks.get_mut(&key).expect("candidates are blocks");
            let points = block.points.take().expect("candidates are in memory");
            self.resident_points -= points.kept.len();
            let rewrite = block.spans.len() >= self.limits.max_spans;
            if block.stored == block.count && !rewrite {
                continue; // unchanged since it came from disk
            }
            let first = if rewrite {
                spill.unused += u64::from(block.stored) * RECORD_BYTES as u64;
                block.spans.clear();
                0
            } else {
                block.stored as usize
            };
            let offset = spill.len + records.len() as u64;
            let cells = points.cells_by_point();
            for (&cell, point) in cells.iter().zip(&points.kept).skip(first) {
                encode(cell, point.offset, &mut records);
            }
            block.spans.push(Span { offset, points: block.count - first as u32 });
            block.stored = block.count;
            #[cfg(test)]
            {
                self.stats.stored += 1;
            }
        }
        spill.append(&records)?;
        if spill.unused > self.limits.compaction_bytes && spill.unused > spill.len - spill.unused {
            self.compact()?;
        }
        Ok(())
    }

    /// Reads the block `key` back from disk, unless it is in memory or has no points.
    fn load(&mut self, key: Cell) -> Result<()> {
        let Some(block) = self.blocks.get_mut(&key) else {
            return Ok(());
        };
        block.last_used = self.tick;
        if block.points.is_some() {
            return Ok(());
        }
        let spill = self.spill.as_mut().expect("a block not in memory is on disk");
        let mut points = BlockPoints {
            cells: FxHashMap::with_capacity_and_hasher(block.count as usize, Default::default()),
            kept: Vec::with_capacity(block.count as usize),
        };
        for span in &block.spans {
            for record in spill.read(*span)?.as_chunks::<RECORD_BYTES>().0 {
                let word = |at: usize| [record[at], record[at + 1], record[at + 2], record[at + 3]];
                let offset = [4, 8, 12].map(|at| f32::from_le_bytes(word(at)));
                points.add(u32::from_le_bytes(word(0)), offset);
            }
        }
        if points.kept.len() != block.count as usize {
            return Err(Error::invalid(format!(
                "{}: temporary file holds {} points of a block instead of {}",
                spill.path.display(),
                points.kept.len(),
                block.count
            )));
        }
        self.resident_points += points.kept.len();
        block.points = Some(Box::new(points));
        #[cfg(test)]
        {
            self.stats.loaded += 1;
        }
        Ok(())
    }

    /// Rewrites the temporary file with only the parts still in use.
    fn compact(&mut self) -> Result<()> {
        let spill = self.spill.as_mut().expect("compacting an existing file");
        spill.files_made += 1;
        let path = spill.folder.path().join(format!("kept-points-{}.bin", spill.files_made));
        let mut writer = BufWriter::with_capacity(8 << 20, create_read_write(&path)?);
        let mut len = 0u64;
        for block in self.blocks.values_mut().filter(|block| !block.spans.is_empty()) {
            let mut points = 0;
            for span in &block.spans {
                writer.write_all(spill.read(*span)?).at(&path)?;
                points += span.points;
            }
            block.spans = vec![Span { offset: len, points }];
            len += u64::from(points) * RECORD_BYTES as u64;
        }
        let file = writer.into_inner().map_err(|error| error.into_error()).at(&path)?;
        let old_path = std::mem::replace(&mut spill.path, path);
        drop(std::mem::replace(&mut spill.file, file));
        fs::remove_file(&old_path).at(&old_path)?;
        spill.len = len;
        spill.unused = 0;
        #[cfg(test)]
        {
            self.stats.compactions += 1;
        }
        Ok(())
    }
}

/// The temporary file of kept points, in a folder of its own that is removed with it.
struct Spill {
    /// Declared before `folder`, so it is closed before the folder is removed.
    file: File,
    path: PathBuf,
    len: u64,
    /// Bytes no longer used, because their blocks were written anew.
    unused: u64,
    files_made: u32,
    /// The bytes read last.
    buffer: Vec<u8>,
    folder: TempFolder,
}

impl Spill {
    fn create(parent: &Path) -> Result<Spill> {
        let folder = TempFolder::create(parent)?;
        let path = folder.path().join("kept-points-0.bin");
        let file = create_read_write(&path)?;
        Ok(Spill { file, path, len: 0, unused: 0, files_made: 0, buffer: Vec::new(), folder })
    }

    fn append(&mut self, bytes: &[u8]) -> Result<()> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.file.seek(SeekFrom::Start(self.len)).at(&self.path)?;
        self.file.write_all(bytes).at(&self.path)?;
        self.len += bytes.len() as u64;
        Ok(())
    }

    fn read(&mut self, span: Span) -> Result<&[u8]> {
        self.buffer.resize(span.points as usize * RECORD_BYTES, 0);
        self.file.seek(SeekFrom::Start(span.offset)).at(&self.path)?;
        self.file.read_exact(&mut self.buffer).at(&self.path)?;
        Ok(&self.buffer)
    }
}

/// The blocks from `range[0]` to `range[1]`.
fn blocks_in([low, high]: [Cell; 2]) -> impl Iterator<Item = Cell> {
    (low[0]..=high[0])
        .flat_map(move |x| (low[1]..=high[1]).flat_map(move |y| (low[2]..=high[2]).map(move |z| [x, y, z])))
}

/// Appends the record of a kept point to `records`.
fn encode(cell: u32, offset: [f32; 3], records: &mut Vec<u8>) {
    records.extend_from_slice(&cell.to_le_bytes());
    for value in offset {
        records.extend_from_slice(&value.to_le_bytes());
    }
}

fn create_read_write(path: &Path) -> Result<File> {
    OpenOptions::new().read(true).write(true).create(true).truncate(true).open(path).at(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Attributes;

    fn chunk(positions: Vec<[f64; 3]>) -> PointChunk {
        PointChunk { positions, ..PointChunk::new(0, Attributes::NONE) }
    }

    /// A deterministic pseudo-random cloud in a 1 m cube.
    fn random_cloud(count: usize, seed: u64) -> Vec<[f64; 3]> {
        let mut state = 0x2545_f491_4f6c_dd1du64 ^ seed;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..count).map(|_| [next(), next(), next()]).collect()
    }

    /// A deterministic shuffle.
    fn shuffled<T>(mut items: Vec<T>, seed: u64) -> Vec<T> {
        let mut state = seed | 1;
        for i in (1..items.len()).rev() {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            items.swap(i, (state % (i as u64 + 1)) as usize);
        }
        items
    }

    /// A folder of its own for one test, empty at the start.
    fn work_folder(name: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!("epcc-spacing-test-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).unwrap();
        path
    }

    fn entries(folder: &Path) -> usize {
        fs::read_dir(folder).unwrap().count()
    }

    /// Thins `points` handed over in chunks of `chunk_points`.
    fn thin(filter: &mut MinSpacing, points: &[[f64; 3]], chunk_points: usize) -> Vec<[f64; 3]> {
        let mut kept = Vec::new();
        for piece in points.chunks(chunk_points) {
            let mut piece = chunk(piece.to_vec());
            filter.apply(&mut piece).unwrap();
            kept.extend(piece.positions);
        }
        kept
    }

    fn in_memory(spacing: f64, folder: &Path) -> MinSpacing {
        MinSpacing::with_limits(spacing, folder, Limits { memory_points: usize::MAX / 4, ..LIMITS })
    }

    /// Blocks of 4 × 4 × 4 cells and room for only `memory_points` kept points, so even small
    /// clouds go to disk in many blocks.
    fn out_of_core(spacing: f64, folder: &Path, memory_points: usize) -> MinSpacing {
        MinSpacing::with_limits(
            spacing,
            folder,
            Limits { memory_points, block_shift: 2, max_spans: 2, compaction_bytes: 0 },
        )
    }

    /// Point after point against every kept point, in exact arithmetic.
    fn reference(points: &[[f64; 3]], spacing: f64) -> Vec<[f64; 3]> {
        let mut kept: Vec<[f64; 3]> = Vec::new();
        for point in points {
            let near =
                kept.iter().any(|k| k.iter().zip(point).map(|(a, b)| (a - b).powi(2)).sum::<f64>() < spacing * spacing);
            if !near {
                kept.push(*point);
            }
        }
        kept
    }

    #[test]
    fn kept_points_respect_the_spacing() {
        let spacing = 0.05;
        let points = random_cloud(20_000, 0);
        let mut cloud = chunk(points.clone());
        MinSpacing::new(spacing).apply(&mut cloud).unwrap();

        let kept = &cloud.positions;
        assert!(kept.len() > 100 && kept.len() < points.len());
        let distance = |a: &[f64; 3], b: &[f64; 3]| a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum::<f64>().sqrt();
        for (i, a) in kept.iter().enumerate() {
            for b in &kept[i + 1..] {
                assert!(distance(a, b) >= spacing, "kept points closer than the spacing");
            }
        }
        for point in &points {
            assert!(kept.iter().any(|k| distance(k, point) < spacing + 1e-9), "dropped point far from all kept ones");
        }
    }

    #[test]
    fn spacing_works_far_from_the_origin() {
        let mut cloud = chunk(vec![
            [500_000.0, 5_400_000.0, 300.0],
            [500_000.004, 5_400_000.0, 300.0],
            [500_000.02, 5_400_000.0, 300.0],
        ]);
        MinSpacing::new(0.01).apply(&mut cloud).unwrap();
        assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn reduction_is_deterministic_across_chunk_sizes() {
        let points = random_cloud(5_000, 0);
        let mut whole = chunk(points.clone());
        MinSpacing::new(0.1).apply(&mut whole).unwrap();
        assert_eq!(whole.positions, thin(&mut MinSpacing::new(0.1), &points, 333));
    }

    #[test]
    fn out_of_core_keeps_the_same_points_in_the_same_order() {
        let folder = work_folder("same");
        let spacing = 0.03;
        // Two sweeps over the same cube, the second one revisiting every block.
        let mut points = random_cloud(30_000, 1);
        points.extend(random_cloud(30_000, 2));
        let expected = thin(&mut in_memory(spacing, &folder), &points, 4_096);
        assert_eq!(entries(&folder), 0, "an in-memory run needs no temporary file");
        assert!(expected.len() > 5_000 && expected.len() < points.len() / 2, "kept {}", expected.len());

        let mut compactions = 0;
        for (memory_points, chunk_points) in [(300, 1_000), (2_000, 7_777), (expected.len() / 3, 25_000)] {
            let mut filter = out_of_core(spacing, &folder, memory_points);
            let kept = thin(&mut filter, &points, chunk_points);
            let stats = filter.grid.stats;
            assert!(stats.stored > 0 && stats.loaded > 0, "{stats:?}");
            compactions += stats.compactions;
            assert_eq!(entries(&folder), 1, "one temporary folder while running");
            drop(filter);
            assert_eq!(entries(&folder), 0, "temporary files removed");
            assert_eq!(kept, expected, "memory for {memory_points} points, chunks of {chunk_points}");
        }
        assert!(compactions > 0, "the temporary file was never compacted");
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn scan_like_order_keeps_the_same_points() {
        let folder = work_folder("rows");
        // Rows along x, advancing in y, then the same surface again in the opposite direction.
        let row = |y: usize, reverse: bool| {
            (0..400).map(move |x| {
                let x = if reverse { 399 - x } else { x };
                [x as f64 * 0.004, y as f64 * 0.004, (x as f64 * 0.01).sin() * 0.05]
            })
        };
        let mut points: Vec<[f64; 3]> = (0..200).flat_map(|y| row(y, false)).collect();
        points.extend((0..200).rev().flat_map(|y| row(y, true).map(|[x, y, z]| [x + 0.002, y + 0.001, z])));
        let spacing = 0.01;
        let expected = thin(&mut in_memory(spacing, &folder), &points, 10_000);
        let mut filter = out_of_core(spacing, &folder, 500);
        assert_eq!(thin(&mut filter, &points, 10_000), expected);
        assert!(filter.grid.stats.loaded > 0);
        drop(filter);
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn points_exactly_at_the_spacing_across_block_borders() {
        let folder = work_folder("border");
        // Spacing and coordinates are exact binary fractions, so distances of exactly the
        // spacing are computed exactly. Blocks are 4 cells = 1 m wide, and the lattice
        // points at whole metres lie on block borders, each exactly one spacing from its
        // neighbours across the border. The first point is in the middle, so blocks on both
        // sides of the origin are used.
        let spacing = 0.25;
        let lattice: Vec<[f64; 3]> = (0..16 * 16 * 16)
            .map(|i| [(i % 16) as f64 * 0.25, (i / 16 % 16) as f64 * 0.25, (i / 256) as f64 * 0.25])
            .collect();
        let mut lattice = shuffled(lattice, 7);
        let middle = lattice.iter().position(|p| *p == [2.0, 2.0, 2.0]).unwrap();
        lattice.swap(0, middle);

        // Nothing is closer than the spacing: every point stays.
        let mut filter = out_of_core(spacing, &folder, 50);
        assert_eq!(thin(&mut filter, &lattice, 500), lattice);
        assert!(filter.grid.stats.loaded > 0);
        drop(filter);

        // Points half a spacing off the lattice compete with their lattice neighbours; which one
        // stays depends on which comes first.
        let between: Vec<[f64; 3]> = lattice.iter().map(|&[x, y, z]| [x + 0.125, y, z - 0.125]).collect();
        let mixed = shuffled(lattice.iter().copied().chain(between).collect(), 11);
        let expected = reference(&mixed, spacing);
        assert!(expected.len() < mixed.len());
        assert_eq!(thin(&mut in_memory(spacing, &folder), &mixed, 1_000), expected);
        for chunk_points in [1, 333, 8_192] {
            let mut filter = out_of_core(spacing, &folder, 64);
            assert_eq!(thin(&mut filter, &mixed, chunk_points), expected, "chunks of {chunk_points}");
            assert!(chunk_points >= mixed.len() || filter.grid.stats.loaded > 0);
        }
        assert_eq!(entries(&folder), 0);
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn out_of_core_is_deterministic() {
        let folder = work_folder("deterministic");
        let points = shuffled(random_cloud(20_000, 3), 5);
        let first = thin(&mut out_of_core(0.04, &folder, 200), &points, 1_500);
        let second = thin(&mut out_of_core(0.04, &folder, 200), &points, 1_500);
        let other_limits = thin(&mut out_of_core(0.04, &folder, 900), &points, 4_000);
        assert_eq!(first, second);
        assert_eq!(first, other_limits);
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn temporary_files_are_removed_when_stopped_early() {
        let folder = work_folder("stopped");
        let points = random_cloud(20_000, 4);
        let mut filter = out_of_core(0.03, &folder, 100);
        for piece in points.chunks(2_000).take(4) {
            filter.apply(&mut chunk(piece.to_vec())).unwrap();
        }
        assert_eq!(entries(&folder), 1);
        drop(filter); // as when a later stage fails or the user cancels
        assert_eq!(entries(&folder), 0);
        fs::remove_dir_all(&folder).unwrap();
    }
}
