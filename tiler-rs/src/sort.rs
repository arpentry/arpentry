//! External merge sort with a configurable memory budget (TILER.md §sort).
//!
//! Records are `(u64 key, variable-length bytes)`. They accumulate in memory
//! until the budget is exceeded, at which point a sorted run is flushed to a
//! temporary file. [`ExternalSorter::into_sorted`] then performs a k-way
//! min-heap merge across the spilled runs plus the still-resident buffer,
//! yielding a globally key-ordered stream.
//!
//! Deep module: the caller only sees `new` / `add` / `into_sorted`. Run
//! spilling, the on-disk run format, the k-way merge, and temp-file cleanup are
//! all internal. The in-memory fast path (budget never exceeded) touches the
//! filesystem zero times.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::iter;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrd};

/// Process-wide counter giving each spilled run a unique file name.
static RUN_SEQ: AtomicU64 = AtomicU64::new(0);

/// Approximate per-record bookkeeping cost charged against the memory budget,
/// on top of the payload bytes (key + `Vec` header, roughly).
const RECORD_OVERHEAD: usize = 32;

/// A sorted stream of `(key, bytes)` records — the item type used everywhere.
pub type Record = (u64, Vec<u8>);

/// Boxed sorted source of records (an in-memory drain or a run-file reader).
type Source = Box<dyn Iterator<Item = io::Result<Record>>>;

/// Accumulates `(key, data)` records and sorts them by key, spilling to disk
/// when the in-memory budget is exceeded.
pub struct ExternalSorter {
    tmp_dir: PathBuf,
    mem_budget: usize,
    buf: Vec<Record>,
    buf_bytes: usize,
    runs: Vec<PathBuf>,
}

impl ExternalSorter {
    /// Creates a sorter that spills runs into `tmp_dir` once `mem_budget` bytes
    /// are buffered. No filesystem access happens until the first spill.
    pub fn new(tmp_dir: impl Into<PathBuf>, mem_budget: usize) -> Self {
        ExternalSorter {
            tmp_dir: tmp_dir.into(),
            // Guarantee forward progress: a zero budget would spill empty runs.
            mem_budget: mem_budget.max(1),
            buf: Vec::new(),
            buf_bytes: 0,
            runs: Vec::new(),
        }
    }

    /// Adds one record. May trigger a spill, hence the `io::Result`.
    pub fn add(&mut self, key: u64, data: &[u8]) -> io::Result<()> {
        self.buf.push((key, data.to_vec()));
        self.buf_bytes += data.len() + RECORD_OVERHEAD;
        if self.buf_bytes >= self.mem_budget {
            self.spill()?;
        }
        Ok(())
    }

    /// Sorts the in-memory buffer and writes it as a run file.
    fn spill(&mut self) -> io::Result<()> {
        if self.buf.is_empty() {
            return Ok(());
        }
        self.buf.sort_unstable_by_key(|&(k, _)| k);

        fs::create_dir_all(&self.tmp_dir)?;
        let seq = RUN_SEQ.fetch_add(1, AtomicOrd::Relaxed);
        let path = self
            .tmp_dir
            .join(format!("arpt-sort-{}-{}.run", std::process::id(), seq));

        let mut w = BufWriter::new(File::create(&path)?);
        for (key, data) in &self.buf {
            w.write_all(&key.to_le_bytes())?;
            w.write_all(&(data.len() as u64).to_le_bytes())?;
            w.write_all(data)?;
        }
        w.flush()?;

        self.runs.push(path);
        self.buf.clear();
        self.buf_bytes = 0;
        Ok(())
    }

    /// Finishes sorting and returns a globally key-ordered iterator.
    ///
    /// The returned [`Sorted`] owns the run files and deletes them when dropped.
    pub fn into_sorted(mut self) -> io::Result<Sorted> {
        // Drain the resident buffer as one more sorted source (no extra spill).
        let mut buf = std::mem::take(&mut self.buf);
        buf.sort_unstable_by_key(|&(k, _)| k);
        self.buf_bytes = 0;

        let runs = std::mem::take(&mut self.runs);
        let mut sources: Vec<Source> = Vec::with_capacity(runs.len() + 1);
        for path in &runs {
            sources.push(open_run(path)?);
        }
        if !buf.is_empty() {
            sources.push(Box::new(buf.into_iter().map(Ok)));
        }

        Sorted::new(sources, runs)
    }
}

impl Drop for ExternalSorter {
    fn drop(&mut self) {
        // Clean up runs if the sorter was abandoned without `into_sorted`.
        for path in &self.runs {
            let _ = fs::remove_file(path);
        }
    }
}

/// Opens a run file as a sorted source of records.
fn open_run(path: &Path) -> io::Result<Source> {
    let mut reader = BufReader::new(File::open(path)?);
    Ok(Box::new(iter::from_fn(move || {
        match read_record(&mut reader) {
            Ok(Some(rec)) => Some(Ok(rec)),
            Ok(None) => None,
            Err(e) => Some(Err(e)),
        }
    })))
}

/// Reads one `(key, data)` record, or `None` at a clean end-of-file.
fn read_record(reader: &mut impl Read) -> io::Result<Option<Record>> {
    let mut key_bytes = [0u8; 8];
    match reader.read_exact(&mut key_bytes) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let mut len_bytes = [0u8; 8];
    reader.read_exact(&mut len_bytes)?;
    let len = u64::from_le_bytes(len_bytes) as usize;
    let mut data = vec![0u8; len];
    reader.read_exact(&mut data)?;
    Ok(Some((u64::from_le_bytes(key_bytes), data)))
}

/// One record sitting at the head of a source, ordered for the merge heap.
struct HeapItem {
    key: u64,
    src: usize,
    data: Vec<u8>,
}

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key && self.src == other.src
    }
}
impl Eq for HeapItem {}
impl Ord for HeapItem {
    // BinaryHeap is a max-heap; invert so the smallest key (then source) pops
    // first, giving a stable, deterministic merge order.
    fn cmp(&self, other: &Self) -> Ordering {
        other.key.cmp(&self.key).then_with(|| other.src.cmp(&self.src))
    }
}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Globally key-ordered iterator produced by [`ExternalSorter::into_sorted`].
///
/// Yields `io::Result<Record>` because reading spilled runs can fail. Run files
/// are deleted when this is dropped.
pub struct Sorted {
    sources: Vec<Source>,
    heap: BinaryHeap<HeapItem>,
    pending: Option<io::Error>,
    cleanup: Vec<PathBuf>,
}

impl Sorted {
    fn new(mut sources: Vec<Source>, cleanup: Vec<PathBuf>) -> io::Result<Self> {
        let mut heap = BinaryHeap::with_capacity(sources.len());
        for (src, source) in sources.iter_mut().enumerate() {
            if let Some(item) = pull(source, src)? {
                heap.push(item);
            }
        }
        Ok(Sorted { sources, heap, pending: None, cleanup })
    }
}

/// Pulls the next record from a source into a [`HeapItem`].
fn pull(source: &mut Source, src: usize) -> io::Result<Option<HeapItem>> {
    match source.next() {
        Some(Ok((key, data))) => Ok(Some(HeapItem { key, src, data })),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}

impl Iterator for Sorted {
    type Item = io::Result<Record>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(e) = self.pending.take() {
            return Some(Err(e));
        }
        let item = self.heap.pop()?;
        // Advance the source we just drained; surface any read error next.
        match pull(&mut self.sources[item.src], item.src) {
            Ok(Some(next)) => self.heap.push(next),
            Ok(None) => {}
            Err(e) => self.pending = Some(e),
        }
        Some(Ok((item.key, item.data)))
    }
}

impl Drop for Sorted {
    fn drop(&mut self) {
        // Close file handles before unlinking the run files.
        self.sources.clear();
        for path in &self.cleanup {
            let _ = fs::remove_file(path);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU32;

    /// A unique, freshly-created temp directory for one test.
    fn test_dir(tag: &str) -> PathBuf {
        static SEQ: AtomicU32 = AtomicU32::new(0);
        let n = SEQ.fetch_add(1, AtomicOrd::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "arpt-sort-test-{}-{}-{}",
            std::process::id(),
            tag,
            n
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn collect(sorted: Sorted) -> Vec<Record> {
        sorted.map(|r| r.unwrap()).collect()
    }

    fn count_runs(dir: &Path) -> usize {
        fs::read_dir(dir)
            .map(|rd| {
                rd.filter_map(|e| e.ok())
                    .filter(|e| e.path().extension().is_some_and(|x| x == "run"))
                    .count()
            })
            .unwrap_or(0)
    }

    #[test]
    fn in_memory_path_sorts_without_spilling() {
        let dir = test_dir("mem");
        let mut s = ExternalSorter::new(&dir, 1 << 30); // huge budget
        for &k in &[5u64, 1, 9, 3, 7] {
            s.add(k, format!("v{k}").as_bytes()).unwrap();
        }
        let out = collect(s.into_sorted().unwrap());
        let keys: Vec<u64> = out.iter().map(|(k, _)| *k).collect();
        assert_eq!(keys, vec![1, 3, 5, 7, 9]);
        assert_eq!(out[0].1, b"v1");
        // Fast path must not have created any run files.
        assert_eq!(count_runs(&dir), 0);
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn spilled_runs_merge_in_global_order() {
        let dir = test_dir("spill");
        // Tiny budget forces a spill roughly every record.
        let mut s = ExternalSorter::new(&dir, 1);
        let input: Vec<u64> = vec![42, 7, 100, 7, 1, 99, 50, 2, 2, 75];
        for &k in &input {
            s.add(k, format!("payload-{k}").into_bytes().as_slice()).unwrap();
        }
        let sorted = s.into_sorted().unwrap();
        let out = collect(sorted);

        let keys: Vec<u64> = out.iter().map(|(k, _)| *k).collect();
        let mut expected = input.clone();
        expected.sort_unstable();
        assert_eq!(keys, expected, "keys must be globally sorted");
        // Payload integrity preserved alongside keys.
        for (k, data) in &out {
            assert_eq!(data, format!("payload-{k}").as_bytes());
        }
        // Run files are cleaned up once the iterator is dropped.
        assert_eq!(count_runs(&dir), 0, "run files must be deleted after consumption");
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn handles_duplicate_keys_and_empty_payloads() {
        let dir = test_dir("dup");
        let mut s = ExternalSorter::new(&dir, 64); // small budget → some spills
        s.add(5, b"").unwrap();
        s.add(5, b"a").unwrap();
        s.add(5, b"bb").unwrap();
        s.add(1, b"x").unwrap();
        let out = collect(s.into_sorted().unwrap());
        let keys: Vec<u64> = out.iter().map(|(k, _)| *k).collect();
        assert_eq!(keys, vec![1, 5, 5, 5]);
        assert_eq!(out.len(), 4);
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn empty_sorter_yields_nothing() {
        let dir = test_dir("empty");
        let s = ExternalSorter::new(&dir, 1024);
        assert!(collect(s.into_sorted().unwrap()).is_empty());
        assert_eq!(count_runs(&dir), 0);
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn abandoned_sorter_cleans_up_runs() {
        let dir = test_dir("abandon");
        {
            let mut s = ExternalSorter::new(&dir, 1);
            s.add(3, b"aaa").unwrap();
            s.add(1, b"bbb").unwrap(); // forces spills
            assert!(count_runs(&dir) > 0, "expected spilled runs before drop");
            // Drop without into_sorted.
        }
        assert_eq!(count_runs(&dir), 0, "drop must remove run files");
        fs::remove_dir_all(&dir).ok();
    }
}
