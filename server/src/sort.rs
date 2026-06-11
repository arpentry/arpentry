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

/// Sanity cap on a record payload length read back from a spilled run file.
const MAX_RECORD_LEN: u64 = 64 * 1024 * 1024;

/// Maximum sources (run files + resident buffers) merged in one k-way pass.
/// Every open run costs a file descriptor, and a parallel phase 1 can spill
/// hundreds of runs; batches beyond this are pre-merged into single larger
/// runs, keeping the count well under conservative OS limits (macOS defaults
/// to 256 descriptors per process).
const MAX_MERGE_SOURCES: usize = 64;

/// Approximate per-record bookkeeping cost charged against the memory budget,
/// on top of the payload bytes (key + `Vec` header, roughly).
const RECORD_OVERHEAD: usize = 32;

/// A sorted stream of `(key, bytes)` records — the item type used everywhere.
pub type Record = (u64, Vec<u8>);

/// Boxed sorted source of records (an in-memory drain or a run-file reader).
/// `Send` so the consuming [`Sorted`] stream can move to another thread.
type Source = Box<dyn Iterator<Item = io::Result<Record>> + Send>;

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
        let path = new_run_path(&self.tmp_dir);
        let mut w = BufWriter::new(File::create(&path)?);
        for (key, data) in &self.buf {
            write_record(&mut w, *key, data)?;
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
    pub fn into_sorted(self) -> io::Result<Sorted> {
        merge(vec![self])
    }
}

/// Merges several independently filled sorters into one globally key-ordered
/// stream — the join point after parallel workers each fed their own sorter.
/// Every spilled run and resident buffer becomes one source of a k-way merge;
/// when there are more than [`MAX_MERGE_SOURCES`], batches of runs are first
/// pre-merged into single larger runs so the final pass never holds more than
/// that many files open at once.
pub fn merge(sorters: Vec<ExternalSorter>) -> io::Result<Sorted> {
    let tmp_dir =
        sorters.first().map(|s| s.tmp_dir.clone()).unwrap_or_else(std::env::temp_dir);
    let mut runs: Vec<PathBuf> = Vec::new();
    let mut buffers: Vec<Vec<Record>> = Vec::new();
    for mut sorter in sorters {
        // Drain the resident buffer as one more sorted source (no extra spill).
        let mut buf = std::mem::take(&mut sorter.buf);
        buf.sort_unstable_by_key(|&(k, _)| k);
        sorter.buf_bytes = 0;
        if !buf.is_empty() {
            buffers.push(buf);
        }
        runs.extend(std::mem::take(&mut sorter.runs));
    }

    // Cap the fan-in (file descriptors) of the final merge.
    while runs.len() + buffers.len() > MAX_MERGE_SOURCES && runs.len() >= 2 {
        let n = runs.len().min(MAX_MERGE_SOURCES);
        let batch: Vec<PathBuf> = runs.drain(..n).collect();
        runs.push(merge_runs_to_file(&tmp_dir, batch)?);
    }

    let mut sources: Vec<Source> = Vec::with_capacity(runs.len() + buffers.len());
    for path in &runs {
        sources.push(open_run(path)?);
    }
    for buf in buffers {
        sources.push(Box::new(buf.into_iter().map(Ok)));
    }
    Sorted::new(sources, runs)
}

/// K-way merges a batch of run files into one new run file, deleting the
/// originals. Holds `batch.len() + 1` descriptors while it runs.
fn merge_runs_to_file(tmp_dir: &Path, batch: Vec<PathBuf>) -> io::Result<PathBuf> {
    let mut sources: Vec<Source> = Vec::with_capacity(batch.len());
    for path in &batch {
        sources.push(open_run(path)?);
    }
    // `Sorted` owns the batch files and removes them once fully drained.
    let merged = Sorted::new(sources, batch)?;

    fs::create_dir_all(tmp_dir)?;
    let path = new_run_path(tmp_dir);
    let mut w = BufWriter::new(File::create(&path)?);
    let write_all = |w: &mut BufWriter<File>| -> io::Result<()> {
        for rec in merged {
            let (key, data) = rec?;
            write_record(w, key, &data)?;
        }
        w.flush()
    };
    match write_all(&mut w) {
        Ok(()) => Ok(path),
        Err(e) => {
            drop(w);
            let _ = fs::remove_file(&path);
            Err(e)
        }
    }
}

/// Allocates a unique run-file path in `tmp_dir`.
fn new_run_path(tmp_dir: &Path) -> PathBuf {
    let seq = RUN_SEQ.fetch_add(1, AtomicOrd::Relaxed);
    tmp_dir.join(format!("arpt-sort-{}-{}.run", std::process::id(), seq))
}

/// Writes one `(key, data)` record in the on-disk run format.
fn write_record(w: &mut impl Write, key: u64, data: &[u8]) -> io::Result<()> {
    w.write_all(&key.to_le_bytes())?;
    w.write_all(&(data.len() as u64).to_le_bytes())?;
    w.write_all(data)
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
    let len = u64::from_le_bytes(len_bytes);
    // A corrupt or truncated run file yields a clean error here instead of a
    // giant allocation (no real record payload approaches this size).
    if len > MAX_RECORD_LEN {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "record length exceeds sanity cap"));
    }
    let mut data = vec![0u8; len as usize];
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
    fn merge_of_several_sorters_is_globally_sorted() {
        let dir = test_dir("merge");
        // Three sorters with overlapping key ranges; tiny budgets force spills
        // in some and leave others fully resident.
        let mut a = ExternalSorter::new(&dir, 1);
        let mut b = ExternalSorter::new(&dir, 1 << 20);
        let mut c = ExternalSorter::new(&dir, 24);
        for &k in &[10u64, 4, 7] {
            a.add(k, format!("a{k}").as_bytes()).unwrap();
        }
        for &k in &[3u64, 11, 7] {
            b.add(k, format!("b{k}").as_bytes()).unwrap();
        }
        for &k in &[1u64, 12] {
            c.add(k, format!("c{k}").as_bytes()).unwrap();
        }
        let out = collect(merge(vec![a, b, c]).unwrap());
        let keys: Vec<u64> = out.iter().map(|(k, _)| *k).collect();
        assert_eq!(keys, vec![1, 3, 4, 7, 7, 10, 11, 12]);
        // Payloads still travel with their keys across the merge.
        for (k, data) in &out {
            assert_eq!(&data[1..], k.to_string().as_bytes());
        }
        assert_eq!(count_runs(&dir), 0, "all sorters' runs cleaned up");
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn merge_caps_fan_in_with_hundreds_of_runs() {
        let dir = test_dir("fanin");
        // Three sorters spilling every record → 120 runs, well over
        // MAX_MERGE_SOURCES, forcing at least one pre-merge pass.
        let mut sorters = Vec::new();
        let mut expected: Vec<u64> = Vec::new();
        for s in 0..3u64 {
            let mut sorter = ExternalSorter::new(&dir, 1);
            for i in 0..40u64 {
                let k = (i * 7 + s * 3) % 100;
                sorter.add(k, format!("{s}-{i}-{k}").as_bytes()).unwrap();
                expected.push(k);
            }
            sorters.push(sorter);
        }
        assert!(count_runs(&dir) > MAX_MERGE_SOURCES, "need enough runs to force pre-merge");

        let out = collect(merge(sorters).unwrap());
        expected.sort_unstable();
        let keys: Vec<u64> = out.iter().map(|(k, _)| *k).collect();
        assert_eq!(keys, expected, "pre-merged stream must stay globally sorted");
        // Every payload survives the rewrite, attached to its key.
        for (k, data) in &out {
            assert!(data.ends_with(format!("-{k}").as_bytes()));
        }
        assert_eq!(count_runs(&dir), 0, "intermediate and original runs cleaned up");
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
