//! Stage snapshots: a pipeline stage's output, written to disk and read back.
//!
//! The pipeline is assemble → solve → ground → synth → emit, and until now only
//! the last of those produced anything durable. `--dump` writes each stage as
//! GeoJSON, but one way: for a human to open in QGIS, never read back and never
//! compared. So there was no way to run a stage without running every stage
//! before it, and no way to ask *which* stage a scorecard movement came from.
//!
//! A snapshot is that boundary made real (`schemas/stage.fbs`). Today it holds
//! the scene — the assemble stage's output — and the format has room for the
//! solved model and the ground beside it.
//!
//! # A snapshot is a cache, never a source
//!
//! Nothing should ever hand-author one, and reusing one is only safe while the
//! thing that produced it has not moved. This project has been bitten by
//! exactly that: a baseline re-cut against a binary that was never rebuilt, and
//! a terrain extract silently reused into a zoom it did not cover. So
//! [`Provenance`] fingerprints the inputs *and the tiler binary*, and
//! [`Snapshot::reusable_for`] refuses out loud rather than reusing quietly. A
//! stale cache that reports itself is an inconvenience; one that does not is a
//! week.
//!
//! # Why the writer and the reader are one file
//!
//! The failure mode of a hand-written codec is a field added to the struct and
//! forgotten in the codec — a snapshot that is subtly, silently wrong, which is
//! the precise class of defect this harness exists to eliminate. Two things
//! hold it off: every conversion here is an exhaustive `match` where it can be,
//! so a new [`Kind`] class breaks the build rather than serialising as
//! something else; and [`tests::a_scene_survives_the_round_trip`] compares a
//! reloaded scene field by field against the one that was written.

use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use flatbuffers::{FlatBufferBuilder, WIPOffset};
use geo_types::Coord;

use crate::assemble::columns::Connector;
use crate::assemble::walks::{Attachment, Census, Evidence, WalkLine, Walks};
use crate::fb::stage::arpentry::stage as fb;
use crate::priors::{Kind, RailClass, RoadClass, WaterClass};
use crate::project::Bounds;
use crate::scene::{
    Corridor, Junction, JunctionMember, SceneGraph, SegmentRef, Span, SpanKind, WaterBody,
};
use crate::value::Value;

/// What a snapshot was built from. Compared before a snapshot is reused.
#[derive(Debug, Clone, PartialEq)]
pub struct Provenance {
    pub bbox: Bounds,
    /// The `--terrain` value verbatim — a DEM path or a resolved field spec.
    /// The ground a stage was solved against is part of what that stage *is*.
    pub terrain: String,
    /// `(layer index, size, mtime seconds)` per input, in layer order. Size and
    /// mtime rather than a content hash: hashing a 954 MB parquet on every run
    /// would cost more than the stage the snapshot is saving, and both move for
    /// every change that matters.
    pub inputs: Vec<(u8, u64, i64)>,
    /// The tiler binary's own mtime, in seconds. The stale-binary guard.
    pub built_by: i64,
    pub built_in_s: f64,
    pub written_at: String,
}

impl Provenance {
    /// The provenance of a run: what it read, over what bounds, with which
    /// ground, built by which binary.
    pub fn of_run(bbox: Bounds, terrain: Option<&Path>, inputs: &[(u8, std::path::PathBuf)]) -> Provenance {
        let stamps = inputs
            .iter()
            .map(|(layer, path)| {
                let (size, mtime) = stat(path);
                (*layer, size, mtime)
            })
            .collect();
        Provenance {
            bbox,
            terrain: terrain.map(|p| p.to_string_lossy().into_owned()).unwrap_or_default(),
            inputs: stamps,
            built_by: current_exe_mtime(),
            built_in_s: 0.0,
            written_at: iso_now(),
        }
    }

    /// Whether this snapshot may stand in for `want`.
    ///
    /// **Two levels, because two different things are being asked.** A
    /// different bbox, ground or input makes the snapshot *wrong* for this
    /// run — facts, not judgement calls, so they refuse. A rebuilt binary is
    /// different: it fires on every rebuild, which is precisely the loop this
    /// flag exists to serve, and it cannot tell "I changed the drawing" from
    /// "I changed the assembler". Refusing there would make the cache useless
    /// for its only use; reusing silently is how this project lost a week to a
    /// baseline cut against a binary that was never rebuilt. So it warns, and
    /// the developer — who knows which stage they touched — decides.
    ///
    /// The reason is carried rather than a bool: "stale" sends someone to
    /// re-run everything, where "built for a different bbox" sends them to the
    /// one flag they changed.
    pub fn verdict(&self, want: &Provenance) -> Verdict {
        if let Some(why) = self.refuses(want) {
            return Verdict::Refuse(why);
        }
        if self.built_by < want.built_by {
            return Verdict::Warn(
                "the tiler has been rebuilt since it was written — reusing it anyway, \
                 which is right only if the change was downstream of this stage"
                    .to_string(),
            );
        }
        Verdict::Reuse
    }

    /// The facts that make a snapshot wrong for `want`, whatever anyone
    /// intends. `None` when none of them hold.
    fn refuses(&self, want: &Provenance) -> Option<String> {
        if self.bbox != want.bbox {
            return Some(format!(
                "built for bbox {:.4},{:.4},{:.4},{:.4}, this run wants {:.4},{:.4},{:.4},{:.4}",
                self.bbox.west,
                self.bbox.south,
                self.bbox.east,
                self.bbox.north,
                want.bbox.west,
                want.bbox.south,
                want.bbox.east,
                want.bbox.north
            ));
        }
        if self.terrain != want.terrain {
            return Some(format!(
                "built against ground `{}`, this run wants `{}`",
                self.terrain, want.terrain
            ));
        }
        if self.inputs != want.inputs {
            return Some("an input has changed since it was built".to_string());
        }
        None
    }
}

/// What a run may do with a snapshot it found.
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    /// Nothing it was built from has moved.
    Reuse,
    /// Usable, with a caveat the run must print.
    Warn(String),
    /// Wrong for this run; compute the stage instead.
    Refuse(String),
}

/// A stage snapshot in memory: whichever artifacts a run chose to keep.
pub struct Snapshot {
    pub provenance: Provenance,
    /// `None` means "not captured", which is not the same as an empty scene —
    /// an extract with no transportation input legitimately assembles nothing.
    pub scene: Option<SceneGraph>,
}

/// Writes a snapshot to `path`.
pub fn write(path: &Path, snap: &Snapshot) -> io::Result<()> {
    let mut b = FlatBufferBuilder::with_capacity(1 << 20);
    let scene = snap.scene.as_ref().map(|s| write_scene(&mut b, s));
    let prov = write_provenance(&mut b, &snap.provenance);
    let root = fb::Snapshot::create(
        &mut b,
        &fb::SnapshotArgs { provenance: Some(prov), scene },
    );
    b.finish(root, Some("arpw"));
    // Written whole then renamed, like the archive: a snapshot half-written by
    // an interrupted run must not be readable, or the next run reuses a
    // truncated scene and blames the geometry.
    let tmp = path.with_extension("arpw.tmp");
    std::fs::write(&tmp, b.finished_data())?;
    std::fs::rename(&tmp, path)
}

/// Reads a snapshot from `path`.
pub fn read(path: &Path) -> io::Result<Snapshot> {
    let bytes = std::fs::read(path)?;
    let root = fb::root_as_snapshot(&bytes)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{}: {e}", path.display())))?;
    let p = root.provenance();
    Ok(Snapshot {
        provenance: Provenance {
            bbox: Bounds {
                west: p.bbox_west(),
                south: p.bbox_south(),
                east: p.bbox_east(),
                north: p.bbox_north(),
            },
            terrain: p.terrain().unwrap_or_default().to_string(),
            inputs: p
                .inputs()
                .map(|v| v.iter().map(|s| (s.layer(), s.bytes(), s.mtime())).collect())
                .unwrap_or_default(),
            built_by: p.built_by(),
            built_in_s: p.built_in_s(),
            written_at: p.written_at().unwrap_or_default().to_string(),
        },
        scene: root.scene().map(read_scene),
    })
}

// ── scene ───────────────────────────────────────────────────────────────────

fn write_scene<'a>(
    b: &mut FlatBufferBuilder<'a>,
    scene: &SceneGraph,
) -> WIPOffset<fb::Scene<'a>> {
    let corridors: Vec<_> = scene.corridors.iter().map(|c| write_corridor(b, c)).collect();
    let corridors = b.create_vector(&corridors);

    let junctions: Vec<_> = scene
        .junctions
        .iter()
        .map(|j| {
            let members: Vec<fb::JunctionMember> = j
                .members
                .iter()
                .map(|m| fb::JunctionMember::new(m.corridor, m.arc))
                .collect();
            let members = b.create_vector(&members);
            let point = coord(&j.point);
            fb::Junction::create(
                b,
                &fb::JunctionArgs {
                    point: Some(&point),
                    connector: j.connector,
                    members: Some(members),
                },
            )
        })
        .collect();
    let junctions = b.create_vector(&junctions);

    let water: Vec<_> = scene
        .water
        .iter()
        .map(|w| {
            let exterior = coords(b, &w.exterior);
            let holes: Vec<_> = w
                .holes
                .iter()
                .map(|h| {
                    let points = coords(b, h);
                    fb::Ring::create(b, &fb::RingArgs { points: Some(points) })
                })
                .collect();
            let holes = b.create_vector(&holes);
            fb::WaterBody::create(
                b,
                &fb::WaterBodyArgs {
                    exterior: Some(exterior),
                    holes: Some(holes),
                    bbox_west: w.bbox.0,
                    bbox_south: w.bbox.1,
                    bbox_east: w.bbox.2,
                    bbox_north: w.bbox.3,
                },
            )
        })
        .collect();
    let water = b.create_vector(&water);

    let witnesses = polylines(b, &scene.witnesses);
    let flows = polylines(b, &scene.flows);
    let walks = write_walks(b, &scene.walks);

    fb::Scene::create(
        b,
        &fb::SceneArgs {
            corridors: Some(corridors),
            junctions: Some(junctions),
            water: Some(water),
            witnesses: Some(witnesses),
            flows: Some(flows),
            walks: Some(walks),
        },
    )
}

fn read_scene(s: fb::Scene<'_>) -> SceneGraph {
    let corridors: Vec<Corridor> =
        s.corridors().map(|v| v.iter().map(read_corridor).collect()).unwrap_or_default();

    let junctions: Vec<Junction> = s
        .junctions()
        .map(|v| {
            v.iter()
                .map(|j| Junction {
                    point: uncoord(j.point().expect("a junction point")),
                    connector: j.connector(),
                    members: j
                        .members()
                        .map(|m| {
                            m.iter()
                                .map(|m| JunctionMember { corridor: m.corridor(), arc: m.arc() })
                                .collect()
                        })
                        .unwrap_or_default(),
                })
                .collect()
        })
        .unwrap_or_default();

    let water: Vec<WaterBody> = s
        .water()
        .map(|v| {
            v.iter()
                .map(|w| WaterBody {
                    exterior: uncoords(w.exterior()),
                    holes: w
                        .holes()
                        .map(|h| h.iter().map(|r| uncoords(r.points())).collect())
                        .unwrap_or_default(),
                    bbox: (w.bbox_west(), w.bbox_south(), w.bbox_east(), w.bbox_north()),
                })
                .collect()
        })
        .unwrap_or_default();

    let witnesses = s
        .witnesses()
        .map(|v| v.iter().map(|p| uncoords(p.points())).collect())
        .unwrap_or_default();
    let flows =
        s.flows().map(|v| v.iter().map(|p| uncoords(p.points())).collect()).unwrap_or_default();

    // `SceneGraph::new` rebuilds the source lookup, so the scene comes back
    // indexed exactly as assemble left it rather than carrying a stored index
    // that could disagree with the corridors it points into.
    let mut scene = SceneGraph::new(corridors);
    scene.junctions = junctions;
    scene.water = water;
    scene.witnesses = witnesses;
    scene.flows = flows;
    if let Some(w) = s.walks() {
        scene.walks = read_walks(w);
    }
    scene
}

fn write_corridor<'a>(
    b: &mut FlatBufferBuilder<'a>,
    c: &Corridor,
) -> WIPOffset<fb::Corridor<'a>> {
    let nodes = coords(b, &c.nodes);
    let arc = b.create_vector(&c.arc);
    let class_key = b.create_string(&c.class_key);
    let spans: Vec<fb::Span> = c
        .spans
        .iter()
        .map(|s| fb::Span::new(s.arc0, s.arc1, s.level, span_kind(s.kind)))
        .collect();
    let spans = b.create_vector(&spans);
    let segments: Vec<_> = c.segments.iter().map(|s| write_segment(b, s)).collect();
    let segments = b.create_vector(&segments);
    let ids: Vec<u64> = c.connectors.iter().map(|(id, _)| *id).collect();
    let arcs: Vec<f64> = c.connectors.iter().map(|(_, a)| *a).collect();
    let connector_ids = b.create_vector(&ids);
    let connector_arcs = b.create_vector(&arcs);
    let kind = kind_to_fb(c.kind);
    fb::Corridor::create(
        b,
        &fb::CorridorArgs {
            id: c.id,
            nodes: Some(nodes),
            arc: Some(arc),
            cos_lat: c.cos_lat,
            kind: Some(&kind),
            class_key: Some(class_key),
            link: c.link,
            // `None` travels as NaN: a FlatBuffers scalar cannot be absent, and
            // no real width is NaN, so the two cannot be confused.
            width_m: c.width_m.unwrap_or(f64::NAN),
            spans: Some(spans),
            segments: Some(segments),
            connector_ids: Some(connector_ids),
            connector_arcs: Some(connector_arcs),
        },
    )
}

fn read_corridor(c: fb::Corridor<'_>) -> Corridor {
    let ids = c.connector_ids();
    let arcs = c.connector_arcs();
    let connectors = match (ids, arcs) {
        (Some(i), Some(a)) => i.iter().zip(a.iter()).collect(),
        _ => Vec::new(),
    };
    let width = c.width_m();
    Corridor {
        id: c.id(),
        nodes: uncoords(c.nodes()),
        arc: c.arc().map(|v| v.iter().collect()).unwrap_or_default(),
        cos_lat: c.cos_lat(),
        kind: kind_from_fb(c.kind().expect("a corridor kind")),
        class_key: c.class_key().to_string(),
        link: c.link(),
        width_m: if width.is_nan() { None } else { Some(width) },
        spans: c
            .spans()
            .map(|v| {
                v.iter()
                    .map(|s| Span {
                        arc0: s.arc0(),
                        arc1: s.arc1(),
                        level: s.level(),
                        kind: span_kind_from(s.kind()),
                    })
                    .collect()
            })
            .unwrap_or_default(),
        segments: c.segments().map(|v| v.iter().map(read_segment).collect()).unwrap_or_default(),
        connectors,
    }
}

fn write_segment<'a>(
    b: &mut FlatBufferBuilder<'a>,
    s: &SegmentRef,
) -> WIPOffset<fb::SegmentRef<'a>> {
    let props: Vec<_> = s
        .properties
        .iter()
        .map(|(k, v)| {
            let key = b.create_string(k);
            let (kind, sv, i, d) = match v {
                Value::String(s) => (fb::ValueKind::String, Some(b.create_string(s)), 0, 0.0),
                Value::Int(n) => (fb::ValueKind::Int, None, *n, 0.0),
                Value::Double(x) => (fb::ValueKind::Double, None, 0, *x),
                Value::Bool(t) => (fb::ValueKind::Bool, None, i64::from(*t), 0.0),
            };
            fb::Property::create(
                b,
                &fb::PropertyArgs { key: Some(key), kind, s: sv, i, d },
            )
        })
        .collect();
    let properties = b.create_vector(&props);
    fb::SegmentRef::create(
        b,
        &fb::SegmentRefArgs {
            source: s.source,
            node0: s.node0 as u32,
            node1: s.node1 as u32,
            properties: Some(properties),
        },
    )
}

fn read_segment(s: fb::SegmentRef<'_>) -> SegmentRef {
    SegmentRef {
        source: s.source(),
        node0: s.node0() as usize,
        node1: s.node1() as usize,
        properties: s
            .properties()
            .map(|v| {
                v.iter()
                    .map(|p| {
                        let key = p.key().to_string();
                        let value = match p.kind() {
                            fb::ValueKind::String => {
                                Value::String(p.s().unwrap_or_default().to_string())
                            }
                            fb::ValueKind::Int => Value::Int(p.i()),
                            fb::ValueKind::Double => Value::Double(p.d()),
                            _ => Value::Bool(p.i() != 0),
                        };
                        (key, value)
                    })
                    .collect()
            })
            .unwrap_or_default(),
    }
}

// ── walks ───────────────────────────────────────────────────────────────────

fn write_walks<'a>(b: &mut FlatBufferBuilder<'a>, w: &Walks) -> WIPOffset<fb::Walks<'a>> {
    let parts = w.parts();
    let lines: Vec<_> = parts
        .0
        .iter()
        .map(|l| {
            let line = coords(b, &l.line);
            let conns: Vec<fb::Connector> =
                l.connectors.iter().map(|c| fb::Connector::new(c.id, c.at)).collect();
            let connectors = b.create_vector(&conns);
            let spans = ranges(b, &l.spans);
            let indoor = ranges(b, &l.indoor);
            let kind = kind_to_fb(l.kind);
            fb::WalkLine::create(
                b,
                &fb::WalkLineArgs {
                    source: l.source,
                    line: Some(line),
                    kind: Some(&kind),
                    tagged: l.tagged,
                    crosswalk: l.crosswalk,
                    connectors: Some(connectors),
                    spans: Some(spans),
                    indoor: Some(indoor),
                },
            )
        })
        .collect();
    let lines = b.create_vector(&lines);
    let starts: Vec<u32> = parts.1.iter().map(|r| r.0).collect();
    let ends: Vec<u32> = parts.1.iter().map(|r| r.1).collect();
    let line_range_start = b.create_vector(&starts);
    let line_range_end = b.create_vector(&ends);
    let atts: Vec<fb::Attachment> = parts
        .2
        .iter()
        .map(|a| {
            fb::Attachment::new(
                a.walk,
                a.line,
                a.walk0,
                a.walk1,
                &kind_to_fb(a.kind),
                a.host,
                a.side,
                a.arc0,
                a.arc1,
                a.offset_m,
                a.spread_m,
                evidence_to_fb(a.evidence),
            )
        })
        .collect();
    let attachments = b.create_vector(&atts);
    let c = parts.3;
    let census = fb::Census::new(
        c.lines,
        c.line_m,
        c.tagged_lines,
        c.tagged_m,
        c.covered_m,
        c.tagged_covered_m,
        c.alongside_m,
        c.attached_lines,
        c.attached_m,
        c.host_arc_m,
        c.tag_only,
        c.alongside_only,
        c.both,
        c.tagged_unhosted,
        c.joined_only,
        c.dropped_short,
        c.dropped_short_m,
        c.broke_lost,
        c.broke_crossed,
        c.broke_host,
        c.broke_side,
        c.broke_turned,
    );
    fb::Walks::create(
        b,
        &fb::WalksArgs {
            lines: Some(lines),
            line_range_start: Some(line_range_start),
            line_range_end: Some(line_range_end),
            attachments: Some(attachments),
            census: Some(&census),
        },
    )
}

fn read_walks(w: fb::Walks<'_>) -> Walks {
    let lines: Vec<WalkLine> = w
        .lines()
        .map(|v| {
            v.iter()
                .map(|l| WalkLine {
                    source: l.source(),
                    line: uncoords(l.line()),
                    kind: kind_from_fb(l.kind().expect("a walk line kind")),
                    tagged: l.tagged(),
                    crosswalk: l.crosswalk(),
                    connectors: l
                        .connectors()
                        .map(|c| {
                            c.iter()
                                .map(|c| Connector { id: c.source(), at: c.arc() })
                                .collect()
                        })
                        .unwrap_or_default(),
                    spans: unranges(l.spans()),
                    indoor: unranges(l.indoor()),
                })
                .collect()
        })
        .unwrap_or_default();
    let starts: Vec<u32> = w.line_range_start().map(|v| v.iter().collect()).unwrap_or_default();
    let ends: Vec<u32> = w.line_range_end().map(|v| v.iter().collect()).unwrap_or_default();
    let ranges: Vec<(u32, u32)> = starts.into_iter().zip(ends).collect();
    let attachments: Vec<Attachment> = w
        .attachments()
        .map(|v| {
            v.iter()
                .map(|a| Attachment {
                    walk: a.walk(),
                    line: a.line(),
                    walk0: a.walk0(),
                    walk1: a.walk1(),
                    kind: kind_from_fb(&a.kind()),
                    host: a.host(),
                    side: a.side(),
                    arc0: a.arc0(),
                    arc1: a.arc1(),
                    offset_m: a.offset_m(),
                    spread_m: a.spread_m(),
                    evidence: evidence_from_fb(a.evidence()),
                })
                .collect()
        })
        .unwrap_or_default();
    let c = w.census().expect("a census");
    let census = Census {
        lines: c.lines(),
        line_m: c.line_m(),
        tagged_lines: c.tagged_lines(),
        tagged_m: c.tagged_m(),
        covered_m: c.covered_m(),
        tagged_covered_m: c.tagged_covered_m(),
        alongside_m: c.alongside_m(),
        attached_lines: c.attached_lines(),
        attached_m: c.attached_m(),
        host_arc_m: c.host_arc_m(),
        tag_only: c.tag_only(),
        alongside_only: c.alongside_only(),
        both: c.both(),
        tagged_unhosted: c.tagged_unhosted(),
        joined_only: c.joined_only(),
        dropped_short: c.dropped_short(),
        dropped_short_m: c.dropped_short_m(),
        broke_lost: c.broke_lost(),
        broke_crossed: c.broke_crossed(),
        broke_host: c.broke_host(),
        broke_side: c.broke_side(),
        broke_turned: c.broke_turned(),
    };
    // The by-host and by-source indexes are rebuilt, never stored: an index
    // that disagreed with the attachments it indexes is a defect no check
    // could see.
    Walks::from_parts(lines, ranges, attachments, census)
}

// ── provenance ──────────────────────────────────────────────────────────────

fn write_provenance<'a>(
    b: &mut FlatBufferBuilder<'a>,
    p: &Provenance,
) -> WIPOffset<fb::Provenance<'a>> {
    let terrain = b.create_string(&p.terrain);
    let written_at = b.create_string(&p.written_at);
    let stamps: Vec<fb::InputStamp> =
        p.inputs.iter().map(|(l, s, m)| fb::InputStamp::new(*l, *s, *m)).collect();
    let inputs = b.create_vector(&stamps);
    fb::Provenance::create(
        b,
        &fb::ProvenanceArgs {
            bbox_west: p.bbox.west,
            bbox_south: p.bbox.south,
            bbox_east: p.bbox.east,
            bbox_north: p.bbox.north,
            terrain: Some(terrain),
            inputs: Some(inputs),
            built_by: p.built_by,
            built_in_s: p.built_in_s,
            written_at: Some(written_at),
        },
    )
}

// ── leaf conversions ────────────────────────────────────────────────────────

fn coord(c: &Coord<f64>) -> fb::Coord {
    fb::Coord::new(c.x, c.y)
}

fn coords<'a>(
    b: &mut FlatBufferBuilder<'a>,
    v: &[Coord<f64>],
) -> WIPOffset<flatbuffers::Vector<'a, fb::Coord>> {
    let cs: Vec<fb::Coord> = v.iter().map(coord).collect();
    b.create_vector(&cs)
}

fn uncoord(c: &fb::Coord) -> Coord<f64> {
    Coord { x: c.lon(), y: c.lat() }
}

fn uncoords(v: Option<flatbuffers::Vector<'_, fb::Coord>>) -> Vec<Coord<f64>> {
    v.map(|v| v.iter().map(uncoord).collect()).unwrap_or_default()
}

fn ranges<'a>(
    b: &mut FlatBufferBuilder<'a>,
    v: &[(f64, f64)],
) -> WIPOffset<flatbuffers::Vector<'a, fb::Range>> {
    let rs: Vec<fb::Range> = v.iter().map(|(a, b)| fb::Range::new(*a, *b)).collect();
    b.create_vector(&rs)
}

fn unranges(v: Option<flatbuffers::Vector<'_, fb::Range>>) -> Vec<(f64, f64)> {
    v.map(|v| v.iter().map(|r| (r.a(), r.b())).collect()).unwrap_or_default()
}

/// Which evidence made a way a sidewalk. Exhaustive for the same reason
/// [`kind_to_fb`] is: a variant added and forgotten here would silently
/// re-label every attachment that carried it.
fn evidence_to_fb(e: Evidence) -> fb::Evidence {
    match e {
        Evidence::Tag => fb::Evidence::Tag,
        Evidence::Alongside => fb::Evidence::Alongside,
        Evidence::Both => fb::Evidence::Both,
    }
}

fn evidence_from_fb(e: fb::Evidence) -> Evidence {
    match e {
        fb::Evidence::Alongside => Evidence::Alongside,
        fb::Evidence::Both => Evidence::Both,
        _ => Evidence::Tag,
    }
}

fn polylines<'a>(
    b: &mut FlatBufferBuilder<'a>,
    lines: &[Vec<Coord<f64>>],
) -> WIPOffset<flatbuffers::Vector<'a, flatbuffers::ForwardsUOffset<fb::Polyline<'a>>>> {
    let ls: Vec<_> = lines
        .iter()
        .map(|l| {
            let points = coords(b, l);
            fb::Polyline::create(b, &fb::PolylineArgs { points: Some(points) })
        })
        .collect();
    b.create_vector(&ls)
}

fn span_kind(k: SpanKind) -> fb::SpanKind {
    match k {
        SpanKind::Grade => fb::SpanKind::Grade,
        SpanKind::Bridge => fb::SpanKind::Bridge,
        SpanKind::Tunnel => fb::SpanKind::Tunnel,
    }
}

fn span_kind_from(k: fb::SpanKind) -> SpanKind {
    match k {
        fb::SpanKind::Bridge => SpanKind::Bridge,
        fb::SpanKind::Tunnel => SpanKind::Tunnel,
        _ => SpanKind::Grade,
    }
}

/// `Kind` as (family, class ordinal).
///
/// **Exhaustive on purpose.** A class added to `priors` and forgotten here
/// would serialise as a different road, and nothing downstream could tell —
/// the corridor would simply be the wrong class in every run that read the
/// snapshot. Written as a full match so that mistake is a compile error.
fn kind_to_fb(k: Kind) -> fb::Kind {
    let (family, class) = match k {
        Kind::Road(c) => (
            fb::KindFamily::Road,
            match c {
                RoadClass::Motorway => 0,
                RoadClass::Trunk => 1,
                RoadClass::Primary => 2,
                RoadClass::Secondary => 3,
                RoadClass::Tertiary => 4,
                RoadClass::Unclassified => 5,
                RoadClass::Residential => 6,
                RoadClass::LivingStreet => 7,
                RoadClass::Service => 8,
                RoadClass::Unknown => 9,
                RoadClass::Track => 10,
                RoadClass::Footway => 11,
                RoadClass::Pedestrian => 12,
                RoadClass::Path => 13,
                RoadClass::Steps => 14,
                RoadClass::Cycleway => 15,
                RoadClass::Bridleway => 16,
                RoadClass::Other => 17,
            },
        ),
        Kind::Rail(c) => (
            fb::KindFamily::Rail,
            match c {
                RailClass::StandardGauge => 0,
                RailClass::NarrowGauge => 1,
                RailClass::BroadGauge => 2,
                RailClass::Funicular => 3,
                RailClass::Subway => 4,
                RailClass::LightRail => 5,
                RailClass::Monorail => 6,
                RailClass::Tram => 7,
                RailClass::Unknown => 8,
            },
        ),
        Kind::Water(c) => (
            fb::KindFamily::Water,
            match c {
                WaterClass::Still => 0,
                WaterClass::Watercourse => 1,
            },
        ),
    };
    fb::Kind::new(family, class)
}

fn kind_from_fb(k: &fb::Kind) -> Kind {
    match k.family() {
        fb::KindFamily::Rail => Kind::Rail(match k.class() {
            0 => RailClass::StandardGauge,
            1 => RailClass::NarrowGauge,
            2 => RailClass::BroadGauge,
            3 => RailClass::Funicular,
            4 => RailClass::Subway,
            5 => RailClass::LightRail,
            6 => RailClass::Monorail,
            7 => RailClass::Tram,
            _ => RailClass::Unknown,
        }),
        fb::KindFamily::Water => Kind::Water(match k.class() {
            1 => WaterClass::Watercourse,
            _ => WaterClass::Still,
        }),
        _ => Kind::Road(match k.class() {
            0 => RoadClass::Motorway,
            1 => RoadClass::Trunk,
            2 => RoadClass::Primary,
            3 => RoadClass::Secondary,
            4 => RoadClass::Tertiary,
            5 => RoadClass::Unclassified,
            6 => RoadClass::Residential,
            7 => RoadClass::LivingStreet,
            8 => RoadClass::Service,
            9 => RoadClass::Unknown,
            10 => RoadClass::Track,
            11 => RoadClass::Footway,
            12 => RoadClass::Pedestrian,
            13 => RoadClass::Path,
            14 => RoadClass::Steps,
            15 => RoadClass::Cycleway,
            16 => RoadClass::Bridleway,
            _ => RoadClass::Other,
        }),
    }
}

// ── environment ─────────────────────────────────────────────────────────────

fn stat(path: &Path) -> (u64, i64) {
    match std::fs::metadata(path) {
        Ok(m) => (m.len(), secs(m.modified().ok())),
        Err(_) => (0, 0),
    }
}

fn current_exe_mtime() -> i64 {
    std::env::current_exe().ok().map(|p| stat(&p).1).unwrap_or(0)
}

fn secs(t: Option<SystemTime>) -> i64 {
    t.and_then(|t| t.duration_since(UNIX_EPOCH).ok()).map(|d| d.as_secs() as i64).unwrap_or(0)
}

/// A timestamp a human can read in the file. Hand-rolled rather than pulling a
/// date crate for one line — the repo's standing trade.
fn iso_now() -> String {
    let s = secs(Some(SystemTime::now()));
    let (days, rem) = (s / 86_400, s % 86_400);
    let (mut y, mut d) = (1970i64, days);
    loop {
        let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
        let len = if leap { 366 } else { 365 };
        if d < len {
            break;
        }
        d -= len;
        y += 1;
    }
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let months = [31, if leap { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let mut m = 0;
    while d >= months[m] {
        d -= months[m];
        m += 1;
    }
    format!(
        "{y:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
        m + 1,
        d + 1,
        rem / 3600,
        (rem % 3600) / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox() -> Bounds {
        Bounds { west: 6.9, south: 46.4, east: 6.93, north: 46.45 }
    }

    fn prov() -> Provenance {
        Provenance {
            bbox: bbox(),
            terrain: "flat?h=390&at=6.915,46.425".into(),
            inputs: vec![(5, 4_322_026, 1_757_000_000)],
            built_by: 1_757_000_100,
            built_in_s: 16.5,
            written_at: iso_now(),
        }
    }

    fn scene() -> SceneGraph {
        let c = Corridor {
            id: 0,
            nodes: vec![Coord { x: 6.909, y: 46.437 }, Coord { x: 6.910, y: 46.438 }],
            arc: vec![0.0, 130.5],
            cos_lat: 46.437f64.to_radians().cos(),
            kind: Kind::Road(RoadClass::Tertiary),
            class_key: "tertiary".into(),
            link: false,
            width_m: Some(7.0),
            spans: vec![
                Span { arc0: 0.0, arc1: 60.0, level: 0, kind: SpanKind::Grade },
                Span { arc0: 60.0, arc1: 130.5, level: -1, kind: SpanKind::Tunnel },
            ],
            segments: vec![SegmentRef {
                source: 42,
                node0: 0,
                node1: 1,
                properties: vec![
                    ("class".into(), Value::String("tertiary".into())),
                    ("level".into(), Value::Int(-1)),
                    ("width".into(), Value::Double(7.25)),
                    ("bridge".into(), Value::Bool(true)),
                ],
            }],
            connectors: vec![(7, 0.0), (9, 130.5)],
        };
        // A second corridor with no width, to pin the None/NaN convention.
        let rail = Corridor {
            id: 1,
            nodes: vec![Coord { x: 6.911, y: 46.436 }, Coord { x: 6.912, y: 46.437 }],
            arc: vec![0.0, 140.0],
            cos_lat: 46.436f64.to_radians().cos(),
            kind: Kind::Rail(RailClass::NarrowGauge),
            class_key: "narrow_gauge".into(),
            link: true,
            width_m: None,
            spans: vec![Span { arc0: 0.0, arc1: 140.0, level: 0, kind: SpanKind::Grade }],
            segments: Vec::new(),
            connectors: Vec::new(),
        };
        let mut s = SceneGraph::new(vec![c, rail]);
        s.junctions = vec![Junction {
            point: Coord { x: 6.910, y: 46.438 },
            connector: 9,
            members: vec![JunctionMember { corridor: 0, arc: 130.5 }],
        }];
        s.water = vec![WaterBody {
            exterior: vec![
                Coord { x: 6.90, y: 46.43 },
                Coord { x: 6.91, y: 46.43 },
                Coord { x: 6.91, y: 46.44 },
            ],
            holes: vec![vec![Coord { x: 6.905, y: 46.435 }, Coord { x: 6.906, y: 46.436 }]],
            bbox: (6.90, 46.43, 6.91, 46.44),
        }];
        s.witnesses = vec![vec![Coord { x: 6.9095, y: 46.4372 }]];
        s.flows = vec![vec![Coord { x: 6.9096, y: 46.4373 }, Coord { x: 6.9097, y: 46.4374 }]];
        s
    }

    /// Writes and reads back, through a file of its own.
    ///
    /// `name` rather than one shared path: cargo runs these tests in parallel
    /// in one process, so a shared file has them writing and deleting each
    /// other's snapshot — which reads exactly like a codec that lost a field.
    fn roundtrip(name: &str, snap: &Snapshot) -> Snapshot {
        let dir = std::env::temp_dir().join(format!("arpw-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("tmp dir");
        let path = dir.join(format!("{name}.arpw"));
        write(&path, snap).expect("write");
        let back = read(&path).expect("read");
        let _ = std::fs::remove_file(&path);
        back
    }

    /// The guard on the whole codec: what comes back is what went in, field by
    /// field. A field added to `SceneGraph` and forgotten in the writer shows
    /// up here rather than as a quietly wrong run three stages downstream.
    #[test]
    fn a_scene_survives_the_round_trip() {
        // Built twice rather than cloned: `SceneGraph` is not `Clone`, and
        // deriving it on a production type to please a test is the wrong way
        // round. `scene()` is deterministic, so the two are the same scene.
        let before = scene();
        let back = roundtrip("scene", &Snapshot { provenance: prov(), scene: Some(scene()) });
        let after = back.scene.expect("a scene");

        assert_eq!(after.corridors.len(), before.corridors.len());
        for (a, b) in after.corridors.iter().zip(&before.corridors) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.nodes, b.nodes);
            assert_eq!(a.arc, b.arc);
            assert_eq!(a.cos_lat, b.cos_lat);
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.class_key, b.class_key);
            assert_eq!(a.link, b.link);
            assert_eq!(a.width_m, b.width_m, "None must not come back as 0.0");
            assert_eq!(a.spans, b.spans);
            assert_eq!(a.segments.len(), b.segments.len());
            for (x, y) in a.segments.iter().zip(&b.segments) {
                assert_eq!(x.source, y.source);
                assert_eq!(x.node0, y.node0);
                assert_eq!(x.node1, y.node1);
                assert_eq!(x.properties, y.properties, "every property type round-trips");
            }
            assert_eq!(a.connectors, b.connectors);
        }
        assert_eq!(after.junctions.len(), before.junctions.len());
        assert_eq!(after.junctions[0].connector, before.junctions[0].connector);
        assert_eq!(after.junctions[0].members.len(), before.junctions[0].members.len());
        assert_eq!(after.water.len(), before.water.len());
        assert_eq!(after.water[0].exterior, before.water[0].exterior);
        assert_eq!(after.water[0].holes, before.water[0].holes);
        assert_eq!(after.water[0].bbox, before.water[0].bbox);
        assert_eq!(after.witnesses, before.witnesses);
        assert_eq!(after.flows, before.flows);
        // The source lookup is rebuilt, so it must answer as it did before.
        assert!(after.lookup(42).is_some(), "the source index came back");
    }

    #[test]
    fn provenance_survives_the_round_trip() {
        let p = prov();
        let back = roundtrip("prov", &Snapshot { provenance: p.clone(), scene: None });
        assert_eq!(back.provenance, p);
        assert!(back.scene.is_none(), "not captured is not the same as empty");
    }

    #[test]
    fn a_snapshot_that_matches_is_reusable() {
        let p = prov();
        assert_eq!(p.verdict(&p), Verdict::Reuse);
    }

    /// The three facts that make a snapshot wrong whatever anyone intends.
    #[test]
    fn a_snapshot_refuses_when_anything_it_was_built_from_moved() {
        let p = prov();

        let mut other_bbox = p.clone();
        other_bbox.bbox.east = 7.0;
        assert!(matches!(p.verdict(&other_bbox), Verdict::Refuse(w) if w.contains("bbox")));

        let mut other_ground = p.clone();
        other_ground.terrain = "ramp?grade=0.03".into();
        assert!(matches!(p.verdict(&other_ground), Verdict::Refuse(w) if w.contains("ground")));

        let mut other_input = p.clone();
        other_input.inputs[0].1 += 1;
        assert!(matches!(p.verdict(&other_input), Verdict::Refuse(w) if w.contains("input")));
    }

    /// A rebuild warns rather than refusing: it fires on every rebuild, which
    /// is the loop the flag exists for, and it cannot tell which stage moved.
    /// Silence would be the stale-binary trap; refusal would be useless.
    #[test]
    fn a_rebuild_warns_rather_than_refusing() {
        let p = prov();

        let mut rebuilt = p.clone();
        rebuilt.built_by = p.built_by + 1;
        assert!(matches!(p.verdict(&rebuilt), Verdict::Warn(w) if w.contains("rebuilt")));

        // An *older* binary says nothing: the snapshot was written by code at
        // least as new as what is reading it.
        let mut older = p.clone();
        older.built_by = p.built_by - 1;
        assert_eq!(p.verdict(&older), Verdict::Reuse);
    }

    /// A fact outranks the caveat: a rebuilt binary AND a moved input is a
    /// refusal, not a warning somebody might wave through.
    #[test]
    fn a_refusal_outranks_a_warning() {
        let p = prov();
        let mut both = p.clone();
        both.built_by = p.built_by + 1;
        both.inputs[0].1 += 1;
        assert!(matches!(p.verdict(&both), Verdict::Refuse(_)));
    }

    #[test]
    fn every_kind_round_trips() {
        let kinds = [
            Kind::Road(RoadClass::Motorway),
            Kind::Road(RoadClass::Steps),
            Kind::Road(RoadClass::Other),
            Kind::Rail(RailClass::StandardGauge),
            Kind::Rail(RailClass::Funicular),
            Kind::Rail(RailClass::Unknown),
            Kind::Water(WaterClass::Still),
            Kind::Water(WaterClass::Watercourse),
        ];
        for k in kinds {
            assert_eq!(kind_from_fb(&kind_to_fb(k)), k, "{k:?}");
        }
    }

    #[test]
    fn the_timestamp_reads_as_a_date() {
        let s = iso_now();
        assert_eq!(s.len(), 20, "{s}");
        assert!(s.ends_with('Z'), "{s}");
        assert!(s.starts_with("20"), "{s}");
    }
}
