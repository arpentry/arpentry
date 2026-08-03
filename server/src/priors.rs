//! Engineering priors — everything the map data does not say, as named,
//! class-keyed parameters in one place (docs/GENERATION.md §6).
//!
//! The vector data gives topology (what is above what, roughly where things
//! start and end); the render needs geometry (heights everywhere). The missing
//! numbers — grade ceilings, clearances, deck thickness, structure widths —
//! are engineering conventions, not measurements. Keeping them here makes them
//! tunable, testable, and honest about being priors rather than data.

/// A road's functional class, parsed once from the Overture `class` string.
/// Keys every class-dependent prior below; unknown or missing classes take the
/// `Minor` defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadClass {
    Motorway,
    Trunk,
    Primary,
    Secondary,
    Minor,
}

impl RoadClass {
    pub fn parse(class: Option<&str>) -> RoadClass {
        match class {
            Some("motorway") => RoadClass::Motorway,
            Some("trunk") => RoadClass::Trunk,
            Some("primary") => RoadClass::Primary,
            Some("secondary") => RoadClass::Secondary,
            _ => RoadClass::Minor,
        }
    }

    /// Maximum grade (rise/run) the class is engineered to hold, or `None` to
    /// leave the road on the terrain. Only the engineered high classes are
    /// capped; a residential street or track genuinely follows whatever slope
    /// it is built on (scenario S9: grade limits must not "fix" a road that
    /// genuinely climbs).
    pub fn grade_limit(self) -> Option<f64> {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => Some(0.06),
            _ => None,
        }
    }

    /// Longitudinal grade cap for a street *bed* (the bench the ground stage
    /// cuts for an unclaimed road, D3) — how fast the bench may climb along
    /// the street. Unlike [`RoadClass::grade_limit`] this is not a solver
    /// ceiling: the bed smoothing it feeds is clamped to stay within
    /// [`BED_MAX_DEVIATION_M`] of the natural ground, so a street that
    /// genuinely climbs still climbs (S9) — the cap only irons the sample
    /// noise and terraces a raw DEM drape throws up between bed nodes.
    pub fn bed_grade(self) -> f64 {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => 0.06,
            RoadClass::Primary => 0.08,
            RoadClass::Secondary => 0.10,
            RoadClass::Minor => 0.15,
        }
    }

    /// How far this class's solved profile may leave its conditioned terrain
    /// reference, in metres — the ground-hugging budget
    /// ([`MAX_ROAD_DEVIATION_M`] is the engineered ceiling; a street's bench
    /// irons noise within [`BED_MAX_DEVIATION_M`] and otherwise trusts the
    /// slope, S9). Between them the connecting classes get an intermediate
    /// budget: a primary road is engineered harder than a lane but not
    /// motorway-hard.
    pub fn deviation_m(self) -> f64 {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => MAX_ROAD_DEVIATION_M,
            RoadClass::Primary | RoadClass::Secondary => 4.0,
            RoadClass::Minor => BED_MAX_DEVIATION_M,
        }
    }

    /// Profile node spacing along the corridor, metres. Engineered classes
    /// sample densely (grade relaxation and rim anchoring want resolution);
    /// the long tail of minor streets — most of the network by length —
    /// samples sparsely, bounding the solve's time and memory. Both are finer
    /// than the old street-bed spacing (`BED_SPACING_M`).
    pub fn node_spacing_m(self) -> f64 {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => NODE_SPACING_M,
            RoadClass::Primary | RoadClass::Secondary => 12.0,
            RoadClass::Minor => 24.0,
        }
    }

    /// Half-width in metres of a swept structure box — bigger roads, bigger
    /// structures. Overture maps each carriageway of a dual carriageway (and
    /// each ramp) as its own segment, so these are *per-carriageway* widths,
    /// not whole-road widths: dual motorway centerlines run only ~8–15 m
    /// apart, and a whole-motorway width on each would overlap the decks. The
    /// values follow the mapped `width` medians in the Swiss extract
    /// (motorway 9 m, trunk 8 m, primary 7 m, secondary 6 m, minor 5–6 m).
    /// A `link` (ramp) is a single lane plus shoulders whatever its class
    /// (mapped medians 4.5–5.5 m).
    pub fn half_width_m(self, link: bool) -> f64 {
        if link {
            return 2.75;
        }
        match self {
            RoadClass::Motorway => 4.5,
            RoadClass::Trunk => 4.0,
            RoadClass::Primary => 3.5,
            RoadClass::Secondary => 3.0,
            RoadClass::Minor => 2.75,
        }
    }
}

/// Whether an Overture `subclass` marks a ramp — narrower than its class's
/// mainline carriageway, whatever that class.
pub fn is_link(subclass: Option<&str>) -> bool {
    subclass == Some("link")
}

/// Painted width in metres of the small service ways — driveways, parking
/// aisles, alleys: a single car's track plus margins, well under the minor
/// street their `service` class would otherwise inherit (Swiss-extract
/// mapped medians run ~3 m).
pub const SERVICE_WAY_WIDTH_M: f64 = 3.0;

/// Physical painted width in metres of a drivable road, keyed by its Overture
/// class/subclass — twice the [`RoadClass::half_width_m`] the structure sweep
/// uses, so the paint stroke and the deck it rides are sized from the same
/// prior and meet edge-to-edge. `None` for non-drivable classes (paths, rail,
/// tracks), which keep their cartographic stroke widths.
pub fn paint_width_m(class: Option<&str>, subclass: Option<&str>) -> Option<f64> {
    let c = class?;
    let drivable = matches!(
        c,
        "motorway"
            | "trunk"
            | "primary"
            | "secondary"
            | "tertiary"
            | "unclassified"
            | "residential"
            | "living_street"
            | "service"
            | "unknown"
    );
    if !drivable {
        return None;
    }
    // The small service ways are narrower than any street class.
    if matches!(subclass, Some("driveway" | "parking_aisle" | "alley")) {
        return Some(SERVICE_WAY_WIDTH_M);
    }
    Some(2.0 * RoadClass::parse(Some(c)).half_width_m(is_link(subclass)))
}

/// How far a mapped `width` may stray from the class prior, as factors of it,
/// before it is distrusted. Mapped widths are rare (0.6–10 % per class on the
/// Swiss extract) but where present they are usually right — the medians
/// match the priors. Beyond these bounds the measurement contradicts the
/// class (a whole right-of-way width on a footpath-sized lane, a typo'd
/// unit), and the prior is kept — the same trust-the-prior resolution the
/// clearance caps above use.
pub const MEASURED_WIDTH_FACTOR_MIN: f64 = 0.35;
pub const MEASURED_WIDTH_FACTOR_MAX: f64 = 3.0;

/// Painted carriageway width in metres (docs/ROADS.md H2): the mapped
/// Overture `width_rules` value when plausible against the class prior, else
/// the prior itself ([`paint_width_m`]). `None` for non-drivable classes even
/// when a width is mapped — their stroke stays cartographic until they grow
/// surfaces of their own (docs/ROADS.md P5).
pub fn carriageway_width_m(
    class: Option<&str>,
    subclass: Option<&str>,
    measured_m: Option<f64>,
) -> Option<f64> {
    let prior = paint_width_m(class, subclass)?;
    match measured_m {
        Some(w)
            if w >= prior * MEASURED_WIDTH_FACTOR_MIN
                && w <= prior * MEASURED_WIDTH_FACTOR_MAX =>
        {
            Some(w)
        }
        _ => Some(prior),
    }
}

/// Vertical clearance a bridge deck's *underside* must keep over a crossed
/// feature (scenarios S3/S4, invariant 3). About 5 m over a road, more over
/// rail (catenary), freeboard over water. The data never states built
/// clearances; these are the engineering minimums.
pub fn clearance_m(lower: crate::scene::CrossedKind) -> f64 {
    match lower {
        crate::scene::CrossedKind::Road => 5.0,
        crate::scene::CrossedKind::Rail => 7.0,
        crate::scene::CrossedKind::Water => 4.0,
    }
}

/// How far either side of a crossing the clearance solver looks for the trough
/// an *unprofiled* crossed feature runs in. A road passing under a bridge runs
/// along the cut made for it, so the lowest ground within a short reach of the
/// intersection is a better stand-in for its surface than the point sample,
/// which beside an abutment reads the trench wall. Short enough that a
/// genuinely open crossing still reads its own flat ground.
pub const CLEARANCE_TROUGH_M: f64 = 20.0;

/// First zoom that carries the road-surface band under the paint
/// (docs/ROADS.md P2), and with it the junction plates the band runs into —
/// the close-up detail rung of the degradation ladder (D5). Coarser zooms
/// render the bare draped strokes; positions never change, only detail sheds.
pub const ROAD_SURFACE_MIN_ZOOM: u8 = 13;

/// Zoom whose tile rects are the chunks the unioned road surface is baked in
/// (docs/ROADS.md §6.1). Every zoom that draws asphalt is at or beyond
/// [`ROAD_SURFACE_MIN_ZOOM`] and the tile grid nests, so every such tile lies
/// wholly inside exactly one chunk: a tile clip never spans a chunk edge, and
/// one region boundary serves every zoom.
pub const PAVE_BAKE_Z: u8 = ROAD_SURFACE_MIN_ZOOM;

/// How far outside its own rect a chunk's union reads input, in metres. A union
/// boundary is local — only geometry within its own half-width plus the closing
/// reach can move it — so this need only cover the widest carriageway plus
/// [`CURB_RETURN_M`] twice over, not a whole tile.
pub const PAVE_PAD_M: f64 = 32.0;

/// Curb-return radius, in metres: the morphological closing that rounds the
/// reflex corners where carriageways meet (docs/ROADS.md §6.5, H3). Applied only
/// inside intersection extents — a closing this wide would otherwise bridge any
/// gap under 6 m and fuse a divided carriageway into one slab.
pub const CURB_RETURN_M: f64 = 3.0;

/// Width of the antialiasing rim inset from the paved boundary, in metres. The
/// strip that carries `edge_across` from the silhouette (127) to the interior
/// (0), and with it the darker casing tone. Wide enough to hold the ~1 px fade
/// at a grazing angle, narrow enough to read as a kerb line rather than a band.
pub const PAVE_RIM_M: f64 = 0.35;

/// Coarsest the paved boundary may be simplified, in metres — a cap *on top of*
/// the tiler's per-zoom line tolerance.
///
/// A road's width is the subject here, not incidental detail: at z13 the generic
/// budget is ~1.2 m, which on a 6 m carriageway is a fifth of its width, and the
/// boundary visibly deforms into angular blobs. A cartographic line can absorb
/// that because only its path matters; a surface cannot. A fifth of a metre is
/// far below the narrowest carriageway yet still four orders coarser than the
/// sub-millimetre precision the union is built at, so it keeps almost all of the
/// vertex reduction.
pub const PAVE_SIMPLIFY_M: f64 = 0.2;

/// Longest paved-boundary edge before the surface mesher subdivides it, in
/// metres, so a long straight kerb still gets vertices to drape on.
pub const PAVE_SEG_M: f64 = 4.0;

/// Longest bed earthwork edge for an unclaimed street, in metres: edges
/// longer than this are subdivided so the bed's targets track the terrain
/// along the road at this resolution.
pub const BED_SPACING_M: f64 = 30.0;

/// How far a smoothed bed target may leave the natural ground at its own
/// centerline node, in metres. The budget that keeps [`RoadClass::bed_grade`]
/// honest: within it the bench irons DEM noise flat, beyond it the terrain is
/// trusted and the bench follows the slope (S9).
pub const BED_MAX_DEVIATION_M: f64 = 2.5;

/// Target spacing in metres after corridor densification for the engineered
/// classes ([`RoadClass::node_spacing_m`]), used both to sample the road
/// profile along the corridor and to subdivide swept geometry so it renders
/// as a smooth curve.
pub const NODE_SPACING_M: f64 = 8.0;

/// Widest DEM notch, in metres of road arc, that a mapped at-grade road is
/// assumed to span on engineered fill (a culvert, an embankment, a small
/// retaining structure) rather than dive through. Gullies, stream cuts, and
/// shadow artifacts under real roads image as narrow V's in a surface DEM;
/// the road existed first — ground continuity across it was engineered.
/// Wider valleys are genuine descents and keep the terrain.
pub const NOTCH_SPAN_M: f64 = 60.0;

/// Deepest per-notch fill the closing may build, in metres. A notch deeper
/// than this under an at-grade road is a data contradiction (a gorge owed a
/// mapped bridge, or a DEM blunder): the terrain is trusted and the road
/// keeps its raw profile there — the same trust cap the clearance solver
/// applies ([`MAX_CLEARANCE_LIFT_M`]).
pub const NOTCH_FILL_MAX_M: f64 = 15.0;

/// Widest convex terrain bump, in metres of road arc, shaved from a road's
/// anchor surface as noise rather than climbed. A surface DEM images canopy
/// shadows, parked vehicles, and upsampling ripple as narrow crests *on* the
/// carriageway; a real road was graded through them. Wider rises are genuine
/// relief and keep the terrain. The opening dual of [`NOTCH_SPAN_M`], sized
/// under it: filling across an engineered culvert is cheaper to assume than
/// cutting through an unmapped crest.
pub const BUMP_SPAN_M: f64 = 50.0;

/// Deepest per-bump shave the opening may take, in metres. A crest that
/// would need a deeper cut is a genuine hill (S9): the terrain is trusted
/// and the road keeps its raw profile there — the mirror of
/// [`NOTCH_FILL_MAX_M`], far tighter because false crests (noise) are
/// shallow while false notches (gorges) can be deep.
pub const BUMP_SHAVE_MAX_M: f64 = 4.0;

/// Largest reconciliation applied where street beds share an endpoint
/// connector (or meet a solved corridor), in metres. Within it the meeting
/// beds are welded to one height so no step crosses the junction; a larger
/// disagreement is a data contradiction and the beds are left apart rather
/// than dragged.
pub const BED_WELD_MAX_M: f64 = 3.0;

/// First zoom that paints longitudinal road markings (docs/ROADS.md P3).
/// Deeper than the surface band's zoom: a 12 cm line is sub-pixel until the
/// camera is close.
pub const MARKING_MIN_ZOOM: u8 = 15;

/// Painted line widths in metres (Swiss norms run 0.10–0.15 m) and how far
/// the edge line's centre sits in from the carriageway edge.
pub const CENTRE_LINE_WIDTH_M: f64 = 0.12;
pub const EDGE_LINE_WIDTH_M: f64 = 0.15;
pub const EDGE_LINE_INSET_M: f64 = 0.30;

/// The marking ladder (docs/ROADS.md §6.5): which longitudinal lines a class
/// paints. The data never says (marking style is almost never mapped), so
/// the ladder is a prior: engineered classes paint a centre line between
/// opposing flows and edge lines at the carriageway edge; a quiet street
/// paints nothing. A one-way carriageway (each half of a dual carriageway,
/// every ramp) has no opposing flow and therefore no centre line.
pub fn has_centre_line(class: &str, oneway: bool) -> bool {
    !oneway && matches!(class, "primary" | "secondary" | "tertiary")
}

pub fn has_edge_lines(class: &str) -> bool {
    // Edge lines only on the motorway network: on lower classes they crowd
    // the centre line into clutter at street widths, and in-town carriageway
    // edges are curbs, not painted lines.
    matches!(class, "motorway" | "trunk")
}

/// Whether a carriageway paints dashed dividers between its same-direction
/// lanes: the engineered one-way carriageways. Motorways and trunks count as
/// one-way even untagged — each carriageway is one by construction — so a
/// data gap (2 % of motorways) does not cost them their lane lines.
pub fn has_lane_lines(class: &str, oneway: bool) -> bool {
    matches!(class, "motorway" | "trunk")
        || (oneway && matches!(class, "primary" | "secondary"))
}

/// Nominal lane width for splitting a carriageway into same-direction lanes.
/// The count is inferred *back* from the carriageway width (docs/ROADS.md
/// H2) — no source states it: `round(width / lane_width)`, floored at one.
pub fn lane_width_m(class: &str) -> f64 {
    match class {
        "motorway" | "trunk" => 3.75,
        "primary" | "secondary" => 3.25,
        _ => 3.0,
    }
}

/// The inferred same-direction lane count of a one-way carriageway.
pub fn lane_count(class: &str, width_m: f64) -> u32 {
    ((width_m / lane_width_m(class)).round() as u32).max(1)
}

/// Approach-ramp grade for a road class with no engineered ceiling: how fast
/// an overpass approach may climb onto its embankment. Engineered classes use
/// their own [`RoadClass::grade_limit`] instead.
pub const RAMP_GRADE: f64 = 0.08;

/// Smallest cut/fill (|solved road − natural terrain|, metres) that earns an
/// earthwork: below this the road drapes on the natural ground and no ground
/// modifier is emitted.
pub const MIN_EARTHWORK_M: f64 = 0.3;

/// How far an earthwork's batter face reaches beyond the bench per metre of
/// cut or fill depth (a 1:2.5 slope). The face is straight and self-limiting:
/// it stops where it meets the natural ground, so this is a bound on the
/// reach, not the reach itself.
pub const EARTHWORK_BATTER: f64 = 2.5;

/// Metres of run per metre of rise on a *wall* face — the shape the earthwork
/// falls back to where an earth batter cannot close.
///
/// A 4:1 face. Steep enough to bridge a stacked pair's separation inside the few
/// metres a switchback leaves between its arms, which an earth slope at 1 in
/// [`EARTHWORK_BATTER`] cannot: thirteen metres of step needs thirty-two metres
/// of batter and has seven. Not vertical, because it is still meshed as ground
/// and a plan-degenerate face has no area to draw; `slope.terrain_face` counts
/// anything past 2:1 as a manufactured wall, and this is meant to be counted —
/// it *is* one, deliberately, where the alternative is no earthwork at all.
pub const WALL_BATTER: f64 = 0.25;

/// The shortest reach a *carve* notch's wall is given — a floor, because a
/// notch cut with no depth to speak of still needs a wall to stand in.
///
/// Road benches have no such floor: see [`crate::ground::batter_reach`] for why
/// one cannot exist there. A floor holds the ground down past the point the
/// face daylights, and the field then steps back to the hillside at the floor's
/// outer edge — out in open ground where no contact line runs, which the detail
/// mesh draws as sawtooth.
pub const EARTHWORK_MIN_BATTER_M: f64 = 2.0;

/// The tallest face, in metres, that holding a bench flat across a cross-slope
/// may cut or fill at the bench edge before the bench is abandoned and the
/// road simply drapes.
///
/// A bench is a terrace, and a terrace on a near-vertical flank is a fiction:
/// on the wall of a gorge, holding a footpath's eight-metre band flat means a
/// twenty-metre rock cut on one side and the same in fill on the other, which
/// is neither what is there nor something a mesh can draw without tearing into
/// sawtooth along the crest. Above the cap the road is left on the natural
/// ground, tilted as the hillside is — which for a trail cut into a cliff is
/// also the truth.
///
/// **The face is two things added together, and this is deliberate.**
/// `road − edge = (road − terrain) + (terrain − edge)`: the road's own
/// departure from the ground, plus the hillside's fall across the band. The
/// wording above describes only the second, so the cap reads like a conflation
/// bug. It is not — it is a crude guard, and both ways of "fixing" it are
/// worse. Measured on the Montreux extract, against
/// `server/verify/baseline-montreux-z16.json`:
///
/// - **Cap the cross-slope term only.** Every tall fill then benches, and a
///   fill whose batter cannot daylight becomes a retaining wall: the steepest
///   terrain face went 264:1 → 542:1, the worst burial 4.2 m → 7.6 m, and the
///   worst deck-into-hillside 6.9 m → 12.5 m. Six metrics regressed, and the
///   worst float did not improve at all.
/// - **Ask instead whether the batter daylights** (`ground::batter_reach`'s own
///   test), deleting this prior. Too strict by far: a 1:2 cross-slope closes
///   nowhere against a 1:2.5 batter, so an ordinary hillside street loses its
///   bench. The unit tests reject it before a retile does.
///
/// So the cap stays, and what it costs is *known and instrumented* rather than
/// invisible: refusing the bench does not put the road back on the ground, it
/// leaves the road where the profile put it and the ground where the DEM had
/// it, with air between. That is 3.8 % of all asphalt standing more than a
/// metre clear of the drawn ground, reaching 15 m — `contact.pavement_floating`.
///
/// The dilemma is structural, not a tuning problem: on steep or tall ground the
/// earthwork must choose between a wall, a float, and no bench, and all three
/// are visible *only because the ground is drawn under the asphalt at all*.
/// Cutting the terrain back to the kerb dissolves the choice rather than
/// balancing it (`data/plans/terrain-hole-plan.md`, docs/VERIFICATION.md §6).
pub const MAX_BENCH_FACE_M: f64 = 3.0;

/// How much further than its flat-ground daylight distance a batter face may
/// reach before the ground counts as running away with it and the face is
/// abandoned for a retaining wall. Above 1 so a gently falling flank still
/// gets its batter; low enough that a face nearly parallel to the hillside —
/// which would cut or fill the whole way out — never gets built.
pub const BATTER_DIVERGENCE_SLOP: f64 = 2.5;

/// Outer bound on that reach, in metres. A face against near-flat ground
/// daylights on its own; this only stops a deep fill (or the near-parallel
/// case, where the ground runs away at almost exactly the batter) from
/// claiming an unbounded footprint.
pub const EARTHWORK_MAX_BATTER_M: f64 = 40.0;

/// Extra width beyond the structure half-width that an earthwork holds at
/// road height (the shoulder) before the slope starts.
pub const EARTHWORK_SHOULDER_M: f64 = 1.0;

/// Flat margin beyond the shoulder that a road earthwork keeps at road
/// height before the batter starts — the verge between the asphalt edge and
/// the top of the batter.
///
/// It used to be a rendering allowance of about one detail-lattice cell, so
/// that a mesh corner just outside the bench could not interpolate natural
/// hillside up across the band edge. The crest contact lines
/// ([`crate::ground::breaklines`]) now hold the bench edge exactly at the
/// detail zoom, so the allowance is no longer needed and the bench is kept
/// narrow: a wide flat bench is a wide terrace cut into (or filled out from)
/// every hillside the road crosses, which is both more ground disturbed than
/// the road needs and a deeper face where it ends. Carve notches (portal
/// cuts, deck daylighting) stay at the engineering width — a wider notch
/// would eat the abutment it daylights.
pub const EARTHWORK_MARGIN_M: f64 = 0.5;

/// Thickness of a bridge deck slab in metres — deck surface to its underside.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// Deck half-width in metres of a non-drivable structure — a footbridge,
/// cycleway or pedestrian bridge. Pedestrian-scale: a narrow slab with no
/// vehicle shoulder, unlike the carriageway-plus-[`STRUCTURE_SHOULDER_M`] a
/// drivable deck carries. Without it a `footway` falls into the `Minor`
/// car-lane [`RoadClass::half_width_m`] and bakes a ~7.5 m deck for a footpath.
pub const PATH_STRUCTURE_HALF_WIDTH_M: f64 = 1.25;

/// Widest lateral gap, in metres, between a non-drivable structure (a
/// footbridge, cycleway) and a parallel drivable bridge for the two to resolve
/// as one physical structure (scenario S8, entity resolution across parallel
/// segments). Overture maps a road bridge and its separated footpath as two
/// independently `bridge`-tagged ways; within this gap they are bound to one
/// shared grade line so their decks ride at one height instead of overlapping
/// at two. A wider gap is a genuinely separate structure and keeps its own
/// solved profile — the same trust-the-data resolution the clearance caps use.
pub const PARALLEL_STRUCTURE_LATERAL_M: f64 = 12.0;

/// Extra half-width in metres a structure (bridge deck, tunnel bore) carries
/// beyond the painted carriageway — the edge beam / shoulder / barrier a real
/// deck adds outside the traffic lanes. Applied to the whole structure sweep
/// (deck and bore alike, so a bridge↔tunnel junction has no width step), while
/// the paint keeps the carriageway width — so the deck-top asphalt frames the
/// road ribbon as a visible shoulder instead of ending flush with it.
pub const STRUCTURE_SHOULDER_M: f64 = 1.0;

/// Vertical clearance of a tunnel bore in metres — road floor to its flat
/// roof. Road tunnels are built to a ~4.5 m vehicle clearance plus equipment
/// headroom.
pub const TUNNEL_HEIGHT_M: f64 = 5.0;

/// Ground cover an underpass keeps between its bore roof and the surface the
/// crossed feature rides on (scenario S6): enough that the crossing feature
/// has a roadbed, not so much that a shallow urban underpass digs a cavern.
pub const TUNNEL_COVER_M: f64 = 0.5;

/// How far in front of a portal mouth the ground is carved down to the bore
/// floor, so the mouth face is daylighted instead of its lower part hiding
/// below grade.
pub const PORTAL_CUT_LEN_M: f64 = 12.0;

/// Deepest an underpass constraint may press the road below its solved
/// profile. A real depressed underpass runs ~7–12 m below grade (bore, cover,
/// slab — sometimes stacked); a demand far beyond that means the level tags
/// and the solved geometry contradict each other (e.g. a mapper-annotated
/// mountain tunnel whose profile stands high over the crossing road), and
/// honouring it would drag the profile — and the earthworks that chase it —
/// hundreds of metres down. Such demands are dropped: the profile is trusted
/// over the tag.
pub const MAX_UNDERPASS_SINK_M: f64 = 15.0;

/// Highest a clearance constraint may lift the road above its solved
/// profile — the raising twin of [`MAX_UNDERPASS_SINK_M`]. A real overpass
/// clears its crossed road by ~6.5–10 m (clearance plus slab, some grade),
/// ~13 m when it stacks over an already-lifted deck; a demand far beyond
/// that means the crossing geometry and the solved profile contradict each
/// other (e.g. a path mapped across a viaduct's plan line high on a flank),
/// and honouring it once flattened kilometres of viaduct at the highest
/// demand — a deck 200 m over Montreux. Such demands are dropped: the
/// profile is trusted over the inferred constraint.
pub const MAX_CLEARANCE_LIFT_M: f64 = 15.0;

/// Longest annotated structure span treated as one rigid box whose constraint
/// holds end to end: a short bridge is lifted as one deck (S4), a short
/// tunnel runs depressed as one cut-and-cover bore (S6, the urban underpass).
/// A longer span is a viaduct or driven tunnel; only the crossing feature's
/// own width lifts or dips, and the road returns to its own grade at the ramp
/// grade — one crossing must not drag kilometres of structure to its height.
pub const STRUCTURE_BOX_MAX_M: f64 = 300.0;

/// How far, in metres, an engineered road may sit above (fill) or below (cut)
/// the draped terrain. The grade ceiling alone, held across a long mountain
/// climb, drifts the road tens of metres from the ground — a phantom viaduct or
/// a road buried deep under a hill that should be a tunnel. Bounding the
/// deviation keeps the road hugging the ground. Sized so the residual cut is
/// shallow enough for the client's road depth-bias to surface
/// (`client/shaders/road.wgsl` `ROAD_DEPTH_MARGIN_M`).
pub const MAX_ROAD_DEVIATION_M: f64 = 8.0;

/// Shortest bridge/tunnel span that is *unconditionally* a structure. A
/// shorter span faces a terrain test in the solve stage
/// (`solve::reconcile_short_spans`): it stays a structure only where the
/// ground genuinely falls away (rises) under it by more than
/// [`SHORT_STRUCTURE_DIP_M`] — a 25 m bridge over a deep stream gully keeps
/// its deck instead of demoting to grade and diving through the cut, while a
/// footbridge annotation on near-flat ground still drapes (baking a deck on
/// it only leaves a tiny box floating over the hill).
pub const MIN_STRUCTURE_M: f64 = 40.0;

/// Smallest mid-span departure of the terrain from the span's end-to-end
/// chord (metres) that makes a sub-[`MIN_STRUCTURE_M`] span a real structure:
/// below it, at-grade draping (and the notch closing) carries the road just
/// as well, and the tiny deck would read as a floating box. Sized at the gap
/// under a deck end that still reads as "landed".
pub const SHORT_STRUCTURE_DIP_M: f64 = 3.0;

/// A grade sliver shorter than this (metres), sandwiched between two structure
/// spans, is treated as an annotation-edge mismatch rather than real at-grade
/// road, and dropped so the structures abut (scenario S10). Genuine at-grade
/// stretches between structures are far longer.
pub const SNAP_RUN_M: f64 = 10.0;

/// Step length when marching a buried portal end outward to find the
/// road/terrain crossing where a bore emerges.
pub const PORTAL_MARCH_M: f64 = 3.0;

/// Furthest a portal is marched outward before giving up and capping there. A
/// runaway guard for a road whose approach stays buried (e.g. a coarse DEM
/// that never dips below the road grade); real portals emerge well within this.
pub const PORTAL_MAX_M: f64 = 200.0;

/// Emergence clearance: a portal cap is placed this far *past* the crossing so
/// the mouth sits just clear of the terrain rather than flush with it.
pub const PORTAL_CLEARANCE_M: f64 = 1.0;

/// Smallest still water body worth flattening, as its bounding box's larger
/// dimension in metres. A pond below this is finer than the DEM resolves and
/// costs a shoreline sampling pass for no visible gain; it stays draped. Sized
/// like [`MIN_STRUCTURE_M`] — the scale below which a feature is not worth
/// synthesising.
pub const MIN_WATER_BODY_M: f64 = 40.0;

/// Percentile of the shoreline-sampled DEM taken as a still water body's
/// surface level (scenario S14, invariant 4). The exterior ring traces the
/// waterline, so its ground images the water level; a low percentile leans
/// toward the water rather than a ring vertex that climbed the bank, so the
/// flattened surface settles in its basin instead of spilling over the shore.
pub const WATER_LEVEL_PCTL: f64 = 0.3;

/// Highest a junction weld may lift a corridor's leg to meet the road it joins
/// (invariant 2): a ramp diverging from an elevated flyover is pulled up to the
/// deck it leaves. The operative plausibility test is the leg's own climbing
/// capacity (its length times its ramp grade — a leg cannot meet a deck it
/// cannot climb to); this constant is the absolute ceiling above it, sized to
/// the tallest single-level interchange ramp. A demand beyond either means the
/// shared connector links roads that do not in fact meet at one height (a
/// mapping error, or a leg that climbs to its own structure elsewhere); the
/// weld is dropped and the leg keeps its solved profile — the same "trust the
/// profile over the inferred constraint" the clearance caps use.
pub const MAX_JUNCTION_WELD_M: f64 = 25.0;

/// Longest chain of segments joined into one corridor, in metres. Corridors
/// longer than this are split; junction-continuity constraints (solve stage)
/// carry coherence across the cut. Bounds the profile arrays and keeps a
/// mis-joined ring road from swallowing a region.
pub const MAX_CORRIDOR_M: f64 = 30_000.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_engineered_classes_are_grade_limited() {
        assert!(RoadClass::parse(Some("motorway")).grade_limit().is_some());
        assert!(RoadClass::parse(Some("trunk")).grade_limit().is_some());
        assert!(RoadClass::parse(Some("residential")).grade_limit().is_none());
        assert!(RoadClass::parse(None).grade_limit().is_none());
    }

    #[test]
    fn half_width_scales_with_class() {
        assert!(RoadClass::Motorway.half_width_m(false) > RoadClass::Minor.half_width_m(false));
        assert_eq!(
            RoadClass::parse(None).half_width_m(false),
            RoadClass::Minor.half_width_m(false)
        );
    }

    #[test]
    fn links_are_narrow_whatever_the_class() {
        assert!(RoadClass::Motorway.half_width_m(true) < RoadClass::Motorway.half_width_m(false));
        assert!(is_link(Some("link")));
        assert!(!is_link(Some("sidewalk")));
        assert!(!is_link(None));
    }

    #[test]
    fn small_service_ways_are_narrow() {
        assert_eq!(paint_width_m(Some("service"), Some("driveway")), Some(SERVICE_WAY_WIDTH_M));
        assert_eq!(
            paint_width_m(Some("service"), Some("parking_aisle")),
            Some(SERVICE_WAY_WIDTH_M)
        );
        // A plain service road keeps the minor-street width.
        assert_eq!(paint_width_m(Some("service"), None), Some(5.5));
    }

    #[test]
    fn measured_width_wins_when_plausible() {
        // A mapped 6.2 m residential street beats the 5.5 m prior.
        assert_eq!(carriageway_width_m(Some("residential"), None, Some(6.2)), Some(6.2));
        // A narrow mapped lane is still plausible (0.35 × 5.5 = 1.9 m).
        assert_eq!(carriageway_width_m(Some("residential"), None, Some(2.0)), Some(2.0));
        // No measurement → the prior.
        assert_eq!(carriageway_width_m(Some("residential"), None, None), Some(5.5));
    }

    #[test]
    fn implausible_width_falls_back_to_the_prior() {
        // A 30 m "residential street" is a right-of-way width, not a lane.
        assert_eq!(carriageway_width_m(Some("residential"), None, Some(30.0)), Some(5.5));
        // A 1 m one contradicts drivability.
        assert_eq!(carriageway_width_m(Some("residential"), None, Some(1.0)), Some(5.5));
        // Non-drivable classes stay cartographic even with a mapped width.
        assert_eq!(carriageway_width_m(Some("footway"), None, Some(1.5)), None);
    }

    #[test]
    fn dual_carriageway_decks_do_not_overlap() {
        // Overture dual-carriageway centerlines run ~8-15 m apart (measured on
        // the Swiss extract, p10 = 8.2 m); two swept motorway boxes must fit.
        assert!(2.0 * RoadClass::Motorway.half_width_m(false) <= 9.0);
    }
}
