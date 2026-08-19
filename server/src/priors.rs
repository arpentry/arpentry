//! Engineering priors — everything the map data does not say, as named,
//! class-keyed parameters in one place (docs/GENERATION.md §9).
//!
//! The vector data gives topology (what is above what, roughly where things
//! start and end); the render needs geometry (heights everywhere). The missing
//! numbers — grade ceilings, clearances, deck thickness, structure widths —
//! are engineering conventions, not measurements. Keeping them here makes them
//! tunable, testable, and honest about being priors rather than data.

/// The transport modality — the first half of the §9 prior key.
///
/// Modality alone gets two cases wrong in opposite directions: a tram lies on
/// a street while a funicular has its own formation and a single gradient. So
/// it is never a key on its own; it names the *class vocabulary* that follows
/// it, and the pair is the key.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Modality {
    Road,
    Rail,
    Water,
}

/// A road class, in Overture's own vocabulary. The drivable classes, then the
/// draped ones — nearly half the network by segment count (§2.2), which is why
/// they are named individually rather than falling into a default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadClass {
    Motorway,
    Trunk,
    Primary,
    Secondary,
    Tertiary,
    Unclassified,
    Residential,
    LivingStreet,
    Service,
    /// Overture's `unknown` road class: mapped as a road, class not stated,
    /// and drivable — distinct from [`RoadClass::Other`], which is a class
    /// string this table does not name.
    Unknown,
    Track,
    Footway,
    Pedestrian,
    Path,
    Steps,
    Cycleway,
    Bridleway,
    /// A class string this table does not name. Takes the most junior
    /// plausible stratum and lays no asphalt (§4.6): a class we cannot read
    /// must not be granted authority or a carriageway on the strength of a
    /// default.
    Other,
}

/// A rail class: the gauge, or the system where the gauge is not the point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RailClass {
    StandardGauge,
    NarrowGauge,
    BroadGauge,
    Funicular,
    Subway,
    LightRail,
    Monorail,
    Tram,
    /// Overture's `unknown` rail class — 5,194 segments on the Swiss extract,
    /// which is why it is a variant with a stated authority rather than a
    /// silent default.
    Unknown,
}

/// A water class. Still bodies are flat; watercourses descend along flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WaterClass {
    Still,
    Watercourse,
}

/// The §9 prior key: `(modality, class)`, as one value.
///
/// Nested rather than a flat enum or a `(Modality, &str)` pair so the match is
/// exhaustive. The flat five-bucket `RoadClass` this replaced ended in
/// `_ => Minor`, which is how a mainline railway and a footpath came to share
/// a residential street's grade, deviation and half-width.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Road(RoadClass),
    Rail(RailClass),
    Water(WaterClass),
}

/// The shape of a class's longitudinal constraint (§9).
///
/// A shape, not a number, because a funicular's constraint is *constant grade*
/// and a ceiling cannot express it: a parameter that pretends otherwise is a
/// lie the solver will act on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GradeShape {
    /// `|dh/ds| <= g`. Every drivable road; what differs between a motorway
    /// and a lane is the deviation budget, not the shape.
    Bounded(f64),
    /// A ceiling plus a vertical-curve radius: a surveyed railway.
    CurvatureLimited { grade: f64, radius_m: f64 },
    /// No profile at all: the feature samples the finished ground.
    Draped,
}

impl GradeShape {
    /// The grade this shape holds along the alignment, for the callers that
    /// need a single number (the relaxation's per-edge ceiling). `None` for a
    /// draped class, which holds no grade of its own.
    pub fn grade(self) -> Option<f64> {
        match self {
            GradeShape::Bounded(g) => Some(g),
            GradeShape::CurvatureLimited { grade, .. } => Some(grade),
            GradeShape::Draped => None,
        }
    }
}

/// Which stratum a class belongs to (§4.2) — its authority, made mechanical.
///
/// `Ord` ascending is authority order, so "solve in authority order" is a sort
/// and "is this senior to that" is a comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stratum {
    /// Hydrology. Absolute authority; publishes the water datum and freeboard.
    H,
    /// Independent rail: a formation that exists whatever the streets do.
    R,
    /// The street network — the negotiating layer.
    S,
    /// Draped features. No authority, and no solve: they sample the finished
    /// ground. Carrying a structure span is *not* a promotion (§4.2).
    D,
    /// Buildings, founded on the finished ground.
    B,
}

/// Everything the data does not say about one `(modality, class)`, in one
/// place (§9).
#[derive(Debug, Clone, Copy)]
pub struct Prior {
    /// The shape of the longitudinal constraint.
    pub grade_shape: GradeShape,
    /// Whether the class holds a *surveyed* alignment. Gates the engineered-only
    /// solve behaviours — rim anchoring, infeasible-anchor absorption into
    /// structures (docs/GROUND.md §1) — which a street must not get: a street's
    /// grade is a bed grade that irons DEM noise, never a standard it was built
    /// to, so treating it as one digs corridors into hillsides.
    pub engineered: bool,
    /// How far the profile may leave its conditioned terrain reference, metres.
    /// This is what lets a residential street follow the hill while a motorway
    /// cuts through it — same grade shape, different budget.
    pub deviation_m: f64,
    /// Profile node spacing along the alignment, metres.
    pub node_spacing_m: f64,
    /// What a feature crossing *over* this must leave above it, metres.
    pub clearance_over_m: f64,
    /// What this must leave when it crosses over something, metres.
    pub clearance_under_m: f64,
    /// Shortest plausible real structure of this class, metres.
    pub min_structure_m: f64,
    /// Half the physical cross-section, metres. `None` for a class with no
    /// width of its own — a footpath's stroke is cartographic.
    pub half_width_m: Option<f64>,
    /// The surface band this class draws — the material of its carriageway in
    /// the paved union. Distinct from having a width, because a draped class
    /// has earthworks as wide as a lane and no surface of its own to draw.
    pub surface: Surface,
    /// Whether this class's profile is **monotone** along its alignment: one
    /// cable, one hill — heights never reverse between the two ends. True for
    /// a funicular and nothing else; a mainline railway undulates and a road
    /// does whatever its hill does. A Required-level constraint, not a
    /// preference: a funicular with a dip is not a steep railway, it is a
    /// physical impossibility, and every defect that puts one there (a bore
    /// ceiling diving at a data-gap end, a fragment seam stepping, a junction
    /// kink) is refuted by this single invariant rather than patched at its
    /// own site.
    pub monotone: bool,
    /// Which stratum this class is solved in.
    pub stratum: Stratum,
}

/// What a class's drawn surface is made of. This decides *whether* a class
/// enters the unioned surface (`Surface::None` keeps a cartographic stroke and
/// nothing else) and *which region* it lands in — ballast and asphalt are
/// separate regions with separate materials, never one merged slab. What stays
/// keyed on `Asphalt` alone: markings, junction plates, casing paint — the
/// road-furniture the ladder paints on asphalt and nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Surface {
    /// A carriageway: the road classes that pave.
    Asphalt,
    /// A rail formation: the track bed of independent rail (§4.2 stratum R).
    /// It gets the same treatment a carriageway gets — a benched band, a
    /// surface mesh, a hole in the drawn ground, an apron where the model
    /// implies a wall — because every drawn-world mechanism that makes a road
    /// robust to a wrong height was missing for rail, and rail paid the whole
    /// price in daylight.
    Ballast,
    /// No surface band: paths, tracks, street-running rail, water. The stroke
    /// is cartographic and the ground is the feature.
    None,
}

/// Half-width in metres of a ramp, whatever its class: a single lane plus
/// shoulders (mapped medians 4.5–5.5 m full width).
pub const LINK_HALF_WIDTH_M: f64 = 2.75;

impl Prior {
    /// Half-width in metres, honouring the ramp override. A `link` is one lane
    /// wide whatever class it carries.
    pub fn half_width_m(&self, link: bool) -> Option<f64> {
        match self.half_width_m {
            Some(_) if link => Some(LINK_HALF_WIDTH_M),
            w => w,
        }
    }

    /// The grade the relaxation holds this class's edges to.
    pub fn grade(&self) -> Option<f64> {
        self.grade_shape.grade()
    }

    /// The shoulder this class's drawn *band* carries beyond its `width_m`,
    /// in metres of half-width.
    ///
    /// Asphalt gets [`STRUCTURE_SHOULDER_M`]: a road's `width_m` is the
    /// painted carriageway, and the real surface continues past the lanes as
    /// verge and edge beam — band and deck both add it, so they meet flush.
    /// Ballast gets none: a rail class's `width_m` is the track zone
    /// (sleepers plus tamped shoulder, the §9 half-widths), the earthworks
    /// beyond are the ground bench's, and only the *structure sweep* adds the
    /// shoulder back (`synth::structure` — a real rail deck is wider than its
    /// track). Drawn with the asphalt rule a single standard-gauge track read
    /// 7 m wide, within a lane of a residential street's whole band.
    pub fn shoulder_m(&self) -> f64 {
        match self.surface {
            Surface::Asphalt => STRUCTURE_SHOULDER_M,
            Surface::Ballast | Surface::None => 0.0,
        }
    }

}

impl Kind {
    /// Parses the §9 key from Overture's `subtype`/`class`/`subclass`.
    ///
    /// An unrecognised class is *not* an error and *not* a silent default to a
    /// street: it takes its modality's `Unknown`, whose stratum is the most
    /// junior plausible one (§4.6). A misclassification that costs authority is
    /// recoverable; one that grants it is not.
    pub fn parse(subtype: Option<&str>, class: Option<&str>, _subclass: Option<&str>) -> Kind {
        // The rail and road class vocabularies are disjoint, so a caller that
        // only has the class string (the width helpers, archive-side checks)
        // still lands on the right modality: a named rail class is rail with
        // or without its subtype. Only the ambiguous `unknown` needs the
        // subtype to tell a rail from a road.
        let rail_class = |class: Option<&str>| match class {
            Some("standard_gauge") => RailClass::StandardGauge,
            Some("narrow_gauge") => RailClass::NarrowGauge,
            Some("broad_gauge") => RailClass::BroadGauge,
            Some("funicular") => RailClass::Funicular,
            Some("subway") => RailClass::Subway,
            Some("light_rail") => RailClass::LightRail,
            Some("monorail") => RailClass::Monorail,
            Some("tram") => RailClass::Tram,
            _ => RailClass::Unknown,
        };
        if subtype.is_none() && rail_class(class) != RailClass::Unknown {
            return Kind::Rail(rail_class(class));
        }
        match subtype {
            Some("rail") => Kind::Rail(rail_class(class)),
            Some("water") => Kind::Water(match class {
                Some("river" | "stream" | "canal" | "drain" | "ditch") => WaterClass::Watercourse,
                _ => WaterClass::Still,
            }),
            _ => Kind::Road(match class {
                Some("motorway") => RoadClass::Motorway,
                Some("trunk") => RoadClass::Trunk,
                Some("primary") => RoadClass::Primary,
                Some("secondary") => RoadClass::Secondary,
                Some("tertiary") => RoadClass::Tertiary,
                Some("unclassified") => RoadClass::Unclassified,
                Some("residential") => RoadClass::Residential,
                Some("living_street") => RoadClass::LivingStreet,
                Some("service") => RoadClass::Service,
                Some("track") => RoadClass::Track,
                Some("footway") => RoadClass::Footway,
                Some("path") => RoadClass::Path,
                Some("steps") => RoadClass::Steps,
                Some("cycleway") => RoadClass::Cycleway,
                Some("bridleway") => RoadClass::Bridleway,
                Some("pedestrian") => RoadClass::Pedestrian,
                Some("unknown") => RoadClass::Unknown,
                _ => RoadClass::Other,
            }),
        }
    }

    pub fn modality(self) -> Modality {
        match self {
            Kind::Road(_) => Modality::Road,
            Kind::Rail(_) => Modality::Rail,
            Kind::Water(_) => Modality::Water,
        }
    }

    /// This kind's priors (§9).
    pub fn prior(self) -> &'static Prior {
        of(self)
    }

    pub fn stratum(self) -> Stratum {
        of(self).stratum
    }
}

/// The prior table (§9), keyed by `(modality, class)`.
///
/// Values are conventions, not measurements. The *shapes* are the design point;
/// the numbers are calibrated separately.
pub fn of(kind: Kind) -> &'static Prior {
    use RailClass as Ra;
    use RoadClass as Ro;
    use WaterClass as Wa;

    // Grades: the engineered classes hold a surveyed ceiling; the rest hold a
    // bed grade, which only irons the terraces a raw DEM drape throws up
    // between nodes. The deviation budget is what separates them in effect —
    // a street breaks grade rather than dive metres below its hillside (S9).
    const MOTORWAY: Prior = Prior {
        grade_shape: GradeShape::Bounded(0.06),
        engineered: true,
        deviation_m: MAX_ROAD_DEVIATION_M,
        node_spacing_m: NODE_SPACING_M,
        clearance_over_m: ROAD_CLEARANCE_M,
        clearance_under_m: ROAD_CLEARANCE_M,
        min_structure_m: MIN_STRUCTURE_M,
        half_width_m: Some(4.5),
        surface: Surface::Asphalt,
        monotone: false,
        stratum: Stratum::S,
    };
    const TRUNK: Prior = Prior { half_width_m: Some(4.0), ..MOTORWAY };
    const PRIMARY: Prior = Prior {
        grade_shape: GradeShape::Bounded(0.08),
        engineered: false,
        deviation_m: 4.0,
        node_spacing_m: 12.0,
        half_width_m: Some(3.5),
        ..MOTORWAY
    };
    const SECONDARY: Prior = Prior {
        grade_shape: GradeShape::Bounded(0.10),
        half_width_m: Some(3.0),
        ..PRIMARY
    };
    // The long tail of streets: sparse nodes (most of the network by length),
    // a tight budget, and a bed grade that is never a solver ceiling.
    const STREET: Prior = Prior {
        grade_shape: GradeShape::Bounded(0.15),
        engineered: false,
        deviation_m: BED_MAX_DEVIATION_M,
        node_spacing_m: 24.0,
        half_width_m: Some(2.75),
        ..PRIMARY
    };
    // Draped: no profile, no authority, no asphalt. This is 46.9 % of the road
    // network (§2.2), so the discipline is load-bearing — any loophole that
    // admits one of these into a solve is a loophole through which half the
    // network can perturb the other half.
    const DRAPED: Prior = Prior {
        grade_shape: GradeShape::Draped,
        engineered: false,
        deviation_m: BED_MAX_DEVIATION_M,
        node_spacing_m: 24.0,
        clearance_over_m: ROAD_CLEARANCE_M,
        clearance_under_m: ROAD_CLEARANCE_M,
        min_structure_m: MIN_STRUCTURE_M,
        // A footpath is about as wide as a lane and its earthworks are that
        // wide too. What a draped class has no business holding is a
        // *profile*, which `GradeShape::Draped` above says outright.
        half_width_m: Some(2.75),
        surface: Surface::None,
        monotone: false,
        stratum: Stratum::D,
    };
    // **Independent rail.** A surveyed alignment on its own formation, and
    // decisively the reason its cuttings and embankments exist: the terrain is
    // a response to the railway, not the other way round (§4.2). Senior to
    // every road, engineered like a motorway and tighter — and it lays
    // ballast, not asphalt: the same surface machinery, its own material and
    // none of the road furniture.
    const MAINLINE: Prior = Prior {
        grade_shape: GradeShape::CurvatureLimited { grade: 0.03, radius_m: 2000.0 },
        engineered: true,
        deviation_m: MAX_ROAD_DEVIATION_M,
        node_spacing_m: NODE_SPACING_M,
        clearance_over_m: RAIL_CLEARANCE_M,
        clearance_under_m: ROAD_CLEARANCE_M,
        min_structure_m: MIN_STRUCTURE_M,
        // The drawn band is the *track zone* — a 2.6 m sleeper plus the
        // tamped ballast shoulder — not the formation. The earthworks beyond
        // are the ground bench's (`ground`, prior half + EARTHWORK_SHOULDER_M),
        // and a structure adds [`STRUCTURE_SHOULDER_M`] back (a real rail
        // deck or bore is wider than its track). Drawn at the 5 m formation
        // the railway read as wide as a residential street.
        half_width_m: Some(1.75),
        surface: Surface::Ballast,
        monotone: false,
        stratum: Stratum::R,
    };
    // Narrow gauge was built to reach places standard gauge could not, so it
    // holds a steeper ceiling and turns tighter; its track zone is a 1.8 m
    // sleeper plus the same tamped shoulder.
    const NARROW: Prior = Prior {
        grade_shape: GradeShape::CurvatureLimited { grade: 0.07, radius_m: 500.0 },
        half_width_m: Some(1.3),
        ..MAINLINE
    };
    // A funicular is laid *on* its hillside — no cuttings, no embankments, a
    // pair of rails pinned to the slope — so the DEM along it is its track bed
    // and the bed is the answer. Measured on the Territet–Glion line, the
    // ground under the alignment climbs at 56.9 %, which is that funicular's
    // published 54–57 %.
    //
    // This was a `Constant(0.45)` — one gradient end to end — and that model
    // failed twice over. The line arrives split into fragments at every
    // connector (four of them inside 640 m at Territet), so a chord between
    // *fragment* ends is neither the funicular's gradient nor the ground's;
    // and where a fragment opens inside a tunnel span it has no ground anchor
    // to start from at all, which floated one 15–21 m over the hill and
    // carried the junction — and both loop tracks above it — with it.
    //
    // So: a ceiling, held at the class convention and raised to the measured
    // bed where the ground earns it (`solve::profile::measured_grade`).
    //
    // 70 %, not the 45 % the constant gradient carried: the number's meaning
    // changed with the shape. As one gradient it was the *typical* incline; as
    // a ceiling it has to cover the steepest the class runs, and 45 % is under
    // Territet–Glion's own 57 %. Held too low it clamps a fragment's structure
    // chord — the Territet bore needs 47.9 % to reach its portal — and lifts
    // the tunnel out of the hillside it is bored through.
    // Bed-first and monotone, both from the same physics: the line is a cable
    // up one hill. The tight deviation keeps the at-grade line on its slope
    // (its bed *is* the terrain), and `monotone` refutes every reversal a
    // constraint interaction can manufacture — a bore ceiling diving at a
    // data-gap end, a fragment seam stepping, a junction kink.
    const FUNICULAR: Prior = Prior {
        grade_shape: GradeShape::Bounded(0.70),
        half_width_m: Some(1.3),
        deviation_m: 2.5,
        monotone: true,
        ..MAINLINE
    };
    // Street-running rail lies *on* the carriageway: rail modality, no
    // authority (S16). The right-of-way test, not the modality, decides — and
    // an unclassified railway takes the same junior default (§4.6, §10),
    // because a misclassification that costs authority is recoverable and one
    // that grants it is not.
    const STREET_RAIL: Prior =
        Prior { clearance_over_m: RAIL_CLEARANCE_M, ..DRAPED };

    match kind {
        Kind::Road(Ro::Motorway) => &MOTORWAY,
        Kind::Road(Ro::Trunk) => &TRUNK,
        Kind::Road(Ro::Primary) => &PRIMARY,
        Kind::Road(Ro::Secondary) => &SECONDARY,
        Kind::Road(
            Ro::Tertiary | Ro::Unclassified | Ro::Residential | Ro::LivingStreet | Ro::Service
            | Ro::Unknown,
        ) => &STREET,
        Kind::Road(
            Ro::Track | Ro::Footway | Ro::Pedestrian | Ro::Path | Ro::Steps | Ro::Cycleway
            | Ro::Bridleway | Ro::Other,
        ) => &DRAPED,
        Kind::Rail(Ra::StandardGauge | Ra::BroadGauge | Ra::Subway) => &MAINLINE,
        Kind::Rail(Ra::NarrowGauge) => &NARROW,
        Kind::Rail(Ra::Funicular) => &FUNICULAR,
        // Tram, light rail and monorail lie in or over a street, and an
        // unclassified railway takes the junior default (§4.6, §10).
        Kind::Rail(Ra::Tram | Ra::LightRail | Ra::Monorail | Ra::Unknown) => &STREET_RAIL,
        Kind::Water(Wa::Still | Wa::Watercourse) => &WATER,
    }
}

/// Water's prior. It publishes clearance and a datum; it holds no alignment of
/// its own, and it never excavates (§4.2).
const WATER: Prior = Prior {
    grade_shape: GradeShape::Draped,
    engineered: false,
    deviation_m: 0.0,
    node_spacing_m: 24.0,
    clearance_over_m: WATER_FREEBOARD_M,
    clearance_under_m: WATER_FREEBOARD_M,
    min_structure_m: MIN_STRUCTURE_M,
    half_width_m: None,
    surface: Surface::None,
    monotone: false,
    stratum: Stratum::H,
};

/// Vertical clearance a bridge deck's *underside* must keep over a crossed
/// road (scenario S4, I3). The data never states built clearances; these are
/// the engineering minimums.
pub const ROAD_CLEARANCE_M: f64 = 5.0;

/// The same over a railway — more, for the catenary.
pub const RAIL_CLEARANCE_M: f64 = 7.0;

/// Freeboard a deck must keep over a water surface (S3).
pub const WATER_FREEBOARD_M: f64 = 4.0;

/// Whether a bare class string names a railway.
///
/// The archive carries a feature's `class` but not its `subtype`, so anything
/// reading a tile back — the verify checks, the section cutter — has to recover
/// the modality from the class alone. `unknown` rail does not count: it is
/// indistinguishable here from a road class the parser does not recognise, and
/// [`of`] gives it the junior default for the same reason.
///
/// Here rather than beside each caller because it is a statement about the
/// class vocabulary, and because it was written twice with identical bodies
/// (`verify::scene::RoadLine::is_rail`, `verify::checks::handoff`).
pub fn class_is_rail(class: &str) -> bool {
    matches!(Kind::parse(Some("rail"), Some(class), None), Kind::Rail(c) if c != RailClass::Unknown)
}

/// Whether a bare class string draws a surface of its own — a carriageway or a
/// rail formation, not a footway.
///
/// A class with no surface contributes nothing to the union
/// (`synth::carriageway::carriageway_sources` skips it), so for anything asking
/// "which drawn band belongs to this feature" the answer for such a class is
/// "none", and a search that does not know that finds whatever happens to be
/// nearest instead.
pub fn class_paves(class: &str) -> bool {
    Kind::parse(None, Some(class), None).prior().surface != Surface::None
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

/// Physical width in metres of a class's surface band — twice the prior's
/// half-width, so the band and the deck it rides are sized from the same
/// number and meet edge-to-edge. Asphalt carriageways and rail formations
/// both have one; `None` for a class with no surface band (paths, tracks,
/// street rail), which keeps its cartographic stroke.
pub fn paint_width_m(class: Option<&str>, subclass: Option<&str>) -> Option<f64> {
    paint_width_of(Kind::parse(None, class, subclass), subclass)
}

/// [`paint_width_m`] against an already-parsed kind.
pub fn paint_width_of(kind: Kind, subclass: Option<&str>) -> Option<f64> {
    let prior = of(kind);
    if prior.surface == Surface::None {
        return None;
    }
    // The small service ways are narrower than any street class.
    if matches!(subclass, Some("driveway" | "parking_aisle" | "alley")) {
        return Some(SERVICE_WAY_WIDTH_M);
    }
    Some(2.0 * prior.half_width_m(is_link(subclass))?)
}

/// How far a mapped `width` may stray from the class prior, as factors of it,
/// before it is distrusted. Mapped widths are rare (0.6–10 % per class on the
/// Swiss extract) but where present they are usually right — the medians
/// match the priors. Beyond these bounds the measurement contradicts the
/// class (a whole right-of-way width on a footpath-sized lane, a typo'd
/// unit), and the prior is kept — the same trust-the-prior resolution the
/// clearance caps use.
pub const MEASURED_WIDTH_FACTOR_MIN: f64 = 0.35;
pub const MEASURED_WIDTH_FACTOR_MAX: f64 = 3.0;

/// Painted carriageway width in metres (docs/ROADS.md H2): the mapped
/// Overture `width_rules` value when plausible against the class prior, else
/// the prior itself ([`paint_width_m`]). `None` for a class that lays no
/// asphalt even when a width is mapped — its stroke stays cartographic until
/// it grows a surface of its own (docs/ROADS.md P5).
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

/// First zoom that paints longitudinal road markings (docs/ROADS.md P3).
/// Deeper than the surface band's zoom: a 12 cm line is sub-pixel until the
/// camera is close.
pub const MARKING_MIN_ZOOM: u8 = 15;

/// Painted line widths in metres (Swiss norms run 0.10–0.15 m) and how far
/// the edge line's centre sits in from the carriageway edge.
pub const CENTRE_LINE_WIDTH_M: f64 = 0.12;
pub const EDGE_LINE_WIDTH_M: f64 = 0.15;
pub const EDGE_LINE_INSET_M: f64 = 0.30;

/// Drawn width of one rail head, in metres. A real head is ~0.07 m; 0.12
/// matches the centre-line convention and takes the same sub-pixel floor and
/// fade the road paint does (`road.wgsl` MARK_MIN_HALF_WIDTH_PX).
pub const RAIL_HEAD_WIDTH_M: f64 = 0.12;

/// Rail gauge by class, in metres — the lateral spacing of the two painted
/// rail heads (`synth::markings::rails_for_line`).
///
/// `None` for street-running rail: it lays no ballast band and keeps its
/// cartographic stroke, so it has no surface for rail heads to ride (yet —
/// a tram's rails would sit on the *road* asphalt, a different drape).
/// Broad gauge is really 1.52–1.67 m, but the Swiss extract holds none and
/// a centimetre of gauge is invisible at these widths.
pub fn rail_gauge_m(kind: &Kind) -> Option<f64> {
    match kind {
        Kind::Rail(RailClass::StandardGauge | RailClass::BroadGauge | RailClass::Subway) => {
            Some(1.435)
        }
        Kind::Rail(RailClass::NarrowGauge | RailClass::Funicular) => Some(1.0),
        _ => None,
    }
}

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
/// it, with air between.
///
/// The dilemma was structural, not a tuning problem: on steep or tall ground the
/// earthwork must choose between a wall, a float, and no bench, and all three
/// were visible *only because the ground was drawn under the asphalt at all*.
/// Cutting the terrain back to the kerb (docs/GROUND.md §3, "the hole")
/// dissolved the choice rather than balancing it. What the cap still costs is
/// now measured at the boundary instead: the bench it declines to build is a
/// wall the model implies and `contact.kerb_lip` reports the height of, and
/// `contact.kerb_unwalled` is the share of that wall the apron fails to close.
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

/// Highest a clearance constraint may lift the road above its solved
/// profile. A real overpass
/// clears its crossed road by ~6.5–10 m (clearance plus slab, some grade),
/// ~13 m when it stacks over an already-lifted deck; a demand far beyond
/// that means the crossing geometry and the solved profile contradict each
/// other (e.g. a path mapped across a viaduct's plan line high on a flank),
/// and honouring it once flattened kilometres of viaduct at the highest
/// demand — a deck 200 m over Montreux. Such demands are dropped: the
/// profile is trusted over the inferred constraint.
pub const MAX_CLEARANCE_LIFT_M: f64 = 15.0;

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

/// Longest chain of segments joined into one corridor, in metres. Corridors
/// longer than this are split; junction-continuity constraints (solve stage)
/// carry coherence across the cut. Bounds the profile arrays and keeps a
/// mis-joined ring road from swallowing a region.
pub const MAX_CORRIDOR_M: f64 = 30_000.0;

#[cfg(test)]
mod tests {
    use super::*;

    fn road(c: RoadClass) -> &'static Prior {
        of(Kind::Road(c))
    }

    #[test]
    fn the_key_is_modality_and_class_not_class_alone() {
        // The failure this table exists to prevent: a flat class enum ending in
        // `_ => Minor` gave a mainline railway a residential street's priors.
        let rail = Kind::parse(Some("rail"), Some("standard_gauge"), None);
        let street = Kind::parse(Some("road"), Some("residential"), None);
        assert_eq!(rail.modality(), Modality::Rail);
        assert_eq!(street.modality(), Modality::Road);
        assert_ne!(rail, street);
        // Same class *string* under a different modality is a different key.
        assert_ne!(
            Kind::parse(Some("rail"), Some("unknown"), None),
            Kind::parse(Some("road"), Some("unknown"), None)
        );
    }

    #[test]
    fn an_unreadable_class_takes_the_junior_default() {
        // §4.6: a misclassification that costs authority is recoverable, one
        // that grants it is not. So an unnamed class must not inherit a
        // street's carriageway or a formation's authority.
        let other = Kind::parse(Some("road"), Some("busway"), None);
        assert_eq!(other, Kind::Road(RoadClass::Other));
        assert_eq!(other.prior().surface, Surface::None, "an unread class must not pave");
        assert_eq!(other.stratum(), Stratum::D, "nor take authority");
        // Overture's literal `unknown` is different: it is mapped as a road.
        assert_eq!(
            Kind::parse(Some("road"), Some("unknown"), None).prior().surface,
            Surface::Asphalt
        );
    }

    #[test]
    fn authority_order_is_the_stratum_order() {
        // `Ord` ascending is authority order, which is what makes "solve in
        // authority order" a sort and "is this senior" a comparison.
        assert!(Stratum::H < Stratum::R);
        assert!(Stratum::R < Stratum::S);
        assert!(Stratum::S < Stratum::D);
        assert!(Stratum::D < Stratum::B);
    }

    #[test]
    fn draped_classes_hold_no_grade_and_pave_nothing() {
        // 46.9 % of the road network. Any loophole admitting one of these into
        // a solve is a loophole through which half the network can perturb the
        // other half (§4.2).
        for c in [RoadClass::Footway, RoadClass::Path, RoadClass::Steps, RoadClass::Track,
                  RoadClass::Cycleway, RoadClass::Bridleway, RoadClass::Pedestrian] {
            let p = road(c);
            assert_eq!(p.grade_shape, GradeShape::Draped, "{c:?} holds a grade");
            assert!(p.grade().is_none(), "{c:?} reports a grade");
            assert_eq!(p.surface, Surface::None, "{c:?} paves");
            // `Other` shares this prior, so cover it here too.
            assert!(!p.engineered, "{c:?} claims a survey");
            assert_eq!(p.stratum, Stratum::D, "{c:?} has authority");
        }
    }

    #[test]
    fn only_surveyed_classes_are_engineered() {
        // The gate on rim anchoring and infeasible-anchor absorption: a
        // street's grade is a bed grade, never a standard it was built to.
        assert!(road(RoadClass::Motorway).engineered);
        assert!(road(RoadClass::Trunk).engineered);
        assert!(!road(RoadClass::Primary).engineered);
        assert!(!road(RoadClass::Residential).engineered);
        assert!(!road(RoadClass::Footway).engineered);
    }

    #[test]
    fn the_deviation_budget_is_what_lets_a_street_follow_the_hill() {
        // §9: residential is `bounded(g)` like a motorway — what differs is how
        // far it may leave the ground, not the shape of its constraint.
        assert!(matches!(road(RoadClass::Motorway).grade_shape, GradeShape::Bounded(_)));
        assert!(matches!(road(RoadClass::Residential).grade_shape, GradeShape::Bounded(_)));
        assert!(road(RoadClass::Residential).deviation_m < road(RoadClass::Motorway).deviation_m);
    }

    #[test]
    fn a_railway_is_crossed_higher_than_a_road() {
        // The catenary. Read from the *crossed* feature's prior, which is the
        // whole reason clearance moved onto the key.
        assert!(
            Kind::Rail(RailClass::StandardGauge).prior().clearance_over_m
                > Kind::Road(RoadClass::Motorway).prior().clearance_over_m
        );
        assert_eq!(Kind::Water(WaterClass::Still).prior().clearance_over_m, WATER_FREEBOARD_M);
    }

    #[test]
    fn street_running_rail_has_no_authority_but_a_railways_clearance() {
        // S16: right-of-way, not modality, decides. A tram lies on the
        // carriageway, so it is junior — and it still carries a catenary.
        let tram = Kind::Rail(RailClass::Tram);
        assert_eq!(tram.stratum(), Stratum::D);
        assert_eq!(tram.prior().clearance_over_m, RAIL_CLEARANCE_M);
        // An unclassified railway takes the same junior default (§10).
        assert_eq!(Kind::Rail(RailClass::Unknown).stratum(), Stratum::D);
        // A railway on its own formation does not.
        assert_eq!(Kind::Rail(RailClass::StandardGauge).stratum(), Stratum::R);
    }

    #[test]
    fn independent_rail_lays_ballast_and_street_rail_lays_nothing() {
        // A railway on its own formation draws a ballast band — the same
        // surface machinery a carriageway gets, its own material — and it is
        // still not asphalt: no markings, no junction plates, no casing.
        // Street-running rail lies on someone else's carriageway and draws
        // nothing of its own.
        for c in [RailClass::StandardGauge, RailClass::NarrowGauge, RailClass::BroadGauge,
                  RailClass::Funicular, RailClass::Subway] {
            assert_eq!(of(Kind::Rail(c)).surface, Surface::Ballast, "{c:?} surface");
            assert!(
                paint_width_of(Kind::Rail(c), None).is_some(),
                "{c:?} has no formation width"
            );
        }
        for c in [RailClass::LightRail, RailClass::Monorail, RailClass::Tram,
                  RailClass::Unknown] {
            assert_eq!(of(Kind::Rail(c)).surface, Surface::None, "{c:?} surface");
        }
        for c in [RailClass::StandardGauge, RailClass::NarrowGauge, RailClass::BroadGauge,
                  RailClass::Funicular, RailClass::Subway, RailClass::LightRail,
                  RailClass::Monorail, RailClass::Tram, RailClass::Unknown] {
            assert_ne!(of(Kind::Rail(c)).surface, Surface::Asphalt, "{c:?} lays asphalt");
        }
    }

    #[test]
    fn a_named_rail_class_is_rail_without_its_subtype() {
        // The width helpers and the archive-side checks only have the class
        // string. The vocabularies are disjoint, so the string alone must
        // land on the rail prior — a track-zone width, not a street's.
        assert_eq!(
            Kind::parse(None, Some("standard_gauge"), None),
            Kind::Rail(RailClass::StandardGauge)
        );
        assert_eq!(paint_width_m(Some("funicular"), None), Some(2.6));
        // The ambiguous `unknown` still needs the subtype: mapped as a road
        // it is drivable, and without one it must not become a railway.
        assert_eq!(Kind::parse(None, Some("unknown"), None), Kind::Road(RoadClass::Unknown));
    }

    #[test]
    fn half_width_scales_with_class() {
        assert!(
            road(RoadClass::Motorway).half_width_m(false)
                > road(RoadClass::Residential).half_width_m(false)
        );
        // A draped class still has a physical width — a footpath is about as
        // wide as a lane, and its earthworks are that wide. What it lacks is a
        // profile.
        assert!(road(RoadClass::Footway).half_width_m(false).is_some());
        assert_eq!(road(RoadClass::Footway).grade(), None);
    }

    #[test]
    fn links_are_narrow_whatever_the_class() {
        assert!(
            road(RoadClass::Motorway).half_width_m(true)
                < road(RoadClass::Motorway).half_width_m(false)
        );
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
        // A plain service road keeps the street width.
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
        // A class that lays no asphalt stays cartographic even with a mapped
        // width.
        assert_eq!(carriageway_width_m(Some("footway"), None, Some(1.5)), None);
    }

    #[test]
    fn dual_carriageway_decks_do_not_overlap() {
        // Overture dual-carriageway centerlines run ~8-15 m apart (measured on
        // the Swiss extract, p10 = 8.2 m); two swept motorway boxes must fit.
        assert!(2.0 * road(RoadClass::Motorway).half_width_m(false).unwrap() <= 9.0);
    }

    /// The whole point of M1 is that it moved no number. These are the values
    /// the flat five-bucket enum produced, spelled out, so a later calibration
    /// has to be deliberate.
    #[test]
    fn the_road_table_reproduces_the_buckets_it_replaced() {
        for (c, grade, dev, spacing, half) in [
            (RoadClass::Motorway, 0.06, 8.0, 8.0, 4.5),
            (RoadClass::Trunk, 0.06, 8.0, 8.0, 4.0),
            (RoadClass::Primary, 0.08, 4.0, 12.0, 3.5),
            (RoadClass::Secondary, 0.10, 4.0, 12.0, 3.0),
            (RoadClass::Residential, 0.15, 2.5, 24.0, 2.75),
            (RoadClass::Service, 0.15, 2.5, 24.0, 2.75),
            (RoadClass::Tertiary, 0.15, 2.5, 24.0, 2.75),
        ] {
            let p = road(c);
            assert_eq!(p.grade(), Some(grade), "{c:?} grade");
            assert_eq!(p.deviation_m, dev, "{c:?} deviation");
            assert_eq!(p.node_spacing_m, spacing, "{c:?} spacing");
            assert_eq!(p.half_width_m(false), Some(half), "{c:?} half-width");
            assert_eq!(p.surface, Surface::Asphalt, "{c:?} must pave");
        }
    }
}
