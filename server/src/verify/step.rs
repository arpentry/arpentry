//! Which pipeline step a metric can falsify.
//!
//! A scorecard tells you *that* something is wrong. It has never told you
//! *where* — 77 metrics arrive as one flat list, and the first hour of any
//! investigation goes on deciding which stage to go and read. This module is
//! that decision, made once and written down.
//!
//! **The assignment is ownership, not correlation.** A metric belongs to the
//! step whose code you would open to fix it. That is deliberately not the same
//! question as "which rung of the terrain dial moves it": the dial measures
//! whether a metric *responds* to the ground, which is strong evidence about
//! ownership and occasionally cuts against it. `slope.rail_grade` reads
//! identically on a plane, a ramp, a cliff and the real DEM, and is still a
//! height property — the solve simply does not take that grade from the
//! terrain. Where the dial and the semantics agree, the assignment below cites
//! the measurement; where they disagree, the semantics win and the note says so.
//!
//! **One table, not a field on every metric.** The alternative was a `step` on
//! each of the 77 `Metric` literals, spread across 25 files, where no two
//! assignments could ever be compared and the taxonomy would exist only as an
//! emergent property of the whole tree. Here it is a list you can read top to
//! bottom and argue with. [`tests::every_metric_has_a_step`] holds it to
//! `checks::all()` and the committed scorecards, so a new check cannot land
//! unassigned — the same guard `verify::documented` puts on the docs table.

use std::fmt;

/// A stage of the pipeline, as something a defect can be attributed to.
///
/// These are coarser than the module tree on purpose: `assemble` and the
/// plan-space half of `synth` are one step here, because a kerb that fails to
/// reach its neighbour is one investigation however many modules it crosses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Step {
    /// **Plan** — what is drawn where, seen from above. Corridor assembly,
    /// carriageway widths and the union, kerb lines, pavement attachment,
    /// crossing registration, paint registration in plan. Nothing here is a
    /// height question, and the terrain dial confirms it: these metrics read
    /// the same on a plane and on a mountain.
    Plan,
    /// **Height** — the vertical solve. Profiles, junction agreement,
    /// clearance and vertical order, the relaxation residuals, and which spans
    /// come out as structures.
    Height,
    /// **Ground** — the engineered ground. Benches, batters, the imprint and
    /// its footprints, the hole, and every check that scores a drawn surface
    /// against the ground beneath it.
    Ground,
    /// **Draw** — the drawn surface. Sheets, bands, meshes, closure, and the
    /// coverage questions: whether something was drawn at all, and whether two
    /// drawn things meet.
    Draw,
    /// **Tile** — the archive. Cross-tile seams, drift between zooms,
    /// determinism. A defect here is in how one world was cut up, not in the
    /// world.
    Tile,
}

impl Step {
    /// Every step, in pipeline order — which is also the order to debug them
    /// in: a plan defect surfaces again as a height defect, never the reverse.
    pub const ALL: [Step; 5] = [Step::Plan, Step::Height, Step::Ground, Step::Draw, Step::Tile];

    pub fn as_str(self) -> &'static str {
        match self {
            Step::Plan => "plan",
            Step::Height => "height",
            Step::Ground => "ground",
            Step::Draw => "draw",
            Step::Tile => "tile",
        }
    }

    /// One line: what this step owns, for the scorecard's group heading.
    pub fn owns(self) -> &'static str {
        match self {
            Step::Plan => "what is drawn where, seen from above",
            Step::Height => "the vertical solve and the structures it implies",
            Step::Ground => "the engineered ground and what stands on it",
            Step::Draw => "the drawn surface: coverage, closure, and paint",
            Step::Tile => "cutting one world into tiles and zooms",
        }
    }

    /// The step a metric id belongs to, or `None` for an id this table has
    /// never heard of.
    ///
    /// `None` rather than a default: an unassigned metric that quietly reads
    /// "plan" would send someone to the wrong stage, which is worse than
    /// sending them nowhere. The test below is what keeps `None` empty.
    pub fn of(id: &str) -> Option<Step> {
        use Step::*;
        // The relaxation residuals are one family and one answer.
        if id.starts_with("solve.residual_") {
            return Some(Height);
        }
        Some(match id {
            // ── Plan ────────────────────────────────────────────────────────
            // Measured invariant across the whole terrain dial at three sites:
            // 250/250/250/251 violations on flat, ramp, step and the real DEM,
            // worst 22 m at every rung.
            "street.kerb_gap" => Plan,
            "street.walk_width_step" => Plan,
            "street.width_step" => Plan,
            "street.kerb_join" => Plan,
            "street.strip_continuity" => Plan,
            "street.crossing_extent" => Plan,
            "street.crossing_skew" => Plan,
            // A mapped way with nothing drawn for it is a plan-space failure to
            // build, whatever the ground under it would have been.
            "network.walk_cover" => Plan,
            "network.walk_material" => Plan,
            "network.walk_reach" => Plan,
            // Overlap is a plan fact. That the model cannot order the overlap
            // is what the metric reports, but the overlap itself is drawn here.
            "order.at_grade_overlap" => Plan,
            "order.building_overlap" => Plan,
            "order.walk_indoors" => Plan,
            "order.walk_on_asphalt" => Plan,
            // Registration: is the paint on the thing it belongs to, in plan.
            "paint.marking_offside" => Plan,
            "paint.edge_line_inset" => Plan,
            "paint.marking_on_crossing" => Plan,
            // Two constructions of one arc, compared in plan.
            "seam.abutment_plan" => Plan,

            // ── Height ──────────────────────────────────────────────────────
            "seam.abutment_step" => Height,
            "seam.band_deck_step" => Height,
            "order.deck_above_carriageway" => Height,
            "order.walk_level" => Height,
            "order.grade_stack" => Height,
            "clearance.deck_over_ground" => Height,
            // The dial's own lesson: on a plane the ordering constraint
            // outlives the mountain, the solve digs, and this fires. It scores
            // the solve against the ground, and the solve owns it.
            "clearance.bore_cover" => Height,
            "contact.deck_carried" => Height,
            "contact.sidewalk_grade" => Height,
            "contact.level_crossing" => Height,
            "slope.road_grade" => Height,
            // Invariant across the dial (4/74 identically at Villeneuve on
            // flat, ramp, step and dem) — and still a height property. The
            // solve does not take a rail's grade from the terrain.
            "slope.rail_grade" => Height,
            "graph.connector_step" => Height,
            "network.walk_joint" => Height,
            "partition.divergence" => Height,
            "partition.junction_joint" => Height,
            "crossing.orphan" => Height,
            "datum.float" => Height,
            "water.descends" => Height,
            "solve.determinism" => Height,
            "authority.inversion_R" | "authority.inversion_S" => Height,
            "structure.bore_daylight" => Height,
            "structure.annotated_lost" => Height,
            "structure.derived_new" => Height,
            "structure.edge_drift" => Height,

            // ── Ground ──────────────────────────────────────────────────────
            // Measured curvature-driven at all three sites: near-zero on a
            // plane, largest on the hill and the real DEM, barely moved by a
            // constant grade or a local cliff.
            "slope.walk_crossfall" => Ground,
            "contact.kerb_lip" => Ground,
            "contact.walk_rim" => Ground,
            "contact.rail_standoff" => Ground,
            "contact.deck_seat" => Ground,
            "contact.building_seat" => Ground,
            "slope.terrain_face" => Ground,
            "slope.terrain_tearing" => Ground,
            "ground.footprint" => Ground,
            "ground.single_source" => Ground,
            "authority.facade_ground" => Ground,

            // ── Draw ────────────────────────────────────────────────────────
            // Coverage and closure: was a surface drawn, and do two drawn
            // surfaces meet.
            "fabric.closure" => Draw,
            "seam.abutment_bare" => Draw,
            "seam.band_deck_bare" => Draw,
            "seam.band_deck_open" => Draw,
            "seam.handover_kerb" => Draw,
            "contact.kerb_unwalled" => Draw,
            "slope.carriageway_face" => Draw,
            "paint.buried" => Draw,
            "paint.stroke_over_band" => Draw,

            // ── Tile ────────────────────────────────────────────────────────
            "seam.terrain_step" => Tile,
            "seam.terrain_split" => Tile,
            "seam.terrain_shade" => Tile,
            "seam.terrain_spill" => Tile,
            "seam.pavement_step" => Tile,
            "lod.structure_drift" => Tile,

            _ => return None,
        })
    }
}

impl fmt::Display for Step {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn ids_in(scorecard: &str) -> BTreeSet<String> {
        let path = format!("{}{scorecard}", env!("CARGO_MANIFEST_DIR"));
        let text = std::fs::read_to_string(&path).expect(&path);
        let card: serde_json::Value = serde_json::from_str(&text).expect(&path);
        card["metrics"]
            .as_array()
            .expect("metrics")
            .iter()
            .filter_map(|m| m["id"].as_str().map(str::to_string))
            .collect()
    }

    /// Every metric the harness can produce is assigned. Built the same way as
    /// `verify::documented`: the archive-side checks answer for themselves
    /// without an archive, and the committed scorecards answer for the model
    /// half, which needs a solved `Model` and cannot be conjured here.
    #[test]
    fn every_metric_has_a_step() {
        let opt = crate::verify::checks::Options::default();
        let mut ids: BTreeSet<String> = crate::verify::checks::all(&opt)
            .into_iter()
            .flat_map(|c| c.finish())
            .map(|m| m.id)
            .collect();
        for card in ["/verify/baseline-montreux-z16.json", "/verify/model-montreux-z16.json"] {
            ids.extend(ids_in(card));
        }

        let unassigned: Vec<&String> = ids.iter().filter(|id| Step::of(id).is_none()).collect();
        assert!(
            unassigned.is_empty(),
            "no step in verify::step for: {unassigned:?} — add a row, don't add a default"
        );
    }

    #[test]
    fn the_residual_family_answers_as_one() {
        for r in ["grade", "clearance", "rigidity", "monotone", "contact", "undercut"] {
            assert_eq!(Step::of(&format!("solve.residual_{r}")), Some(Step::Height));
        }
    }

    #[test]
    fn an_unknown_id_is_unassigned_not_defaulted() {
        assert_eq!(Step::of("street.something_new"), None);
        assert_eq!(Step::of(""), None);
    }

    #[test]
    fn the_steps_are_listed_in_pipeline_order() {
        // ALL is the debugging order as well as the pipeline's: a plan defect
        // surfaces again as a height defect, never the reverse.
        assert_eq!(Step::ALL[0], Step::Plan);
        assert_eq!(Step::ALL[4], Step::Tile);
        let mut sorted = Step::ALL;
        sorted.sort();
        assert_eq!(sorted, Step::ALL, "the enum's order must be the pipeline's");
    }
}
