//! The arrangement: the rect as one planar subdivision, every face tagged.
//!
//! Step 1 of `data/plans/one-ground-2026-09-16.md`. Until now the paved
//! surface and the hole under it were two regions built by two expressions:
//! `mesh::by_sheet` meshes each sheet on its own, while the ground is cut from
//! `union_all(sheets.shapes()) − spanned`. A union dissolves the edge where
//! two pieces touch and puts vertices where they crossed; the separately
//! meshed pieces keep that edge and have no such vertices. The two boundaries
//! are then *different*, and a vertex of one has no counterpart in the other
//! — which is what `bench`'s `seam` and `unmet` count, and what the cross-mesh
//! lookup's eight-cell search was added to paper over.
//!
//! **It was never a rounding problem.** The plan and CLAUDE.md both say the
//! cause is that "a point that has been through one more boolean than its
//! neighbour lands up to half a grid away". It cannot be: [`crate::poly`]
//! pins its adapter, so every output point is an exact multiple of
//! [`poly::GRID_M`] and feeding one back in is the identity
//! (`poly::tests::a_boolean_over_a_snapped_operand_is_idempotent`). Two
//! different regions is the whole of it, and no care with the lattice
//! reconciles two different regions.
//!
//! So the rect is cut **once**, by every material boundary at once
//! ([`poly::slice`]), and what comes back is a set of faces that partition it
//! with one set of split points: a vertex on a shared edge is the same `Pt`,
//! bit for bit, in both faces that carry it. Each face is then asked what it
//! is by a single interior point ([`poly::inside`]) — a tag, not another
//! boolean.
//!
//! **The walls are not cuts, and that was measured.** The asphalt stops at
//! the *closed* facade (`facade.built`) and the pavement at the open one
//! (`facade.solid`), so adding both as cuts looks obviously right. On the
//! junction-with-houses specimen it takes `seam` 25 → 20 and `unmet` **4 →
//! 8**: it moves one number the right way and the other the wrong way, which
//! is not a fix. Whatever the residual there is, it is not a wall the faces
//! have not been told about.
//!
//! **The mesher and the bench read it**: the mesh triangulates the faces of
//! each material, and the bench takes the hole, the span mask and the edges
//! from here rather than building its own.

use crate::bench::{Portals, OVER_RIM_M, ROOM_REACH_M};
use crate::frame::Extent;
use crate::poly::{self, Indexed, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::world::{Polyline2, Profiles, Sheets, Surface};

/// What a face of the arrangement is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    /// Not paved: the terrain, and the only material the ground mesh draws.
    Ground,
    Carriageway,
    Pavement,
    Ballast,
}

impl Material {
    pub fn name(self) -> &'static str {
        match self {
            Material::Ground => "ground",
            Material::Carriageway => "carriageway",
            Material::Pavement => "pavement",
            Material::Ballast => "ballast",
        }
    }
}

/// One face of the subdivision: a region of the rect, and what it is.
#[derive(Debug, Clone)]
pub struct Face {
    pub shape: Shape,
    pub material: Material,
    /// The sheet this face belongs to, for the families that have them
    /// ([`crate::width::Family::solves`]). `None` for ground, for the
    /// pavement — which is not partitioned — and for a paved face no sheet
    /// claims.
    pub sheet: Option<usize>,
    /// Over a span rather than on the ground. Paved, and **it does not cut
    /// the ground**: a viaduct flies over terrain that is still there
    /// (invariant I3), and the soffit is what closes under it.
    pub spanned: bool,
    /// Inside a gallery's footprint — a road under the ground whose tube
    /// fits nowhere. Ground, and it *does* cut: the tube stands in the
    /// trench the bench digs for it.
    pub gallery: bool,
    /// Within [`crate::room::WALL_REACH_M`] of the asphalt: the band the
    /// bench lifts to the road's height, against the part beyond it that
    /// drapes.
    ///
    /// **A cut, not a boolean.** The two rules disagree by up to the whole
    /// drop — 15.6 m on the loop box — and every face the bench draws is
    /// built off a *rim*, so the walk has to be meshed in two parts with the
    /// step between them on an edge of each. `mesh` used to `dilate` and
    /// `intersect` to find that line, which put vertices on the walk that no
    /// other mesh had. Here it is one more boundary where the ground does
    /// something different on each side, which is exactly what a cut is.
    pub near: bool,
}

impl Face {
    /// Whether this face cuts the terrain's hole.
    ///
    /// Paving on the ground does; paving over a span does not (invariant I3:
    /// a viaduct flies over ground that is still there, and the soffit closes
    /// under it); a gallery does, though it is ground — the tube stands in
    /// the trench the bench digs for it.
    pub fn cuts(&self) -> bool {
        self.gallery || (self.material != Material::Ground && !self.spanned)
    }
}

/// The rect, partitioned.
#[derive(Debug, Clone, Default)]
pub struct Arrangement {
    pub faces: Vec<Face>,
    /// **The second layer: paving over a span that lies over other paving.**
    ///
    /// The partition has one face per point, and it is the *ground's*: where
    /// a deck crosses a street in plan, the face under it is the street's,
    /// which cuts the terrain and meets its own kerbs. The deck's paving
    /// over that face is here instead — the same shape, from the same slice,
    /// so it welds to the rest of its sheet by position exactly as the faces
    /// do. Never in [`Arrangement::edges`] and never in the hole: a deck's
    /// side is closed by its slab, and the ground under it is the street's.
    pub decks: Vec<Face>,
    /// The paving over a span that `Face::spanned` was tagged by
    /// ([`over_spans`]), kept so that `bench` reads the one mask rather
    /// than rebuilding it: the hole and the lift must agree on it.
    pub over: Shapes,
    /// The area of the walk a deck carries, the part of `over` the sheets
    /// alone do not hold.
    pub carried_m2: f64,
    /// [`Arrangement::edges`], computed once.
    pub edges: Vec<Edge>,
}

impl Arrangement {
    /// The faces of `material`.
    pub fn of(&self, material: Material) -> impl Iterator<Item = &Face> {
        self.faces.iter().filter(move |f| f.material == material)
    }

    /// The same over both layers: every face of `material` a mesh must
    /// draw, the ground's and the decks' over it.
    pub fn layered(&self, material: Material) -> impl Iterator<Item = &Face> {
        self.faces.iter().chain(&self.decks).filter(move |f| f.material == material)
    }

    /// The regions that cut the terrain's hole: every paved face on the
    /// ground, and every gallery's. The arrangement's answer to `bench`'s
    /// `outline`, as faces rather than as an expression.
    pub fn hole(&self) -> Shapes {
        self.faces.iter().filter(|f| f.cuts()).map(|f| f.shape.clone()).collect()
    }
}

/// The paving that is **over a span**, as `bench` has always drawn it: the
/// sheets' own span regions, the walk a deck carries, and a centimetre of
/// rim so that a vertex *on* a span's edge reads as over it.
///
/// A deck's free edge is the highest thing on it, and left out of the mask it
/// put the whole standoff back into `fill`. The carried walk is here for the
/// reason `structure::carried` cannot reach: a sidewalk running along a
/// bridge the mapper never tagged is over a span with no span of its own to
/// ask about, and it opened a hole in the ground eight metres beneath itself.
///
/// Returns the mask and the carried walk's area.
pub fn over_spans(surface: &Surface, sheets: &Sheets) -> (Shapes, f64) {
    let on_spans = sheets.spanned();
    let carried = poly::intersect(&surface.walk, &poly::dilate(&on_spans, ROOM_REACH_M));
    let carried_m2 = poly::area(&carried);
    (poly::dilate(&poly::union_of(&[&on_spans, &carried]), OVER_RIM_M), carried_m2)
}

/// One edge of the subdivision: a segment, and the faces on each side.
///
/// The unit [`crate::bench`]'s rule works on — "an arrangement edge is either
/// *welded*, its two faces sharing vertices and one height, or *split*, each
/// face taking its own and the mesher emitting the quad between them"
/// (`data/plans/one-ground-2026-09-16.md` §3.3). A segment rather than a
/// maximal shared boundary, because that is what a mesher emits a quad
/// across.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub a: Pt,
    pub b: Pt,
    /// The faces either side. `right` is `None` on the rect's own boundary,
    /// where there is nothing on the far side.
    pub left: usize,
    pub right: Option<usize>,
}

impl Arrangement {
    /// Every edge of the subdivision, with the faces it separates.
    ///
    /// **This is only meaningful if the faces share their segments exactly**,
    /// which is what one slice is supposed to give and what `edges` /
    /// `dangling` in the step's line measure. A segment carried by three
    /// faces, or by one away from the rect's border, is a subdivision that is
    /// not one, and no edge rule can be written over it.
    fn edges_of(faces: &[Face]) -> Vec<Edge> {
        let key = |p: &Pt| (p[0].to_bits(), p[1].to_bits());
        let mut at: std::collections::HashMap<((u64, u64), (u64, u64)), Vec<usize>> =
            Default::default();
        let mut seg: std::collections::HashMap<((u64, u64), (u64, u64)), (Pt, Pt)> =
            Default::default();
        for (i, f) in faces.iter().enumerate() {
            for ring in &f.shape {
                for k in 0..ring.len() {
                    let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                    let (ka, kb) = (key(&a), key(&b));
                    if ka == kb {
                        continue;
                    }
                    let e = if ka <= kb { (ka, kb) } else { (kb, ka) };
                    at.entry(e).or_default().push(i);
                    seg.entry(e).or_insert((a, b));
                }
            }
        }
        let mut out: Vec<Edge> = at
            .into_iter()
            .map(|(e, faces)| {
                let (a, b) = seg[&e];
                Edge { a, b, left: faces[0], right: faces.get(1).copied() }
            })
            .collect();
        // A function of the faces, not of a hash order.
        out.sort_by(|x, y| {
            (x.a[0], x.a[1], x.b[0], x.b[1]).partial_cmp(&(y.a[0], y.a[1], y.b[0], y.b[1])).unwrap()
        });
        out
    }
}

/// The whole rect as **one mesh**: every face triangulated in a single pass,
/// so the materials share their boundary vertices by index and not merely by
/// position.
///
/// §3.1's "one mesh" and the thing §3.2's solve needs — it wants one vertex
/// set to relax over, and today the ground and each material are separate
/// `Tri`s that meet only at coincident positions. `of_face` is the face each
/// triangle came from, so a material is a filter over the triangles rather
/// than a mesh of its own.
///
/// **A vertex here is one height, and some vertices need two.** Where a kerb
/// splits, the road and the pavement answer 0.12 m apart and one vertex
/// cannot carry both (§3.2 against §3.3). `split_vertices` in the step's line
/// counts them — 331 against ~85 000 on the junction with houses — and
/// duplicating them is what a caller must do before lifting. This function
/// does not: it is the arrangement meshed, and what heights go on it is the
/// bench's.
pub struct OneMesh {
    pub tri: crate::world::Tri,
    /// The face each triangle came from, indexing [`Arrangement::faces`].
    pub of_face: Vec<u32>,
    pub stats: crate::mesh::Stats,
}

impl Arrangement {
    /// Triangulates every face at once over `grid`, each vertex at `height`.
    pub fn mesh(
        &self,
        grid: &crate::grid::Grid,
        height: &dyn Fn(poly::Pt) -> f64,
    ) -> OneMesh {
        let shapes: Shapes = self.faces.iter().map(|f| f.shape.clone()).collect();
        let (tri, of_face, stats) = crate::mesh::tagged(&shapes, grid, height);
        OneMesh { tri, of_face, stats }
    }
}

/// Cuts the rect into faces along every material boundary, and tags them.
pub fn run(
    extent: &Extent,
    spans: &[Polyline2],
    profiles: &Profiles,
    surface: &Surface,
    sheets: &Sheets,
) -> (Arrangement, Summary) {
    let r = &extent.rect;
    let outer = vec![poly::rect(r.x0, r.y0, r.x1, r.y1)];
    let (spanned, carried_m2) = over_spans(surface, sheets);
    let (_, galleries) = Portals::new(spans, profiles);
    // **Clipped to the walk.** The reach means one thing only — which side
    // of it a *pavement* face is on — and the ring itself runs on through
    // open ground where the walk is narrower than it. Cut over the whole
    // rect it splits the ground there too, which is not wrong but is a
    // boundary that says nothing: it put extra vertices in the ground mesh,
    // and on a 50 % ramp one of them found a deeper place than any vertex
    // before it and took `cut` from 4.255 m to 4.755. A cut is for a
    // boundary something differs across.
    let reach = poly::intersect(&poly::dilate(&surface.carriageway, crate::room::WALL_REACH_M), &surface.walk);

    // **The asphalt is cut by the sheets, not by `surface.carriageway`.** A
    // sheet holds paving the surface does not — the span ribbons merged into
    // its group, and the kerb returns the fillet added once a junction's
    // decks had joined it — and the sheets are what `mesh` triangulates. Cut
    // by the surface instead, a face would end where no mesh does.
    let (mut paved, mut paved_family): (Shapes, Vec<crate::width::Family>) = (Vec::new(), Vec::new());
    for s in &sheets.sheets {
        for shape in &s.shapes {
            paved.push(shape.clone());
            paved_family.push(s.family);
        }
    }

    // Every boundary that matters, as cut lines. The span mask and the
    // galleries are among them: each says the ground does something different
    // on one side than the other, which is what a cut is for.
    let mut cuts = poly::rings(&paved);
    cuts.extend(poly::rings(&surface.walk));
    cuts.extend(poly::rings(&spanned));
    cuts.extend(poly::rings(&galleries));
    cuts.extend(poly::rings(&reach));
    let ring_count = cuts.len();
    // **Sliced, then conformed.** The slice gives faces that share their
    // split points, which is what a mesh needs to weld; [`poly::conform`]
    // makes them agree *segment for segment*, which is what an edge rule
    // needs (§3.3) and what `dangling` measures. It costs `bench seam` — 20 →
    // 45 on `house:across` — and is landed anyway: an edge rule that cannot
    // be written is worth more than a diagnostic of the two-mesh world it
    // replaces.
    let faces = poly::conform(&poly::slice(&outer, &cuts));

    // The tags, by one interior point per face. Indexed because there are as
    // many queries as faces and as many regions as the world has paving.
    let sheet_paving = Indexed::new(&paved);
    let walk = Indexed::new(&surface.walk);
    let over = Indexed::new(&spanned);
    let hollow = Indexed::new(&galleries);
    let within = Indexed::new(&reach);
    // Which *sheet* a paved face is in, over the same region list: a sheet is
    // several regions, and this maps a region back to it.
    let sheet_of: Vec<usize> = sheets
        .sheets
        .iter()
        .enumerate()
        .flat_map(|(i, s)| std::iter::repeat_n(i, s.shapes.len()))
        .collect();
    // Which sheet's paving is over one of its own spans, over the same
    // device: a sheet's `spans` is the part of its paving the ground steps
    // must be able to tell.
    let span_paving: Shapes = sheets.sheets.iter().flat_map(|s| s.spans.iter().cloned()).collect();
    let span_of: Vec<usize> = sheets
        .sheets
        .iter()
        .enumerate()
        .flat_map(|(i, s)| std::iter::repeat_n(i, s.spans.len()))
        .collect();
    let span_index = Indexed::new(&span_paving);
    let family_of = |i: usize| match sheets.sheets[i].family {
        crate::width::Family::Rail => Material::Ballast,
        crate::width::Family::Walk => Material::Pavement,
        crate::width::Family::Carriageway => Material::Carriageway,
    };

    let mut out = Vec::with_capacity(faces.len());
    let mut decks: Vec<Face> = Vec::new();
    let mut unprobed: Vec<usize> = Vec::new();
    let mut unprobed_m2 = 0.0f64;
    for shape in faces {
        // A face too thin to name a point inside takes its neighbour's
        // material below. Calling it ground instead is what a first cut did,
        // and on `house:across` it cost 40 of the corpus's 40 remaining
        // `seam` misses: four sub-millimetre slivers — the hairline the
        // facade's own docs warn of, where a passage corridor's edge crosses
        // a wall a lattice step off it — sat *inside* the asphalt, and a
        // ground face inside the paving punches a ring into the hole's
        // outline that no paved mesh has a vertex for. `dense` then
        // subdivides that ring at every lattice crossing.
        let Some(p) = poly::inside(&shape) else {
            unprobed.push(out.len());
            unprobed_m2 += poly::area(std::slice::from_ref(&shape));
            out.push(Face {
                shape,
                material: Material::Ground,
                sheet: None,
                spanned: false,
                gallery: false,
                near: false,
            });
            continue;
        };
        // **Two sheets at one point is a grade separation.** The sheets'
        // regions are disjoint on the ground, so where two of them hold one
        // face, one is flying: a sheet whose own span paving holds the face
        // is over it, and a sheet whose ground paving does is under it. The
        // one under keeps the face — it is on the ground, cuts the terrain
        // and meets its own kerbs — and every one over it gets a deck face
        // of the same shape. With one sheet, or none on the ground, nothing
        // changes: the partition's face is the first sheet's, as it was.
        let mut holders: Vec<usize> = Vec::new();
        for r in sheet_paving.all(p) {
            if !holders.iter().any(|&h| sheet_of[h] == sheet_of[r]) {
                holders.push(r);
            }
        }
        if holders.len() > 1 {
            let over_own: Vec<usize> = span_index.all(p).into_iter().map(|k| span_of[k]).collect();
            let (upper, lower): (Vec<usize>, Vec<usize>) =
                holders.iter().partition(|&&r| over_own.contains(&sheet_of[r]));
            if let (false, Some(&ground)) = (upper.is_empty(), lower.first()) {
                for &r in &upper {
                    decks.push(Face {
                        shape: shape.clone(),
                        material: family_of(sheet_of[r]),
                        sheet: Some(sheet_of[r]),
                        spanned: true,
                        gallery: false,
                        near: false,
                    });
                }
                out.push(Face {
                    shape,
                    material: family_of(sheet_of[ground]),
                    sheet: Some(sheet_of[ground]),
                    spanned: false,
                    gallery: hollow.contains(p),
                    near: within.contains(p),
                });
                continue;
            }
        }
        // The sheets answer first and carry their own family; the walk is
        // what is paved and in no sheet, because the pavement is not
        // partitioned.
        let region = sheet_paving.which(p);
        let material = match region.map(|i| paved_family[i]) {
            Some(crate::width::Family::Carriageway) => Material::Carriageway,
            Some(crate::width::Family::Rail) => Material::Ballast,
            Some(crate::width::Family::Walk) => Material::Pavement,
            None if walk.contains(p) => Material::Pavement,
            None => Material::Ground,
        };
        out.push(Face {
            shape,
            material,
            sheet: region.map(|i| sheet_of[i]),
            spanned: over.contains(p),
            gallery: hollow.contains(p),
            near: within.contains(p),
        });
    }

    adopt(&mut out, &unprobed);
    let edges = Arrangement::edges_of(&out);
    let arrangement = Arrangement { faces: out, decks, over: spanned, carried_m2, edges };
    let summary = measure(&arrangement, extent, ring_count, unprobed.len(), unprobed_m2);
    (arrangement, summary)
}

/// Gives every face of `orphans` the material of the neighbour it shares the
/// most **vertices** with.
///
/// A face too thin to hold a probe is too thin to be anything of its own: it
/// is a lattice artefact of two cut lines running a hair apart, and the
/// honest answer is that it belongs to whatever is around it. Doing this
/// rather than calling them ground is what lets `outline`'s union absorb
/// them — a sliver that is asphalt like the face beside it has no boundary
/// of its own left — and on `house:across` it is 40 of the corpus's 40
/// remaining `seam` misses.
fn adopt(faces: &mut [Face], orphans: &[usize]) {
    if orphans.is_empty() {
        return;
    }
    type Key = (u64, u64);
    let key = |p: &Pt| (p[0].to_bits(), p[1].to_bits());
    // Keyed on **vertices**, not on edges. A sliver's neighbour may have
    // subdivided the edge they share — the whole reason the sliver exists is
    // that two cut lines ran a hair apart — so an edge-for-edge match finds
    // the wrong neighbours, or none. Measured on the corpus: shared vertices
    // 20 misses, shared edges 25 either as a count or weighted by length.
    let mut at: std::collections::HashMap<Key, Vec<usize>> = Default::default();
    for (i, f) in faces.iter().enumerate() {
        for p in f.shape.iter().flatten() {
            at.entry(key(p)).or_default().push(i);
        }
    }
    for &i in orphans {
        let mut shared: std::collections::HashMap<usize, usize> = Default::default();
        for p in faces[i].shape.iter().flatten() {
            for &j in at.get(&key(p)).into_iter().flatten() {
                if j != i {
                    *shared.entry(j).or_default() += 1;
                }
            }
        }
        // Most shared vertices wins, ties to the lowest index so the answer
        // is a function of the faces and not of a hash order.
        //
        // **Preferring a neighbour that *cuts* was tried and is worse**, at
        // 25 misses against 20 — which is worth recording, because it is the
        // rule the failure mode argues for: a non-cutting face inside paving
        // that cuts is exactly what punches a spurious ring into the hole, so
        // sending every sliver into the hole ought to help. It does not, and
        // that says the remaining misses are not this.
        let Some((&j, _)) = shared.iter().max_by_key(|(j, n)| (**n, std::cmp::Reverse(**j))) else {
            continue;
        };
        let n = faces[j].clone();
        let f = &mut faces[i];
        (f.material, f.sheet, f.spanned, f.gallery, f.near) =
            (n.material, n.sheet, n.spanned, n.gallery, n.near);
    }
}

/// What the subdivision came to: its size, its materials, and the two
/// properties it exists for.
fn measure(
    a: &Arrangement,
    extent: &Extent,
    rings: usize,
    unprobed: usize,
    unprobed_m2: f64,
) -> Summary {
    // `+ 0.0` normalises the negative zero an empty sum can carry, so a
    // material that is not there reads `0` rather than `-0`.
    let m2 = |m: Material| -> f64 {
        a.of(m).map(|f| poly::area(std::slice::from_ref(&f.shape))).sum::<f64>() + 0.0
    };
    let r = &extent.rect;
    let total: f64 = a.faces.iter().map(|f| poly::area(std::slice::from_ref(&f.shape))).sum();

    // **It is a partition.** The faces cover the rect and do not overlap, so
    // their areas sum to it. A drift here is the kernel losing or doubling a
    // face, and it is the one thing that would make everything downstream
    // wrong at once.
    //
    // Reported **against the lattice**, not in bare square metres. The rect's
    // own corners are snapped to [`poly::GRID_M`] on the way in, which moves
    // its area by up to half a grid step along each side of its perimeter —
    // 0.033 m² on the loop box's 1.5 km × 1.1 km, which is 2e-8 of it and not
    // a face going missing. `closure` is the drift as a multiple of that
    // bound, so 1.0 is the most the snapping can explain and anything above
    // it is the kernel.
    let perimeter = 2.0 * (r.width() + r.height());
    let closure = (total - r.width() * r.height()).abs() / (perimeter * poly::GRID_M);

    // **One set of split points.** Every vertex away from the rect's own
    // edge is carried by at least two faces — the two that meet along it. A
    // vertex belonging to one face alone is a boundary that was built twice,
    // which is the defect this step exists to remove, and it must read 0.
    let on_border = |p: &Pt| {
        (p[0] - r.x0).abs() < 1e-9
            || (p[0] - r.x1).abs() < 1e-9
            || (p[1] - r.y0).abs() < 1e-9
            || (p[1] - r.y1).abs() < 1e-9
    };
    let mut seen: std::collections::HashMap<(u64, u64), usize> = std::collections::HashMap::new();
    for p in a.faces.iter().flat_map(|f| f.shape.iter().flatten()) {
        *seen.entry((p[0].to_bits(), p[1].to_bits())).or_default() += 1;
    }
    let lone = a
        .faces
        .iter()
        .flat_map(|f| f.shape.iter().flatten())
        .filter(|p| !on_border(p))
        .filter(|p| seen[&(p[0].to_bits(), p[1].to_bits())] == 1)
        .count();

    // The adjacency step 2's edge rule needs, and the check that it exists:
    // every segment away from the rect's own border must be carried by
    // exactly two faces.
    let edges = &a.edges;
    let dangling = edges
        .iter()
        .filter(|e| e.right.is_none() && !(on_border(&e.a) && on_border(&e.b)))
        .count();
    // **How many vertices carry more than one answer.** §3.2 wants one height
    // per vertex and §3.3 wants a kerb edge *split* — and a kerb vertex has
    // two heights, a kerb's rise apart. So a single mesh over the whole rect
    // cannot be one vertex per position: it has to duplicate wherever the
    // faces meeting at a vertex do not all agree.
    //
    // **Read it against the mesh, not against this step.** Nearly every
    // vertex *here* is on a boundary — the arrangement's vertices are the
    // cut points, so 96 % of them meeting two materials is close to a
    // tautology. The mesh adds a vertex at every lattice crossing inside a
    // face, tens of thousands of them, and the count below is the numerator
    // over that: on the junction with houses, 331 against the ground mesh's
    // ~85 000. Duplicating them is cheap, and the single mesh is practical.
    let mut mats: std::collections::HashMap<(u64, u64), std::collections::BTreeSet<u8>> =
        Default::default();
    for f in &a.faces {
        for p in f.shape.iter().flatten() {
            mats.entry((p[0].to_bits(), p[1].to_bits()))
                .or_default()
                .insert(f.material as u8);
        }
    }
    let split_vertices = mats.values().filter(|m| m.len() > 1).count();
    let spanned = a.faces.iter().filter(|f| f.spanned).count();
    let gallery = a.faces.iter().filter(|f| f.gallery).count();
    let walk_m2 = |near: bool| -> f64 {
        a.of(Material::Pavement)
            .filter(|f| f.near == near)
            .map(|f| poly::area(std::slice::from_ref(&f.shape)))
            .sum::<f64>()
            + 0.0
    };
    let (near, far) = (walk_m2(true), walk_m2(false));
    Summary::new()
        .with("cuts", rings)
        .with("faces", a.faces.len())
        .with("vertices", seen.len())
        .with("edges", edges.len())
        // Segments carried by one face away from the rect's border: a
        // subdivision that is not one, and no edge rule can be written over
        // it. Must read 0.
        .with("dangling", dangling)
        // Vertices where two materials meet, which a single mesh must carry
        // twice: the numerator of the one-height-per-vertex problem (§3.2
        // against §3.3). The denominator is the *mesh's* vertex count, which
        // this step does not have — see the note above.
        .with("split_vertices", split_vertices)
        .with_m2("ground_m2", m2(Material::Ground))
        .with_m2("carriageway_m2", m2(Material::Carriageway))
        .with_m2("pavement_m2", m2(Material::Pavement))
        .with_m2("ballast_m2", m2(Material::Ballast))
        .with("spanned", spanned)
        // The second layer: deck paving over another sheet's ground, which
        // the partition gave to the ground.
        .with("decks", a.decks.len())
        .with_m2("deck_m2", a.decks.iter().map(|f| poly::area(std::slice::from_ref(&f.shape))).sum::<f64>() + 0.0)
        .with("gallery", gallery)
        // How the walk divided at the room's reach — the near band the bench
        // lifts against the far part it drapes. `mesh::walk_split` is the
        // same fact in vertices; this is it in area, and a run where either
        // is zero is a run where the cut found nothing.
        .with_m2("walk_near_m2", near)
        .with_m2("walk_far_m2", far)
        .with("closure", format!("{closure:.3}"))
        .with_share("unshared", lone, seen.len())
        .with("unprobed", format!("{unprobed} ({unprobed_m2:.3} m2)"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;

    fn arranged(ground: &str, net: &str, steps: Vec<Step>) -> (Arrangement, Summary) {
        let (w, s) = built(ground, net, None, 10.0, &steps);
        (w.arrangement.expect("the arrangement step ran"), s.last())
    }

    /// **The faces partition the rect**, and every vertex off its edge is
    /// shared. The two properties the whole step is for, on a junction.
    #[test]
    fn the_faces_are_one_subdivision_of_the_rect() {
        let (a, s) = arranged("flat?h=400", "net:cross", upto(Step::Arrangement));
        assert!(a.faces.len() > 1, "the paving cuts the rect: {s}");
        assert_eq!(s.num("unshared"), 0.0, "a boundary was built twice: {s}");
        assert!(s.num("closure") <= 1.0, "not a partition: {s}");
        assert_eq!(s.num("unprobed"), 0.0, "a face could not be tagged: {s}");
    }

    /// The materials are the surface's own, area for area: the tagging adds
    /// nothing and loses nothing.
    #[test]
    fn a_tagged_face_is_the_material_the_surface_laid() {
        let (w, _) = built("flat?h=400", "net:sidewalk?d=6", None, 10.0, &upto(Step::Arrangement));
        let surface = &w.room.as_ref().expect("the room step ran").surface;
        let a = w.arrangement.as_ref().expect("the arrangement step ran");
        let rect = &w.extent.rect;
        let outer = vec![poly::rect(rect.x0, rect.y0, rect.x1, rect.y1)];
        for (material, laid) in [
            (Material::Carriageway, &surface.carriageway),
            (Material::Pavement, &surface.walk),
        ] {
            // Against the paving *clipped to the rect*: a ribbon may run off
            // the world, and the arrangement only ever covers the rect.
            let want = poly::area(&poly::intersect(&outer, laid));
            let got: f64 = a.of(material).map(|f| poly::area(std::slice::from_ref(&f.shape))).sum();
            assert!((got - want).abs() < 1e-3, "{}: {got} vs {want}", material.name());
        }
    }

    /// **A deck's paving is a face of its own, and it is not in the hole.**
    /// Invariant I3: a viaduct flies over ground that is still there.
    #[test]
    fn paving_over_a_span_is_not_part_of_the_hole() {
        let (a, s) = arranged(
            "gorge?depth=30&width=40",
            "net:straight?span=0.35,0.65&kind=bridge",
            upto(Step::Arrangement),
        );
        assert!(s.num("spanned") > 0.0, "the deck's paving is a face: {s}");
        let hole = a.hole();
        for f in a.faces.iter().filter(|f| f.spanned) {
            let probe = poly::inside(&f.shape).expect("a span face has an inside");
            assert!(!poly::contains(&hole, probe), "a deck cut the ground: {s}");
        }
    }

    /// **A deck over another road does not take that road's ground.**
    ///
    /// The partition has one face per point, and where an overpass's deck
    /// lies over the street beneath it in plan that face used to go to
    /// whichever sheet the index named first: the street lost its paving
    /// under the deck — 30 m² of the specimen's 2 200 — and at the Viaduc de
    /// Chillon the roads below showed bare terrain for the width of the
    /// viaduct. The street keeps its face; the deck's paving is a second
    /// layer. So every sheet's paving is in the arrangement area for area,
    /// and the street's is in the *partition*, where it cuts the ground.
    #[test]
    fn a_deck_over_a_road_leaves_the_road_its_ground() {
        // `len=201`: at 200 m both ways have a station at the crossing
        // point itself, `crossing::Net` joins them there by position, and
        // the floor lifts the street with the deck — two decks, no ground.
        let (w, all) = built("flat?h=400", "net:overpass?len=201", None, 10.0, &upto(Step::Arrangement));
        let s = all.last();
        let a = w.arrangement.as_ref().expect("built");
        let sheets = w.sheets.as_ref().expect("built");
        let rect = &w.extent.rect;
        let outer = vec![poly::rect(rect.x0, rect.y0, rect.x1, rect.y1)];
        let area = |f: &Face| poly::area(std::slice::from_ref(&f.shape));
        for (i, sheet) in sheets.sheets.iter().enumerate() {
            let want = poly::area(&poly::intersect(&outer, &sheet.shapes));
            let got: f64 = a.faces.iter().chain(&a.decks).filter(|f| f.sheet == Some(i)).map(area).sum();
            assert!((got - want).abs() < 1e-3, "sheet {i}: {got} of {want} m2 in the arrangement: {s}");
        }
        assert!(!a.decks.is_empty(), "the overpass lies over the street: {s}");
        // The street under the deck is on the ground: in the partition, and
        // cutting the terrain's hole.
        let hole = a.hole();
        for d in &a.decks {
            assert!(d.spanned, "a deck face is over a span");
            let probe = poly::inside(&d.shape).expect("a deck face has an inside");
            assert!(poly::contains(&hole, probe), "the street under the deck does not cut the ground: {s}");
        }
    }

    /// **One mesh: the materials share their vertices by index.**
    ///
    /// The property §3.2 needs and the separate `Tri`s never had. A kerb
    /// vertex is one entry in one array, reached from a carriageway triangle
    /// and from a pavement triangle alike — so a solve over this graph has
    /// one unknown there, not two that must find each other at the kernel's
    /// grid afterwards.
    #[test]
    fn one_mesh_shares_its_vertices_between_materials() {
        let (w, _) = built("flat?h=400", "net:sidewalk?d=6", None, 10.0, &upto(Step::Arrangement));
        let a = w.arrangement.as_ref().expect("built");
        let terrain = w.terrain.as_ref().expect("built");
        let m = a.mesh(&terrain.grid, &|p| crate::terrain::height_at(terrain, p[0], p[1]));
        assert!(!m.tri.indices.is_empty(), "the rect meshed to nothing");
        assert_eq!(m.of_face.len(), m.tri.indices.len() / 3, "one tag per triangle");

        // The materials each triangle at a vertex belongs to.
        let mut at: std::collections::HashMap<u32, std::collections::BTreeSet<&str>> =
            Default::default();
        for (t, face) in m.tri.indices.chunks_exact(3).zip(&m.of_face) {
            let name = a.faces[*face as usize].material.name();
            for &v in t {
                at.entry(v).or_default().insert(name);
            }
        }
        let shared = at.values().filter(|s| s.len() > 1).count();
        assert!(shared > 0, "no vertex is reached from two materials: not one mesh");
        // And the tags are real: every material of the arrangement that has
        // a face is reachable through the triangles.
        let meshed: std::collections::BTreeSet<&str> =
            m.of_face.iter().map(|f| a.faces[*f as usize].material.name()).collect();
        for want in ["ground", "carriageway", "pavement"] {
            assert!(meshed.contains(want), "{want} has no triangles: {meshed:?}");
        }
    }

    /// The arrangement is a function of the world: two runs agree.
    #[test]
    fn the_arrangement_is_a_function_of_the_world() {
        let steps = upto(Step::Arrangement);
        let (w, _) = built("flat?h=400", "net:tee?d=8&hook=5", None, 10.0, &steps);
        let a = w.arrangement.as_ref().expect("built");
        let roads = w.roads.as_ref().expect("built");
        let (again, _) = run(
            &w.extent,
            &roads.spans,
            w.profile.as_ref().expect("built"),
            &w.room.as_ref().expect("built").surface,
            w.sheets.as_ref().expect("built"),
        );
        assert_eq!(a.faces.len(), again.faces.len());
        for (x, y) in a.faces.iter().zip(&again.faces) {
            assert_eq!(x.shape, y.shape);
            assert_eq!(x.material, y.material);
            assert_eq!(x.spanned, y.spanned);
        }
    }
}
