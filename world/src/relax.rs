//! The relax: one height per vertex, from one linear system.
//!
//! §3.2 of `data/plans/one-ground-2026-09-16.md` wrote the ground as a
//! minimisation, `Σ w·(z − dem)² + Σ ‖∇z − ∇dem‖²`, which in terms of the
//! residual `e = z − dem` is exactly
//!
//! ```text
//! E(e) = eᵀ (W + L) e
//! ```
//!
//! with `L` a graph Laplacian and `W` the diagonal of weights. So the system
//! to solve is `(W + L) e = 0` over the free vertices, with `e` held at the
//! pinned ones, and the height is `dem + e`. **Away from every pin the answer
//! is exactly the DEM** (`e = 0` solves it), and between two pins it is the
//! smoothest surface that meets both.
//!
//! **It is not the ground's rule, and that was measured.** Wired as the whole
//! ground it sets a *decay rate* rather than a slope, so a 10 m fill falls
//! away at whatever the weight makes of it (49 m off the batter on the loop
//! box); [`crate::ground::Ground`] keeps the batter a slope. What it is for
//! is the one place the batter has no answer: two pins that disagree facing
//! each other across a strip narrower than their batters, where the nearest
//! segment's rule can only hand over from one to the other in a fold
//! ([`crate::ground::Ground::ramp`]). There every vertex around the fold is
//! held at the batter's own value except the fold's, `w` is zero, and the
//! relax is the harmonic surface between them: a ramp, not a decay.
//!
//! The Laplacian is **weighted**: an edge's weight is what the triangles on
//! either side of it make of it (the cotangent weights, for a triangle mesh
//! the exact discrete `∫ ‖∇e‖²`), so a mesh of slivers and lattice cells
//! solves for the same surface a regular grid would. `W + L` restricted to
//! the free vertices is symmetric and, wherever every free component touches
//! a pin, positive definite, so conjugate gradients converge.

/// One vertex's role in the solve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pin {
    /// Held at this residual off the DEM: a pin, or a vertex whose value a
    /// rule outside the solve already gives.
    At(f64),
    /// Solved for.
    Free,
}

/// Slack on the residual, in metres, at which conjugate gradients stop.
///
/// Far under anything geometric on purpose: it is not a tolerance on the
/// ground but the floor under which the *shape* of the answer stops being the
/// energy's and starts being the solver's. `relax_decays_at_the_rate_the_weight_sets`
/// reads the decay ratio down to a residual of a micron, and at 1e-9 the
/// ratio there was already the stopping rule rather than the decay.
const EPS_M: f64 = 1e-14;

/// Most iterations, whatever the residual: a guard, not a budget. CG on this
/// system converges in far fewer, and a run that reaches this is a graph that
/// is not what it should be.
const MAX_ITERS: usize = 10_000;

/// Solves `(W + L) e = 0` for the free vertices of a graph of `n` vertices
/// joined by `edges` (each with its weight), holding `e` at the pinned ones.
///
/// Returns the residual per vertex — add the DEM to get the height. An
/// unpinned component of the graph reads exactly zero, which is the DEM, and
/// is the right answer for ground no feature touches.
pub fn relax(n: usize, edges: &[(u32, u32, f64)], pin: &[Pin], w: f64) -> Vec<f64> {
    assert_eq!(pin.len(), n, "one pin per vertex");
    // The unknowns are the free vertices; the pinned ones move to the
    // right-hand side. `b = -(W + L) e_pinned`, restricted to free rows.
    let mut e: Vec<f64> = pin
        .iter()
        .map(|p| match p {
            Pin::At(v) => *v,
            Pin::Free => 0.0,
        })
        .collect();
    let free = |i: usize| pin[i] == Pin::Free;

    // r = b - A x, with x = 0 over the free vertices: the pinned values are
    // already in `e`, so one application of A to `e` and a negation is it.
    let apply = |v: &[f64], out: &mut Vec<f64>| {
        out.clear();
        out.extend(v.iter().enumerate().map(|(i, x)| if free(i) { w * x } else { 0.0 }));
        for &(i, j, k) in edges {
            let (i, j) = (i as usize, j as usize);
            let d = k * (v[i] - v[j]);
            if free(i) {
                out[i] += d;
            }
            if free(j) {
                out[j] -= d;
            }
        }
    };

    let mut r = Vec::new();
    apply(&e, &mut r);
    for (i, x) in r.iter_mut().enumerate() {
        *x = if free(i) { -*x } else { 0.0 };
    }
    let mut p = r.clone();
    let mut rr: f64 = r.iter().map(|x| x * x).sum();
    if rr.sqrt() <= EPS_M {
        return e;
    }
    let mut ap = Vec::new();
    for _ in 0..MAX_ITERS {
        apply(&p, &mut ap);
        let pap: f64 = p.iter().zip(&ap).map(|(a, b)| a * b).sum();
        if !(pap > 0.0) {
            // Singular: an unpinned component with `w = 0` has no unique
            // answer. Stop rather than divide by nothing.
            break;
        }
        let alpha = rr / pap;
        for i in 0..n {
            if free(i) {
                e[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }
        }
        let next: f64 = r.iter().map(|x| x * x).sum();
        if next.sqrt() <= EPS_M {
            break;
        }
        let beta = next / rr;
        rr = next;
        for i in 0..n {
            p[i] = r[i] + beta * p[i];
        }
    }
    e
}

#[cfg(test)]
mod tests {
    use super::*;

    /// How strongly a free vertex is pulled back to the DEM, per unit of edge
    /// coupling.
    ///
    /// It sets the decay alone, and the relation is exact on a uniform chain: the
    /// residual falls by `r` per vertex where `r` is the decaying root of
    /// `r² − (2 + w)r + 1 = 0`. At `w = 1` that is `(3 − √5)/2 ≈ 0.382` — an
    /// earthwork three or four cells wide. Smaller `w` reaches further.
    ///
    /// The ground's ramp runs at `w = 0`, held at every edge of its region,
    /// so it never reads this; the checks here keep the damped form honest
    /// for whatever wants a decay rather than a ramp.
    const WEIGHT: f64 = 1.0;

    /// A chain of `n` vertices, `0—1—2—…`.
    fn chain(n: usize) -> Vec<(u32, u32, f64)> {
        (0..n as u32 - 1).map(|i| (i, i + 1, 1.0)).collect()
    }

    /// **The decay is the weight's, exactly.** On a uniform chain the
    /// interior equation is `w·e_i + (2e_i − e_{i−1} − e_{i+1}) = 0`, whose
    /// decaying solution falls by the root of `r² − (2 + w)r + 1 = 0` per
    /// vertex. That is the whole of [`WEIGHT`]'s meaning, and it is checked
    /// rather than described.
    #[test]
    fn relax_decays_at_the_rate_the_weight_sets() {
        for w in [0.25, 1.0, 4.0] {
            let n = 60;
            let mut pin = vec![Pin::Free; n];
            pin[0] = Pin::At(1.0);
            let e = relax(n, &chain(n), &pin, w);
            let r = ((2.0 + w) - ((2.0 + w) * (2.0 + w) - 4.0).sqrt()) / 2.0;
            // Away from the near end, and only while the residual is well
            // above [`EPS_M`]: past that the ratio is the solver's stopping
            // tolerance rather than the decay, and at `w = 4` the chain is
            // under 1e-7 by the tenth vertex.
            let mut checked = 0;
            for i in 3..n - 3 {
                if e[i] < 1e-6 {
                    break;
                }
                // The ratio can be no truer than the solve is: an absolute
                // slack of [`EPS_M`] on a residual of `e[i]` is that much
                // relative slack on the quotient.
                let slack = 1e-9 + EPS_M / e[i];
                let got = e[i + 1] / e[i];
                assert!((got - r).abs() < slack, "w={w} at {i}: {got} vs {r}");
                checked += 1;
            }
            assert!(checked >= 3, "w={w}: only {checked} vertices carried the decay");
            assert!((e[0] - 1.0).abs() < 1e-12, "the pin moved");
        }
    }

    /// **Ground no feature touches is the DEM, to the bit.** The solve does
    /// not need telling where to stop: `e = 0` is the exact answer away from
    /// every pin, so there is no taper, no field limit, and no distance at
    /// which a rule hands over to another.
    #[test]
    fn a_vertex_no_pin_reaches_is_the_terrain_exactly() {
        let n = 400;
        let mut pin = vec![Pin::Free; n];
        pin[0] = Pin::At(3.0);
        let e = relax(n, &chain(n), &pin, WEIGHT);
        assert_eq!(e[n - 1], 0.0, "the far end is not the DEM");
        // And nothing is pulled the wrong way: the residual is monotone.
        for i in 1..n - 1 {
            assert!(e[i] >= e[i + 1] - 1e-12, "not monotone at {i}");
            assert!(e[i] >= 0.0);
        }
    }

    /// **Two pins are interpolated, not fought over.** A road each side of a
    /// strip of ground, at different heights: the field between them is
    /// smooth and lies between the two, which is the junction blend and the
    /// batter arriving from the energy rather than from a case.
    #[test]
    fn the_field_between_two_pins_is_smooth_and_between_them() {
        let n = 21;
        let mut pin = vec![Pin::Free; n];
        pin[0] = Pin::At(2.0);
        pin[n - 1] = Pin::At(-1.0);
        let e = relax(n, &chain(n), &pin, 0.0);
        // With no pull to the DEM the answer is the harmonic one: a straight
        // ramp between the pins.
        for i in 0..n {
            let want = 2.0 + (-1.0 - 2.0) * (i as f64) / (n as f64 - 1.0);
            assert!((e[i] - want).abs() < 1e-6, "at {i}: {} vs {want}", e[i]);
        }
    }

    /// A grid, so the solve is exercised on the topology it will actually
    /// meet: symmetric pins give a symmetric field, which no case-function
    /// with a nearest-axis rule managed at a junction.
    #[test]
    fn a_symmetric_pin_on_a_grid_gives_a_symmetric_field() {
        let (w, h) = (15usize, 15usize);
        let id = |x: usize, y: usize| (y * w + x) as u32;
        let mut edges = Vec::new();
        for y in 0..h {
            for x in 0..w {
                if x + 1 < w {
                    edges.push((id(x, y), id(x + 1, y), 1.0));
                }
                if y + 1 < h {
                    edges.push((id(x, y), id(x, y + 1), 1.0));
                }
            }
        }
        let n = w * h;
        let mut pin = vec![Pin::Free; n];
        pin[id(7, 7) as usize] = Pin::At(1.0);
        let e = relax(n, &edges, &pin, WEIGHT);
        for y in 0..h {
            for x in 0..w {
                let (mx, my) = (w - 1 - x, h - 1 - y);
                let (a, b) = (e[id(x, y) as usize], e[id(mx, my) as usize]);
                assert!((a - b).abs() < 1e-9, "({x},{y}) {a} vs ({mx},{my}) {b}");
            }
        }
        assert!(e[id(7, 7) as usize] > e[id(7, 8) as usize], "the pin is the peak");
    }
}
