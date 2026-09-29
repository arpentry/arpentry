//! Probe: the reference along every solving way whose id starts with one of
//! the prefixes — at each station the ground, what the closing and the
//! opening made of it, the conditioned height, and the blind and spanned
//! masks.
//!
//!   cargo run --release --example ref_probe -- ZONE w,s,e,n ID_PREFIX...

mod common;

use arpentry_world::reference;
use arpentry_world::step::Step;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let want: Vec<&str> = a[2..].iter().map(|s| s.as_str()).collect();
    let world = common::world(std::path::Path::new(&a[0]), &a[1], Step::Reference);

    let reference = world.reference.as_ref().unwrap();
    for ax in &reference.axes {
        let w = &reference.ways[ax.way];
        if !want.iter().any(|p| w.id.starts_with(p)) {
            continue;
        }
        let closed = reference::close_notches(&ax.s, &ax.ground);
        let opened = reference::open_bumps(&ax.s, &closed);
        println!(
            "== {} [{}] len={:.1} spans={:?}",
            w.id,
            w.class,
            ax.len(),
            w.spans.iter().map(|s| (s.a0, s.a1, s.kind)).collect::<Vec<_>>()
        );
        println!(
            "  refused_notch={:?} refused_crest={:?}",
            ax.refused_notch, ax.refused_crest
        );
        println!(
            "{:>9} {:>9} {:>9} {:>9} {:>9} {:>8} {:>8} {:>6} {:>8}",
            "s", "ground", "closed", "opened", "ref(h)", "fill", "shave", "blind", "spanned"
        );
        for k in 0..ax.s.len() {
            println!(
                "{:>9.3} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>+8.2} {:>+8.2} {:>6} {:>8}",
                ax.s[k],
                ax.ground[k],
                closed[k],
                opened[k],
                ax.h[k],
                closed[k] - ax.ground[k],
                opened[k] - closed[k],
                ax.blind[k],
                ax.spanned[k]
            );
        }
    }
}
