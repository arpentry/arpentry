# Design Principles

Key principles for this codebase, drawn from *A Philosophy of Software Design* by John Ousterhout.

## Complexity is the enemy

Complexity manifests as **change amplification** (one change requires edits in many places), **cognitive load** (how much you need to know to make a change), and **unknown unknowns** (not obvious what needs to change). Complexity is incremental — each shortcut adds a little, and it accumulates.

## Strategic programming

Working code is not enough. Every change should be an investment in the system's design, not just a fix for today's problem. Spend a little extra time to find a simpler structure rather than taking the fastest path.

## Deep modules

The best modules have **simple interfaces and complex implementations**. A deep module hides significant complexity behind a small API. Shallow modules — where the interface is nearly as complex as the implementation — add little value and increase the system's overall surface area.

## Pull complexity downward

When there is a choice between making something harder for the implementer vs. the caller, push the complexity into the implementation. It is better for a module to handle a hard case internally than to leak that complexity to every caller.

## Define errors out of existence

The best way to handle errors is to design APIs so that error conditions cannot arise. Instead of returning error codes that every caller must handle, design the function so it always succeeds (e.g., by making operations idempotent, or absorbing edge cases internally).

## Different layer, different abstraction

Each layer in a system should provide a distinctly different abstraction. If two adjacent layers have similar interfaces, it is a sign that the layering is not adding value — consider merging them or rethinking the boundary.

## Information hiding

Each module should encapsulate design decisions. If a piece of knowledge (a format detail, an algorithm, a constant) is used by many modules, it is **information leakage** — a change to that knowledge requires changes everywhere. Concentrate such knowledge behind one interface.

## Somewhat general-purpose

Design interfaces that are somewhat general-purpose — not tied to today's one use case, but not over-engineered for hypothetical futures. The sweet spot: an interface that naturally serves the current need while being usable in plausible adjacent situations without modification.

## Comments describe what is not obvious

Comments should explain **why**, not what. The code already says what. Good comments capture: the intent behind a design decision, non-obvious constraints, and things a reader could not deduce from the code alone.
