# Design Principles

## Complexity is the enemy

Complexity manifests as change amplification, cognitive load, and unknown unknowns. It accumulates incrementally — each shortcut adds a little.

## Deep modules

The best modules have simple interfaces and complex implementations. A deep module hides significant complexity behind a small API.

## Pull complexity downward

When there is a choice between making something harder for the implementer vs. the caller, push the complexity into the implementation.

## Define errors out of existence

Design APIs so that error conditions cannot arise — make operations idempotent, absorb edge cases internally. When errors are unavoidable, handle them through return values.

## Different layer, different abstraction

Each layer should provide a distinctly different abstraction. If two adjacent layers have similar interfaces, consider merging them.

## Information hiding

Each module should encapsulate design decisions. Concentrate shared knowledge behind one interface to prevent information leakage.

## Somewhat general-purpose

Design interfaces that naturally serve the current need while being usable in plausible adjacent situations — not tied to one use case, not over-engineered for hypothetical futures.

## Comments describe what is not obvious

Comments should explain *why*, not what. The code already says what.
