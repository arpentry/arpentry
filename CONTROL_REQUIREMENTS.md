# Map Control Specification

## 1. Overview

This document specifies the behavior of a **Map Control** — a camera interaction system designed for navigating 3D scenes from a top-down or oblique "map-like" perspective.

The fundamental metaphor is that the user is looking down at a surface (a map, terrain, game board, etc.) and wishes to **pan** across it, **zoom** into and out of it, and **rotate/tilt** the view to see it from different angles. Unlike a free-orbit camera (which revolves around a point in space), a Map Control treats panning as the primary action and rotation as secondary — this matches how people expect to interact with map-like content.

---

## 2. Conceptual Model

### 2.1 Camera and Target

The control manages a **camera** and a **target point** (also called the focus point or pivot). The camera always looks at the target. The relationship between camera and target is described in spherical coordinates:

- **Radius** (distance): How far the camera is from the target.
- **Polar angle** (phi / pitch): The vertical angle from the "up" axis. A polar angle of 0 means the camera is directly above the target looking straight down. A polar angle of π/2 means the camera is level with the target (horizon view).
- **Azimuthal angle** (theta / bearing): The horizontal rotation around the target, measured from a reference direction (typically north or the positive Z-axis).

When the user pans, the target moves across the ground plane and the camera follows. When the user zooms, the radius changes. When the user rotates, the azimuthal and/or polar angles change while the target stays fixed.

### 2.2 Up Vector

The control preserves a consistent "up" direction (by default, the +Y axis). The camera never rolls — the horizon always stays level. This is a core distinction from trackball-style controls that allow arbitrary camera orientation.

### 2.3 Ground-Plane Panning

When panning, the target moves along the **ground plane** (the plane orthogonal to the up vector passing through the current target point), not in screen space. This means that when the camera is tilted, dragging the mouse moves the view in a way that tracks the surface beneath the cursor, matching the intuition of grabbing and sliding a physical map.

This is the key behavioral difference from a standard orbit control, where panning defaults to screen-space movement (the target slides along a plane parallel to the camera's image plane). Map controls override this so that panning always moves along the world-horizontal plane.

---

## 3. Input Bindings

### 3.1 Mouse

| Input | Action |
|---|---|
| **Left-click + drag** | Pan (slide the map) |
| **Right-click + drag** | Rotate (orbit around the target) |
| **Middle-click + drag** | Dolly (move closer/farther along the look direction) |
| **Scroll wheel** | Zoom in/out (dolly) |
| **Left-click + Ctrl/Meta/Shift + drag** | Rotate (modifier overrides pan to rotate) |
| **Right-click + Ctrl/Meta/Shift + drag** | Pan (modifier overrides rotate to pan) |

The modifier key swap ensures that both pan and rotate are accessible from both buttons when needed. The design prioritizes pan on the primary (left) button because map interaction is predominantly panning.

### 3.2 Touch

| Input | Action |
|---|---|
| **One-finger drag** | Pan |
| **Two-finger pinch/spread** | Zoom (dolly) |
| **Two-finger rotate** | Rotate (orbit around the target) |
| **Two-finger drag (midpoint translation)** | This occurs simultaneously with pinch — the zoom and rotation are composited from the two-finger gesture |

When two fingers are active, the control simultaneously processes dolly (from the change in distance between the fingers) and rotation (from the change in angle between the fingers). The midpoint of the two fingers is also tracked to allow concurrent panning if desired.

### 3.3 Keyboard

| Input | Action |
|---|---|
| **Arrow keys** | Pan the view (up/down/left/right by a fixed pixel amount) |
| **Shift + Arrow keys** (or Ctrl/Meta + Arrow) | Rotate the view |
| **+ / =** | Zoom in (MapLibre-style; optional) |
| **- / \_** | Zoom out (MapLibre-style; optional) |

Arrow key panning moves the view by a configurable number of pixels per keypress (default: 7 pixels). Rotation by keyboard increments by a small angular step.

### 3.4 Double-Click / Double-Tap

Optionally, double-clicking or double-tapping zooms in by a fixed amount, centered on the cursor/tap location. This is standard in MapLibre but not in three.js MapControls.
