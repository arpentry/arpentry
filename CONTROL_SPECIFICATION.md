# Map Control — Implementation Specification

This document translates the user-facing behaviors described in `CONTROL.md` into the fixed-camera / moving-globe architecture of the Arpentry viewer (`VIEWER.md`).

---

## 1. Architecture Recap

The camera sits at the origin looking along **−Z**. Navigation modifies five parameters that define a **globe transform** — how the globe is positioned and rotated relative to the fixed camera:

| Parameter | Symbol | Description | Range |
|-----------|--------|-------------|-------|
| Longitude | `lon` | Interest point longitude (radians) | −π to +π |
| Latitude | `lat` | Interest point latitude (radians) | −89° to +89° |
| Altitude | `alt` | Distance from interest point to camera (meters) | 500 m to 30,000 km |
| Tilt | `tilt` | Angle from nadir; 0 = looking straight down | 0 to 60° |
| Bearing | `bearing` | Clockwise rotation from north | 0 to 360° |

These produce a per-tile model matrix:

```
M_tile = mat4(R_tilt * R_globe, R_tilt * R_globe * delta + (0, 0, -alt))
```

where `R_globe` rotates the globe so the interest point faces the camera, and `R_tilt` applies tilt and bearing. The view matrix is identity. The projection is a standard perspective with 45° vertical FOV.

From the user's perspective, changing `lon`/`lat` looks like panning the map, changing `alt` looks like zooming, and changing `tilt`/`bearing` looks like orbiting. This document specifies exactly how input events drive these parameter changes.

---

## 2. Pan — Moving the Interest Point

### 2.1 Behavior

Panning slides the map under the cursor. The user grabs a point on the globe surface and drags it; the globe follows so that the grabbed point stays under the cursor throughout the drag.

### 2.2 Algorithm: Ray-Cast Pan

On **mouse-down**: cast a ray from the cursor through the camera and intersect it with the WGS84 ellipsoid. Store the hit point as the **anchor** in geodetic coordinates `(anchor_lon, anchor_lat)`.

On **mouse-move** (while dragging): cast a ray from the current cursor position. Intersect with the ellipsoid. Convert the hit to geodetic `(cursor_lon, cursor_lat)`. Adjust the interest point:

```
lon += anchor_lon − cursor_lon
lat += anchor_lat − cursor_lat
```

This keeps the anchor point glued to the cursor. Because moving `lon`/`lat` changes `R_globe` which changes where the ray lands, apply the correction iteratively (2–3 iterations converge to sub-pixel precision — the same technique used by `arpt_camera_zoom_at`).

### 2.3 Fallback: Linear Pan

If the cursor ray misses the ellipsoid (rare — only when tilt is high and cursor is above the horizon), fall back to the existing linear approximation:

```
rad_per_px = 2 * alt * tan(fov/2) / (vp_height * WGS84_A)
```

Rotate the pixel delta `(dx, dy)` by `bearing` before applying to `lon`/`lat`, with a `1/cos(lat)` correction on longitude.

### 2.4 Why Ray-Cast

The linear model introduces drift when tilted: the cursor separates from the grab point because the meter-per-pixel ratio varies across the screen. Ray-casting eliminates this — the user can pan with a 50° tilt and the grabbed point stays precisely under the finger. This is the "ground-plane panning" behavior described in CONTROL.md, adapted to a curved surface.

---

## 3. Zoom — Changing Altitude

### 3.1 Behavior

Zoom changes the distance between camera and globe. Scroll zooms centered on the cursor — the point under the cursor stays fixed on screen.

### 3.2 Algorithm: Cursor-Anchored Zoom

1. Cast ray from cursor, intersect ellipsoid → anchor point `P(lon, lat)`.
2. Multiply `alt` by the zoom factor.
3. Iteratively adjust `(lon, lat)` so that `P` still projects to the same screen position (2–3 iterations, same as ray-cast pan).

This is already implemented in `arpt_camera_zoom_at`.

### 3.3 Zoom Factor

| Source | Factor |
|--------|--------|
| Scroll wheel (discrete) | `0.95^yoff` per scroll tick |
| Trackpad pinch | `0.95^delta` (continuous delta from gesture) |
| Keyboard +/− | `0.95` / `1/0.95` per keypress |
| Double-click / double-tap | Fly-to animation: target `alt = alt * 0.5`, centered on click point |

### 3.4 Limits

Altitude is clamped to `[500 m, 30,000 km]`. Near this range, zoom factor still applies but the result is clamped. No bounce or spring — just a hard stop.

---

## 4. Rotate — Tilt and Bearing

### 4.1 Behavior

Rotation changes the viewing angle around the interest point. Horizontal drag adjusts bearing (compass rotation); vertical drag adjusts tilt (pitch from nadir).

### 4.2 Algorithm

From pixel deltas `(dx, dy)`:

```
bearing += dx * BEARING_SENSITIVITY
tilt    -= dy * TILT_SENSITIVITY
```

Sensitivity constants: `TILT_SENSITIVITY = 0.005 rad/px`, `BEARING_SENSITIVITY = 0.005 rad/px`.

Tilt is clamped to `[0, 60°]`. Bearing wraps modulo 360°.

### 4.3 Note on Interest Point

During rotation the interest point `(lon, lat)` stays fixed. The globe pivots in place, which from the user's perspective looks like the camera orbiting around the interest point.

---

## 5. Input Bindings

### 5.1 Mouse

| Input | Action |
|-------|--------|
| **Left-drag** | Pan (ray-cast) |
| **Right-drag** | Rotate (tilt + bearing) |
| **Scroll wheel** | Zoom at cursor |
| **Left-drag + Ctrl/Meta/Shift** | Rotate (modifier overrides pan) |
| **Right-drag + Ctrl/Meta/Shift** | Pan (modifier overrides rotate) |
| **Double-click** | Fly-to: zoom in one level centered on click |

The modifier swap ensures both actions are reachable from either button. Pan is on the primary button because map interaction is predominantly panning.

### 5.2 Keyboard

| Input | Action |
|-------|--------|
| Arrow keys | Pan by fixed step (100 px equivalent) |
| Shift + Arrow keys | Rotate: ↑↓ = tilt ±10°, ←→ = bearing ±15° |
| + / = | Zoom in (factor 0.95) |
| − / _ | Zoom out (factor 1/0.95) |

### 5.3 Touch (Emscripten target)

| Input | Action |
|-------|--------|
| One-finger drag | Pan (ray-cast) |
| Two-finger pinch/spread | Zoom at midpoint |
| Two-finger rotate | Bearing change |
| Two-finger vertical drag | Tilt change |

Two-finger gestures are decomposed simultaneously: the distance delta drives zoom, the angle delta drives bearing, and the midpoint translation drives pan. All three are applied each frame.

---

## 6. Inertia and Damping

### 6.1 Velocity Channels

On drag release, the last frame's delta is captured as a velocity:

| Channel | Unit | Source |
|---------|------|--------|
| `vel_pan_x`, `vel_pan_y` | px/sec | Left-drag release |
| `vel_tilt`, `vel_bearing` | rad/sec | Right-drag release |

### 6.2 Decay

Each frame, velocities decay by an exponential factor:

```
decay = (1 − DAMPING)^(dt * 60)
```

where `DAMPING = 0.12` and `dt` is the frame delta in seconds. This is frame-rate-independent: at 60 fps each frame decays by 12%; at 30 fps each frame decays by ~23%, giving the same overall curve.

Velocities below `1e-6` snap to zero to avoid perpetual micro-movement.

### 6.3 Integration

Pan velocity is integrated through `arpt_camera_pan` (currently linear; should use the same ray-cast method as interactive pan for consistency, but linear is acceptable for inertia since precision matters less when the user isn't actively dragging). Tilt/bearing velocity is integrated through `arpt_camera_tilt_bearing`.

---

## 7. Fly-To Animation

Double-click or programmatic navigation triggers a smooth fly-to:

1. Record start state `(lon₀, lat₀, alt₀)`.
2. Compute target: the double-clicked point becomes the new interest point at half the current altitude.
3. Interpolate with ease-in-out cubic over 0.4 seconds:
   ```
   t_ease = t < 0.5 ? 4t³ : 1 − (−2t + 2)³ / 2
   lon = lon₀ + wrap(lon₁ − lon₀) * t_ease
   lat = lerp(lat₀, lat₁, t_ease)
   alt = lerp(alt₀, alt₁, t_ease)
   ```
4. Longitude delta is wrapped to `[−π, π]` to take the short path around the antimeridian.

During animation, all velocity channels are zeroed and interactive input is still accepted (which cancels the animation implicitly by setting new velocities).

---

## 8. Screen-to-Globe Ray Casting

All the above operations depend on a shared primitive: casting a ray from a screen pixel through the fixed camera into ECEF space, then intersecting with the WGS84 ellipsoid.

### 8.1 Screen → Camera-Space Ray

```
ndc_x = 2 * sx / vp_width − 1
ndc_y = 1 − 2 * sy / vp_height
dir_cam = normalize(ndc_x * half_h * aspect, ndc_y * half_h, −1)
```

where `half_h = tan(fov/2)`.

### 8.2 Camera-Space → ECEF

The combined rotation `R = R_tilt * R_globe` transforms ECEF to camera space. Its inverse (= transpose, since R is orthogonal) transforms camera-space directions back to ECEF:

```
dir_ecef = R^T * dir_cam
```

The camera position in ECEF:

```
origin_ecef = interest_ecef + R^T * (0, 0, alt)
```

### 8.3 Ellipsoid Intersection

Standard quadratic solution for the ray `P = origin + t * dir` against the WGS84 ellipsoid `(x/a)² + (y/a)² + (z/b)² = 1`. Returns the nearest positive `t`, or miss.

---

## 9. Implementation Mapping

The existing `camera.c` API already covers most of this specification. The table below maps specification sections to API functions and notes what changes:

| Spec Section | API Function | Status |
|-------------|-------------|--------|
| §2 Pan (ray-cast) | `arpt_camera_pan` | **Change**: replace linear model with ray-cast anchor tracking |
| §3 Zoom | `arpt_camera_zoom_at` | Exists — no change needed |
| §4 Rotate | `arpt_camera_tilt_bearing` | Exists — no change needed |
| §5.1 Modifier swap | `main.c` callbacks | **Add**: check modifier keys in mouse-button callback |
| §5.3 Touch | Emscripten event handlers | **Add**: new for Emscripten build |
| §6 Inertia | `arpt_camera_update` | Exists — no change needed |
| §7 Fly-to | `arpt_camera_fly_to_screen` | Exists — no change needed |
| §8 Ray-cast | `arpt_camera_screen_to_ray` | Exists — no change needed |

### 9.1 Pan API Change

The current `arpt_camera_pan(cam, dx, dy)` takes pixel deltas and applies a linear sensitivity. For ray-cast pan, the API changes to anchor-based tracking:

```c
/** Begin a pan gesture — store the anchor point under the cursor. */
void arpt_camera_pan_begin(arpt_camera *cam, double screen_x, double screen_y);

/** Continue a pan gesture — adjust interest point so anchor stays under cursor. */
void arpt_camera_pan_move(arpt_camera *cam, double screen_x, double screen_y);
```

The old `arpt_camera_pan(dx, dy)` is kept for keyboard panning and inertia integration, where the linear model is sufficient.

### 9.2 Modifier Key Swap

In `on_mouse_button` and `on_cursor_pos`, check for `GLFW_MOD_SHIFT | GLFW_MOD_CONTROL | GLFW_MOD_SUPER`. When any modifier is held:
- Left-drag → rotate (instead of pan)
- Right-drag → pan (instead of rotate)
