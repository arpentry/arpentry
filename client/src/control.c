#include "control.h"
#include <GLFW/glfw3.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/html5.h>
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define DAMPING             0.12
#define TILT_SENSITIVITY    0.005   /* rad/px */
#define BEARING_SENSITIVITY 0.005   /* rad/px */
#define KEY_PAN_PX          100.0   /* pixels per keypress */
#define KEY_TILT_DEG        10.0
#define KEY_BEARING_DEG     15.0
#define ZOOM_BASE           0.95
#define FLYTO_DURATION      0.4     /* seconds */
#define DBLCLICK_INTERVAL   0.3     /* seconds */
#define DBLCLICK_DIST       5.0     /* pixels */
#define VEL_EPSILON         1e-6

#define DEG2RAD(d) ((d) * M_PI / 180.0)

/* ── Drag mode ─────────────────────────────────────────────────────────── */

typedef enum {
    DRAG_IDLE,
    DRAG_PAN,
    DRAG_ROTATE,
} drag_mode_t;

/* ── Fly-to state ──────────────────────────────────────────────────────── */

typedef struct {
    bool active;
    double elapsed;
    double start_lon, start_lat, start_alt;
    double target_lon, target_lat, target_alt;
} flyto_t;

/* ── Main struct ───────────────────────────────────────────────────────── */

struct arpt_control {
    arpt_camera *cam;
    GLFWwindow *window;

    /* Drag state */
    drag_mode_t drag_mode;
    double last_sx, last_sy;
    double prev_sx, prev_sy;  /* for velocity capture: position two frames ago */

    /* Inertia velocities (per second) */
    double vel_pan_x, vel_pan_y;
    double vel_tilt, vel_bearing;

    /* Double-click detection */
    double last_click_time;
    double last_click_x, last_click_y;

    /* Fly-to animation */
    flyto_t flyto;

#ifdef __EMSCRIPTEN__
    /* Touch state */
    int touch_count;
    double touch0_x, touch0_y;
    double touch1_x, touch1_y;
    double pinch_dist;
    double pinch_angle;
#endif
};

/* ── Helpers ───────────────────────────────────────────────────────────── */

static inline double wrap_lon(double delta) {
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    return delta;
}

static inline double ease_in_out_cubic(double t) {
    return t < 0.5 ? 4.0 * t * t * t
                    : 1.0 - pow(-2.0 * t + 2.0, 3.0) / 2.0;
}

static inline bool has_modifier(int mods) {
    return (mods & (GLFW_MOD_CONTROL | GLFW_MOD_SUPER | GLFW_MOD_SHIFT)) != 0;
}

/** Cancel fly-to and zero velocities (any user input does this). */
static void cancel_animation(arpt_control *ctrl) {
    ctrl->flyto.active = false;
    ctrl->vel_pan_x = ctrl->vel_pan_y = 0.0;
    ctrl->vel_tilt = ctrl->vel_bearing = 0.0;
}

/* ── GLFW callbacks ────────────────────────────────────────────────────── */

static void on_mouse_button(GLFWwindow *window, int button, int action,
                             int mods) {
    arpt_control *ctrl = glfwGetWindowUserPointer(window);
    if (!ctrl) return;

    double sx, sy;
    glfwGetCursorPos(window, &sx, &sy);

    if (action == GLFW_PRESS) {
        cancel_animation(ctrl);

        bool swap = has_modifier(mods);
        bool is_left = (button == GLFW_MOUSE_BUTTON_LEFT);
        bool is_right = (button == GLFW_MOUSE_BUTTON_RIGHT);

        if ((is_left && !swap) || (is_right && swap)) {
            ctrl->drag_mode = DRAG_PAN;
            arpt_camera_pan_begin(ctrl->cam, sx, sy);
        } else if ((is_right && !swap) || (is_left && swap)) {
            ctrl->drag_mode = DRAG_ROTATE;
        }

        ctrl->last_sx = sx;
        ctrl->last_sy = sy;
        ctrl->prev_sx = sx;
        ctrl->prev_sy = sy;

        /* Double-click detection (left button only) */
        if (is_left && !swap) {
            double now = glfwGetTime();
            double dx = sx - ctrl->last_click_x;
            double dy = sy - ctrl->last_click_y;
            double dist = sqrt(dx * dx + dy * dy);

            if (now - ctrl->last_click_time < DBLCLICK_INTERVAL &&
                dist < DBLCLICK_DIST) {
                /* Trigger fly-to: zoom to half altitude centered on click */
                double click_lon, click_lat;
                if (arpt_camera_screen_to_geodetic(ctrl->cam, sx, sy,
                                                    &click_lon, &click_lat)) {
                    ctrl->flyto.active = true;
                    ctrl->flyto.elapsed = 0.0;
                    ctrl->flyto.start_lon = arpt_camera_lon(ctrl->cam);
                    ctrl->flyto.start_lat = arpt_camera_lat(ctrl->cam);
                    ctrl->flyto.start_alt = arpt_camera_altitude(ctrl->cam);
                    ctrl->flyto.target_lon = click_lon;
                    ctrl->flyto.target_lat = click_lat;
                    ctrl->flyto.target_alt = ctrl->flyto.start_alt * 0.5;
                }

                ctrl->drag_mode = DRAG_IDLE;
                ctrl->last_click_time = 0.0;
            } else {
                ctrl->last_click_time = now;
                ctrl->last_click_x = sx;
                ctrl->last_click_y = sy;
            }
        }
    } else if (action == GLFW_RELEASE) {
        /* Capture velocity from last frame's delta */
        if (ctrl->drag_mode == DRAG_PAN) {
            /* Use delta from prev to last (the last move step) as velocity.
               Scale to per-second assuming ~16ms frame. We use the actual
               time from the update loop, but approximate here. */
            ctrl->vel_pan_x = (ctrl->last_sx - ctrl->prev_sx) * 60.0;
            ctrl->vel_pan_y = (ctrl->last_sy - ctrl->prev_sy) * 60.0;
        } else if (ctrl->drag_mode == DRAG_ROTATE) {
            double dx = ctrl->last_sx - ctrl->prev_sx;
            double dy = ctrl->last_sy - ctrl->prev_sy;
            ctrl->vel_bearing = dx * BEARING_SENSITIVITY * 60.0;
            ctrl->vel_tilt = -dy * TILT_SENSITIVITY * 60.0;
        }
        ctrl->drag_mode = DRAG_IDLE;
    }
}

static void on_cursor_pos(GLFWwindow *window, double sx, double sy) {
    arpt_control *ctrl = glfwGetWindowUserPointer(window);
    if (!ctrl) return;

    if (ctrl->drag_mode == DRAG_PAN) {
        ctrl->prev_sx = ctrl->last_sx;
        ctrl->prev_sy = ctrl->last_sy;
        arpt_camera_pan_move(ctrl->cam, sx, sy);
        ctrl->last_sx = sx;
        ctrl->last_sy = sy;
    } else if (ctrl->drag_mode == DRAG_ROTATE) {
        double dx = sx - ctrl->last_sx;
        double dy = sy - ctrl->last_sy;
        ctrl->prev_sx = ctrl->last_sx;
        ctrl->prev_sy = ctrl->last_sy;
        arpt_camera_tilt_bearing(ctrl->cam, -dy * TILT_SENSITIVITY,
                                  dx * BEARING_SENSITIVITY);
        ctrl->last_sx = sx;
        ctrl->last_sy = sy;
    }
}

static void on_scroll(GLFWwindow *window, double xoffset, double yoffset) {
    (void)xoffset;
    arpt_control *ctrl = glfwGetWindowUserPointer(window);
    if (!ctrl) return;

    cancel_animation(ctrl);

    double sx, sy;
    glfwGetCursorPos(window, &sx, &sy);
    double factor = pow(ZOOM_BASE, yoffset);
    arpt_camera_zoom_at(ctrl->cam, sx, sy, factor);
}

static void on_key(GLFWwindow *window, int key, int scancode, int action,
                    int mods) {
    (void)scancode;
    arpt_control *ctrl = glfwGetWindowUserPointer(window);
    if (!ctrl) return;

    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    cancel_animation(ctrl);

    bool shift = (mods & GLFW_MOD_SHIFT) != 0;

    switch (key) {
    case GLFW_KEY_UP:
        if (shift) arpt_camera_tilt_bearing(ctrl->cam, DEG2RAD(KEY_TILT_DEG), 0.0);
        else arpt_camera_pan(ctrl->cam, 0.0, -KEY_PAN_PX);
        break;
    case GLFW_KEY_DOWN:
        if (shift) arpt_camera_tilt_bearing(ctrl->cam, DEG2RAD(-KEY_TILT_DEG), 0.0);
        else arpt_camera_pan(ctrl->cam, 0.0, KEY_PAN_PX);
        break;
    case GLFW_KEY_LEFT:
        if (shift) arpt_camera_tilt_bearing(ctrl->cam, 0.0, DEG2RAD(-KEY_BEARING_DEG));
        else arpt_camera_pan(ctrl->cam, -KEY_PAN_PX, 0.0);
        break;
    case GLFW_KEY_RIGHT:
        if (shift) arpt_camera_tilt_bearing(ctrl->cam, 0.0, DEG2RAD(KEY_BEARING_DEG));
        else arpt_camera_pan(ctrl->cam, KEY_PAN_PX, 0.0);
        break;
    case GLFW_KEY_EQUAL:   /* + / = */
    case GLFW_KEY_KP_ADD:
        arpt_camera_zoom_at(ctrl->cam,
            arpt_camera_vp_width(ctrl->cam) / 2.0,
            arpt_camera_vp_height(ctrl->cam) / 2.0,
            ZOOM_BASE);
        break;
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT:
        arpt_camera_zoom_at(ctrl->cam,
            arpt_camera_vp_width(ctrl->cam) / 2.0,
            arpt_camera_vp_height(ctrl->cam) / 2.0,
            1.0 / ZOOM_BASE);
        break;
    default:
        break;
    }
}

/* ── Emscripten touch handlers ─────────────────────────────────────────── */

#ifdef __EMSCRIPTEN__

static EM_BOOL on_touchstart(int type, const EmscriptenTouchEvent *e, void *ud) {
    (void)type;
    arpt_control *ctrl = ud;
    cancel_animation(ctrl);

    ctrl->touch_count = e->numTouches;
    if (e->numTouches >= 1) {
        ctrl->touch0_x = e->touches[0].targetX;
        ctrl->touch0_y = e->touches[0].targetY;
        if (e->numTouches == 1) {
            arpt_camera_pan_begin(ctrl->cam, ctrl->touch0_x, ctrl->touch0_y);
        }
    }
    if (e->numTouches >= 2) {
        ctrl->touch1_x = e->touches[1].targetX;
        ctrl->touch1_y = e->touches[1].targetY;
        double dx = ctrl->touch1_x - ctrl->touch0_x;
        double dy = ctrl->touch1_y - ctrl->touch0_y;
        ctrl->pinch_dist = sqrt(dx * dx + dy * dy);
        ctrl->pinch_angle = atan2(dy, dx);
    }
    return EM_TRUE;
}

static EM_BOOL on_touchmove(int type, const EmscriptenTouchEvent *e, void *ud) {
    (void)type;
    arpt_control *ctrl = ud;

    if (e->numTouches == 1) {
        /* One-finger: pan */
        double sx = e->touches[0].targetX;
        double sy = e->touches[0].targetY;
        arpt_camera_pan_move(ctrl->cam, sx, sy);
        ctrl->touch0_x = sx;
        ctrl->touch0_y = sy;
    } else if (e->numTouches >= 2) {
        double t0x = e->touches[0].targetX, t0y = e->touches[0].targetY;
        double t1x = e->touches[1].targetX, t1y = e->touches[1].targetY;

        /* Midpoint delta → pan */
        double old_mx = (ctrl->touch0_x + ctrl->touch1_x) * 0.5;
        double old_my = (ctrl->touch0_y + ctrl->touch1_y) * 0.5;
        double new_mx = (t0x + t1x) * 0.5;
        double new_my = (t0y + t1y) * 0.5;
        arpt_camera_pan(ctrl->cam, new_mx - old_mx, new_my - old_my);

        /* Distance delta → zoom */
        double dx = t1x - t0x, dy = t1y - t0y;
        double new_dist = sqrt(dx * dx + dy * dy);
        if (ctrl->pinch_dist > 1.0 && new_dist > 1.0) {
            double factor = ctrl->pinch_dist / new_dist;
            arpt_camera_zoom_at(ctrl->cam, new_mx, new_my, factor);
        }
        ctrl->pinch_dist = new_dist;

        /* Angle delta → bearing */
        double new_angle = atan2(dy, dx);
        double d_angle = new_angle - ctrl->pinch_angle;
        /* Wrap */
        while (d_angle > M_PI) d_angle -= 2.0 * M_PI;
        while (d_angle < -M_PI) d_angle += 2.0 * M_PI;
        arpt_camera_tilt_bearing(ctrl->cam, 0.0, -d_angle);
        ctrl->pinch_angle = new_angle;

        /* Vertical midpoint delta → tilt */
        double vert_delta = new_my - old_my;
        arpt_camera_tilt_bearing(ctrl->cam, -vert_delta * TILT_SENSITIVITY, 0.0);

        ctrl->touch0_x = t0x; ctrl->touch0_y = t0y;
        ctrl->touch1_x = t1x; ctrl->touch1_y = t1y;
    }
    return EM_TRUE;
}

static EM_BOOL on_touchend(int type, const EmscriptenTouchEvent *e, void *ud) {
    (void)type;
    arpt_control *ctrl = ud;
    ctrl->touch_count = e->numTouches;
    /* If going from 2 → 1 finger, re-anchor the remaining finger */
    if (e->numTouches == 1) {
        ctrl->touch0_x = e->touches[0].targetX;
        ctrl->touch0_y = e->touches[0].targetY;
        arpt_camera_pan_begin(ctrl->cam, ctrl->touch0_x, ctrl->touch0_y);
    }
    return EM_TRUE;
}

#endif /* __EMSCRIPTEN__ */

/* ── Public API ────────────────────────────────────────────────────────── */

arpt_control *arpt_control_create(arpt_camera *cam, GLFWwindow *window) {
    arpt_control *ctrl = calloc(1, sizeof(*ctrl));
    if (!ctrl) return NULL;

    ctrl->cam = cam;
    ctrl->window = window;

    glfwSetWindowUserPointer(window, ctrl);
    glfwSetMouseButtonCallback(window, on_mouse_button);
    glfwSetCursorPosCallback(window, on_cursor_pos);
    glfwSetScrollCallback(window, on_scroll);
    glfwSetKeyCallback(window, on_key);

#ifdef __EMSCRIPTEN__
    emscripten_set_touchstart_callback("canvas", ctrl, EM_TRUE, on_touchstart);
    emscripten_set_touchmove_callback("canvas", ctrl, EM_TRUE, on_touchmove);
    emscripten_set_touchend_callback("canvas", ctrl, EM_TRUE, on_touchend);
    emscripten_set_touchcancel_callback("canvas", ctrl, EM_TRUE, on_touchend);
#endif

    return ctrl;
}

void arpt_control_update(arpt_control *ctrl, double dt) {
    if (!ctrl || dt <= 0.0) return;

    /* ── Fly-to animation ──────────────────────────────────────────────── */
    if (ctrl->flyto.active) {
        ctrl->flyto.elapsed += dt;
        double t = ctrl->flyto.elapsed / FLYTO_DURATION;
        if (t >= 1.0) {
            t = 1.0;
            ctrl->flyto.active = false;
        }
        double e = ease_in_out_cubic(t);

        double dlon = wrap_lon(ctrl->flyto.target_lon - ctrl->flyto.start_lon);
        double lon = ctrl->flyto.start_lon + dlon * e;
        double lat = ctrl->flyto.start_lat +
                     (ctrl->flyto.target_lat - ctrl->flyto.start_lat) * e;
        double alt = ctrl->flyto.start_alt +
                     (ctrl->flyto.target_alt - ctrl->flyto.start_alt) * e;

        arpt_camera_set_position(ctrl->cam, lon, lat, alt);
        return; /* Skip inertia during fly-to */
    }

    /* ── Inertia decay ─────────────────────────────────────────────────── */
    if (ctrl->drag_mode != DRAG_IDLE) return; /* No inertia while dragging */

    double decay = pow(1.0 - DAMPING, dt * 60.0);

    /* Pan inertia */
    if (fabs(ctrl->vel_pan_x) > VEL_EPSILON ||
        fabs(ctrl->vel_pan_y) > VEL_EPSILON) {
        arpt_camera_pan(ctrl->cam,
                         ctrl->vel_pan_x * dt,
                         ctrl->vel_pan_y * dt);
        ctrl->vel_pan_x *= decay;
        ctrl->vel_pan_y *= decay;
        if (fabs(ctrl->vel_pan_x) < VEL_EPSILON) ctrl->vel_pan_x = 0.0;
        if (fabs(ctrl->vel_pan_y) < VEL_EPSILON) ctrl->vel_pan_y = 0.0;
    }

    /* Tilt/bearing inertia */
    if (fabs(ctrl->vel_tilt) > VEL_EPSILON ||
        fabs(ctrl->vel_bearing) > VEL_EPSILON) {
        arpt_camera_tilt_bearing(ctrl->cam,
                                  ctrl->vel_tilt * dt,
                                  ctrl->vel_bearing * dt);
        ctrl->vel_tilt *= decay;
        ctrl->vel_bearing *= decay;
        if (fabs(ctrl->vel_tilt) < VEL_EPSILON) ctrl->vel_tilt = 0.0;
        if (fabs(ctrl->vel_bearing) < VEL_EPSILON) ctrl->vel_bearing = 0.0;
    }
}

void arpt_control_free(arpt_control *ctrl) {
    free(ctrl);
}
