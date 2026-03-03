#ifndef ARPENTRY_CONTROL_H
#define ARPENTRY_CONTROL_H

#include "camera.h"
#include <stdbool.h>

typedef struct GLFWwindow GLFWwindow;
typedef struct arpt_control arpt_control;

/**
 * Create a map control that installs GLFW input callbacks on the window.
 * The control does NOT own the camera or window — caller manages lifetimes.
 */
arpt_control *arpt_control_create(arpt_camera *cam, GLFWwindow *window);

/**
 * Advance inertia decay and fly-to animation. Call once per frame after
 * glfwPollEvents() and before rendering.
 */
void arpt_control_update(arpt_control *ctrl, double dt);

/** Returns true if the control needs a redraw (input, inertia, fly-to). */
bool arpt_control_needs_redraw(arpt_control *ctrl);

/** Free the control. Does not restore GLFW callbacks. */
void arpt_control_free(arpt_control *ctrl);

/**
 * Optional event filter: called before processing mouse button events.
 * Return true to consume the event (prevent map interaction).
 */
typedef bool (*arpt_event_filter_fn)(int button, int action, double sx,
                                     double sy, void *userdata);

void arpt_control_set_event_filter(arpt_control *ctrl, arpt_event_filter_fn fn,
                                   void *userdata);

#endif /* ARPENTRY_CONTROL_H */
