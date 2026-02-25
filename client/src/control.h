#ifndef ARPENTRY_CONTROL_H
#define ARPENTRY_CONTROL_H

#include "camera.h"

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

/** Free the control. Does not restore GLFW callbacks. */
void arpt_control_free(arpt_control *ctrl);

#endif /* ARPENTRY_CONTROL_H */
