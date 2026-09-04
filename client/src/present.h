#ifndef ARPENTRY_PRESENT_H
#define ARPENTRY_PRESENT_H

#include <stdbool.h>
#include <stdint.h>
#include <webgpu/webgpu.h>

typedef struct GLFWwindow GLFWwindow;

/**
 * Where a frame goes: a window's swapchain, or nowhere.
 *
 * The viewer's render path is identical either way; only the acquisition of
 * a colour target and the fate of the finished frame differ.  Hiding both
 * behind one type keeps the difference out of the frame loop, and lets a
 * headless run skip GLFW, the display, and the surface entirely — which is
 * what makes the client usable as a measuring instrument (docs/VIEWER.md
 * "Headless capture").
 */
typedef struct arpt_present arpt_present;

/**
 * Open a window and its WebGPU surface. Returns NULL if either fails —
 * including when there is no display, which is the case a headless run
 * exists to avoid rather than to diagnose.
 */
arpt_present *arpt_present_create_window(WGPUInstance instance, int width,
                                          int height, const char *title);

/**
 * A presentation with no window, no surface and no display: frames land in
 * an offscreen texture and are read back. `format` should match what a
 * window would have chosen on this machine, or captures taken the two ways
 * will not compare.
 */
arpt_present *arpt_present_create_headless(uint32_t width, uint32_t height,
                                            WGPUTextureFormat format);

void arpt_present_free(arpt_present *p);

/** The surface to make the adapter compatible with; NULL when headless. */
WGPUSurface arpt_present_surface(const arpt_present *p);

/** The window for input and cursor queries; NULL when headless. */
GLFWwindow *arpt_present_window(const arpt_present *p);

/**
 * Settle on the swapchain format and size. Call once the device exists, and
 * again on every resize. Headless ignores `device`-side reconfiguration
 * beyond resizing its target.
 */
void arpt_present_configure(arpt_present *p, WGPUDevice device,
                             WGPUTextureFormat format, uint32_t width,
                             uint32_t height);

/** The format frames are rendered in. Valid after configure. */
WGPUTextureFormat arpt_present_format(const arpt_present *p);

/**
 * Acquire this frame's colour target, or NULL to skip the frame.
 *
 * `capture` asks for a target that can be copied from. A surface texture
 * cannot be (wgpu-native), so a windowed capture renders offscreen and is
 * never presented; headless is always offscreen, so capture costs nothing
 * there. Pair every non-NULL return with arpt_present_end.
 */
WGPUTextureView arpt_present_acquire(arpt_present *p, bool capture);

/** The texture behind the current acquire. Valid until arpt_present_end. */
WGPUTexture arpt_present_texture(const arpt_present *p);

/** Present the frame if it was drawn to a window, then release it. */
void arpt_present_end(arpt_present *p);

/** Framebuffer size in physical pixels. */
void arpt_present_framebuffer_size(const arpt_present *p, int *w, int *h);

/** Window size in logical pixels — matches cursor coordinates. */
void arpt_present_window_size(const arpt_present *p, int *w, int *h);

/**
 * Seconds since start. Headless counts frames at a fixed step instead of
 * reading a clock, so a capture does not depend on how fast the machine
 * that took it happened to be.
 */
double arpt_present_time(arpt_present *p);

/** Pump input. `block` waits up to 100 ms when there is nothing to draw. */
void arpt_present_poll(arpt_present *p, bool block);

bool arpt_present_should_close(const arpt_present *p);
void arpt_present_close(arpt_present *p);

#endif /* ARPENTRY_PRESENT_H */
