#ifndef ARPENTRY_UI_H
#define ARPENTRY_UI_H

#include <webgpu/webgpu.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct arpt_ui arpt_ui;

typedef enum {
    ARPT_UI_NONE = 0,
    ARPT_UI_ZOOM_IN,
    ARPT_UI_ZOOM_OUT,
    ARPT_UI_RESET_NORTH,
    ARPT_UI_RESET_TILT,
} arpt_ui_action;

/**
 * Create the UI overlay.
 * surface_format must match the render pass color attachment.
 * fb_width/fb_height are framebuffer dimensions; pixel_ratio = fb / window.
 */
arpt_ui *arpt_ui_create(WGPUDevice device, WGPUQueue queue,
                        WGPUTextureFormat surface_format, uint32_t fb_width,
                        uint32_t fb_height, float pixel_ratio);

void arpt_ui_free(arpt_ui *ui);

void arpt_ui_resize(arpt_ui *ui, uint32_t fb_width, uint32_t fb_height,
                    float pixel_ratio);

/** Update compass bearing and tilt indicator (radians). */
void arpt_ui_set_state(arpt_ui *ui, float bearing_rad, float tilt_rad);

/** Update cursor position for hover effects (screen/window coordinates). */
void arpt_ui_set_cursor(arpt_ui *ui, float screen_x, float screen_y);

/**
 * Hit-test a click at screen coordinates.
 * Returns the action for the button under the cursor, or ARPT_UI_NONE.
 */
arpt_ui_action arpt_ui_hit_test(const arpt_ui *ui, float screen_x,
                                float screen_y);

/** Draw the UI overlay into the current render pass. */
void arpt_ui_draw(arpt_ui *ui, WGPURenderPassEncoder pass);

#endif /* ARPENTRY_UI_H */
