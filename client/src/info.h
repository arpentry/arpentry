#ifndef ARPENTRY_INFO_H
#define ARPENTRY_INFO_H

#include <webgpu/webgpu.h>
#include <stdint.h>

typedef struct arpt_info arpt_info;

/**
 * Create the info overlay (camera state display).
 * surface_format must match the render pass color attachment.
 * fb_width/fb_height are framebuffer dimensions; pixel_ratio = fb / window.
 */
arpt_info *arpt_info_create(WGPUDevice device, WGPUQueue queue,
                            WGPUTextureFormat surface_format,
                            uint32_t fb_width, uint32_t fb_height,
                            float pixel_ratio);

void arpt_info_free(arpt_info *info);

void arpt_info_resize(arpt_info *info, uint32_t fb_width, uint32_t fb_height,
                      float pixel_ratio);

/** Update camera state for display. Angles in degrees, altitude in meters. */
void arpt_info_set_camera(arpt_info *info, double lon_deg, double lat_deg,
                          double altitude, double bearing_deg,
                          double tilt_deg, double zoom_level);

/** Draw the info overlay into the current render pass. */
void arpt_info_draw(arpt_info *info, WGPURenderPassEncoder pass);

#endif /* ARPENTRY_INFO_H */
