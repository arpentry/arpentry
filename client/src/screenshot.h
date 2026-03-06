#ifndef ARPENTRY_SCREENSHOT_H
#define ARPENTRY_SCREENSHOT_H

#include <stdbool.h>
#include <stdint.h>
#include <webgpu/webgpu.h>

/**
 * Read back a rendered surface texture and save it as a PNG file.
 *
 * The texture must have been created with WGPUTextureUsage_CopySrc.
 * Handles BGRA→RGBA conversion when format is BGRA8Unorm.
 *
 * Returns true on success, false on failure.
 */
bool arpt_screenshot_save(WGPUInstance instance, WGPUDevice device,
                          WGPUQueue queue, WGPUTexture texture,
                          WGPUTextureFormat format, uint32_t width,
                          uint32_t height, const char *path);

#endif /* ARPENTRY_SCREENSHOT_H */
