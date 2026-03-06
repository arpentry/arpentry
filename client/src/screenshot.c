#include "screenshot.h"
#include <webgpu/wgpu.h> /* wgpuDevicePoll (wgpu-native extension) */

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma clang diagnostic pop

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Buffer map callback state */
typedef struct {
    bool done;
    bool ok;
} map_state;

static void on_buffer_mapped(WGPUBufferMapAsyncStatus status, void *ud) {
    map_state *s = ud;
    s->ok = (status == WGPUBufferMapAsyncStatus_Success);
    s->done = true;
}

bool arpt_screenshot_save(WGPUInstance instance, WGPUDevice device,
                          WGPUQueue queue, WGPUTexture texture,
                          WGPUTextureFormat format, uint32_t width,
                          uint32_t height, const char *path) {
    uint32_t bytes_per_pixel = 4;
    /* WebGPU requires row alignment to 256 bytes */
    uint32_t unpadded_row = width * bytes_per_pixel;
    uint32_t padded_row = (unpadded_row + 255) & ~255u;
    uint64_t buf_size = (uint64_t)padded_row * height;

    /* Create staging buffer */
    WGPUBufferDescriptor buf_desc = {
        .label = "screenshot_staging",
        .size = buf_size,
        .usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead,
        .mappedAtCreation = false,
    };
    WGPUBuffer staging = wgpuDeviceCreateBuffer(device, &buf_desc);
    if (!staging) {
        fprintf(stderr, "[SCREENSHOT] failed to create staging buffer\n");
        return false;
    }

    /* Copy texture to buffer */
    WGPUCommandEncoder encoder =
        wgpuDeviceCreateCommandEncoder(device, NULL);

    WGPUImageCopyTexture src = {
        .texture = texture,
        .mipLevel = 0,
        .origin = {0, 0, 0},
        .aspect = WGPUTextureAspect_All,
    };
    WGPUImageCopyBuffer dst = {
        .buffer = staging,
        .layout = {
            .offset = 0,
            .bytesPerRow = padded_row,
            .rowsPerImage = height,
        },
    };
    WGPUExtent3D extent = {width, height, 1};
    wgpuCommandEncoderCopyTextureToBuffer(encoder, &src, &dst, &extent);

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, NULL);
    wgpuQueueSubmit(queue, 1, &cmd);
    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(encoder);

    /* Map the staging buffer */
    map_state state = {false, false};
    wgpuBufferMapAsync(staging, WGPUMapMode_Read, 0, buf_size,
                       on_buffer_mapped, &state);

    /* Spin on instance events until map completes */
    /* wgpuInstanceProcessEvents is unimplemented in this wgpu-native
       distribution; use the wgpu-native extension wgpuDevicePoll instead. */
    while (!state.done)
        wgpuDevicePoll(device, true, NULL);

    if (!state.ok) {
        fprintf(stderr, "[SCREENSHOT] buffer map failed\n");
        wgpuBufferRelease(staging);
        return false;
    }

    const uint8_t *mapped =
        wgpuBufferGetConstMappedRange(staging, 0, buf_size);
    if (!mapped) {
        fprintf(stderr, "[SCREENSHOT] failed to get mapped range\n");
        wgpuBufferUnmap(staging);
        wgpuBufferRelease(staging);
        return false;
    }

    /* Copy to contiguous RGBA buffer, stripping row padding */
    uint8_t *pixels = malloc((size_t)(width * height * bytes_per_pixel));
    if (!pixels) {
        wgpuBufferUnmap(staging);
        wgpuBufferRelease(staging);
        return false;
    }

    bool need_swap = (format == WGPUTextureFormat_BGRA8Unorm ||
                      format == WGPUTextureFormat_BGRA8UnormSrgb);

    for (uint32_t y = 0; y < height; y++) {
        const uint8_t *src_row = mapped + (size_t)y * padded_row;
        uint8_t *dst_row = pixels + (size_t)y * unpadded_row;
        memcpy(dst_row, src_row, unpadded_row);

        if (need_swap) {
            for (uint32_t x = 0; x < width; x++) {
                uint8_t *px = dst_row + x * 4;
                uint8_t tmp = px[0];
                px[0] = px[2];
                px[2] = tmp;
            }
        }
    }

    wgpuBufferUnmap(staging);
    wgpuBufferRelease(staging);

    /* Write PNG */
    int ok = stbi_write_png(path, (int)width, (int)height, 4, pixels,
                            (int)(width * bytes_per_pixel));
    free(pixels);

    if (!ok) {
        fprintf(stderr, "[SCREENSHOT] failed to write %s\n", path);
        return false;
    }

    printf("[SCREENSHOT] saved %s (%ux%u)\n", path, width, height);
    return true;
}
