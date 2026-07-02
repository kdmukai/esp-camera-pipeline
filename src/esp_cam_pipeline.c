/*
 * ESP Camera Pipeline — Core Engine
 *
 * Triple-buffered camera → display → consumer pipeline.
 * Extracted and evolved from Kern.
 *
 * Three independent frame rates:
 *   Camera  — sensor native rate (the only hard clock)
 *   Display — up to camera rate, skips if display lock unavailable
 *   Consumer — whatever pace the consumer sustains via lock/release
 *
 * None of the three stages block each other.
 */

#include "esp_cam_pipeline_internal.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "cam_pipeline";

/*
 * Buffer allocation with SPIRAM preferred, internal RAM fallback.
 */
static uint8_t *allocate_buffer(size_t size) {
#if SOC_PPA_SUPPORTED
    /* PPA DMA requires cache-line-aligned buffers */
    uint8_t *buf = heap_caps_aligned_alloc(128, size,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_aligned_alloc(128, size,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
#else
    uint8_t *buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
#endif
    return buf;
}

/*
 * Center-crop camera frame to display dimensions.
 * Camera must be >= display in both dimensions.
 */
static void horizontal_crop(const uint8_t *camera_buf, uint8_t *display_buf,
                            uint32_t camera_width, uint32_t camera_height,
                            uint32_t display_width, uint32_t display_height) {
    if (camera_width < display_width || camera_height < display_height) {
        ESP_LOGE(TAG,
                 "Camera resolution %" PRIu32 "x%" PRIu32
                 " smaller than display %" PRIu32 "x%" PRIu32,
                 camera_width, camera_height, display_width, display_height);
        return;
    }

    uint32_t crop_x = (camera_width - display_width) / 2;
    uint32_t crop_y = (camera_height - display_height) / 2;
    const uint16_t *src = (const uint16_t *)camera_buf;
    uint16_t *dst = (uint16_t *)display_buf;

    for (uint32_t y = 0; y < display_height; y++) {
        const uint16_t *src_row = src + ((y + crop_y) * camera_width) + crop_x;
        uint16_t *dst_row = dst + (y * display_width);
        memcpy(dst_row, src_row, display_width * 2);
    }
}

#ifdef CONFIG_CAM_PIPELINE_DEBUG
static void log_debug_metrics(struct cam_pipeline *p) {
    int64_t now = esp_timer_get_time();
    int64_t elapsed_us = now - p->last_log_time;

    if (elapsed_us < (CONFIG_CAM_PIPELINE_DEBUG_LOG_INTERVAL_MS * 1000)) {
        return;
    }

    float elapsed_sec = elapsed_us / 1000000.0f;
    float camera_fps = p->camera_frames / elapsed_sec;
    float display_fps = p->display_frames / elapsed_sec;
    float display_skip_pct = 0;
    if (p->camera_frames > 0) {
        display_skip_pct =
            (p->display_skips * 100.0f) / p->camera_frames;
    }
    float consumer_fps = p->consumer_frames / elapsed_sec;

    float avg_lock_wait_ms = 0;
    float avg_hold_time_ms = 0;
    if (p->consumer_frames > 0) {
        avg_lock_wait_ms =
            (p->consumer_lock_wait_us / p->consumer_frames) / 1000.0f;
        avg_hold_time_ms =
            (p->consumer_hold_time_us / p->consumer_frames) / 1000.0f;
    }

    ESP_LOGI(TAG,
             "cam=%.1ffps disp=%.1ffps(skip %.0f%%) "
             "consumer=%.1ffps lock_wait=%.1fms hold=%.1fms",
             camera_fps, display_fps, display_skip_pct, consumer_fps,
             avg_lock_wait_ms, avg_hold_time_ms);

    // Reset counters
    p->camera_frames = 0;
    p->display_frames = 0;
    p->display_skips = 0;
    p->consumer_frames = 0;
    p->consumer_lock_wait_us = 0;
    p->consumer_hold_time_us = 0;
    p->last_log_time = now;
}
#endif

/*
 * Camera frame callback — called by the camera driver for each captured frame.
 * Runs on the camera driver's core (typically Core 0).
 *
 * Triple buffer selection: picks whichever buffer is neither front nor locked
 * as the back buffer. If only one buffer is available (the other two are front
 * and locked), it still works — the camera writes to the only free buffer.
 * If the camera fires again before the consumer releases, it overwrites the
 * back buffer in place (latest frame wins).
 */
static void frame_cb(uint8_t *camera_buf, uint32_t camera_width,
                     uint32_t camera_height, void *user_ctx) {
    struct cam_pipeline *p = (struct cam_pipeline *)user_ctx;

    __atomic_add_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);

    if (p->closing || p->destruction_in_progress || !p->is_initialized ||
        !p->event_group) {
        __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
        return;
    }

    EventBits_t bits = xEventGroupGetBits(p->event_group);
    if (!(bits & EVENT_TASK_RUN) || (bits & EVENT_DELETE)) {
        __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
        return;
    }

    /* Frozen: hold the current front frame on screen (and stable for a
     * consumer to latch). Skip all processing — no back-buffer selection,
     * no transform, no display push, no promote — so both the displayed
     * frame and front_buffer stay put until unfrozen. */
    if (p->frozen) {
        __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
        return;
    }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    __atomic_add_fetch(&p->camera_frames, 1, __ATOMIC_RELAXED);
    log_debug_metrics(p);
#endif

    /* Demand-driven frame skip: if no consumer has picked up the previous
     * frame, skip this one to avoid wasting PPA time.  Skip at most 1
     * frame — guarantees max staleness of 2 camera periods and keeps
     * enough processed frames flowing to feed the display. */
    if (!p->front_consumed && p->skip_count < 1 && p->front_buffer) {
        p->skip_count++;
#ifdef CONFIG_CAM_PIPELINE_DEBUG
        static int skip_total = 0;
        static int64_t skip_last_log = 0;
        skip_total++;
        int64_t now = esp_timer_get_time();
        if (now - skip_last_log > 2000000) {
            ESP_LOGI(TAG, "CAM SKIP: %d frames in 2s", skip_total);
            skip_total = 0;
            skip_last_log = now;
        }
#endif
        __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
        return;
    }
    p->skip_count = 0;

    // Select back buffer: whichever of the 3 is neither front nor locked
    xSemaphoreTake(p->buffer_mutex, portMAX_DELAY);
    uint8_t *back = NULL;
    for (int i = 0; i < 3; i++) {
        if (p->buffers[i] != p->front_buffer &&
            p->buffers[i] != p->locked_buffer) {
            back = p->buffers[i];
            break;
        }
    }
    p->back_buffer = back;
    xSemaphoreGive(p->buffer_mutex);

    if (!back) {
        // All three buffers occupied — shouldn't happen with single consumer,
        // but guard defensively
        __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
        return;
    }

    // Transform camera frame into back buffer

#if SOC_PPA_SUPPORTED
    if (p->ppa_client && !p->closing) {
        /* PPA pipeline is crop → scale → rotate.  When rotating 90/270°
         * the output dimensions swap, so the scale step must target
         * swapped dimensions to produce display_width × display_height
         * after rotation. */
        bool swap = (p->ppa_angle == PPA_SRM_ROTATION_ANGLE_90 ||
                     p->ppa_angle == PPA_SRM_ROTATION_ANGLE_270);
        uint32_t target_w = swap ? p->display_height : p->display_width;
        uint32_t target_h = swap ? p->display_width  : p->display_height;

        /* Uniform fill: scale to cover the target, center-crop the excess */
        float sx = (float)target_w / camera_width;
        float sy = (float)target_h / camera_height;
        float scale = (sx > sy) ? sx : sy;

        /* PPA scale is 4-bit fractional (scale = int + frag/16).
         * Quantize to the next higher representable scale so the output
         * is >= target dimensions, then center the oversized result. */
        float q_scale = ceilf(scale * 16.0f) / 16.0f;
        uint32_t in_w = (uint32_t)ceilf((float)target_w / q_scale);
        uint32_t in_h = (uint32_t)ceilf((float)target_h / q_scale);
        if (in_w > camera_width) in_w = camera_width;
        if (in_h > camera_height) in_h = camera_height;
        uint32_t off_x = (camera_width - in_w) / 2;
        uint32_t off_y = (camera_height - in_h) / 2;

#ifdef CONFIG_CAM_PIPELINE_DEBUG
        /* Pre-rotation output + post-rotation centering offset (debug log only) */
        uint32_t out_w = (uint32_t)(in_w * q_scale);
        uint32_t out_h = (uint32_t)(in_h * q_scale);
        if (out_w < target_w) out_w = target_w;
        if (out_h < target_h) out_h = target_h;
        uint32_t post_w = swap ? out_h : out_w;
        uint32_t post_h = swap ? out_w : out_h;
        uint32_t out_off_x = (post_w - p->display_width) / 2;
        uint32_t out_off_y = (post_h - p->display_height) / 2;

        static bool ppa_logged = false;
        if (!ppa_logged) {
            ESP_LOGI(TAG, "PPA: cam=%"PRIu32"x%"PRIu32" rot=%d"
                     " target=%"PRIu32"x%"PRIu32" -> disp=%"PRIu32"x%"PRIu32
                     " ideal=%.4f quant=%.4f in=%"PRIu32"x%"PRIu32
                     " out=%"PRIu32"x%"PRIu32" out_off=%"PRIu32",%"PRIu32,
                     camera_width, camera_height, p->ppa_angle * 90,
                     target_w, target_h,
                     p->display_width, p->display_height,
                     scale, q_scale, in_w, in_h, out_w, out_h,
                     out_off_x, out_off_y);
            ppa_logged = true;
        }

        int64_t ppa_t0 = esp_timer_get_time();
#endif
        ppa_srm_oper_config_t srm = {
            .in.buffer = camera_buf,
            .in.pic_w = camera_width,
            .in.pic_h = camera_height,
            .in.block_w = in_w,
            .in.block_h = in_h,
            .in.block_offset_x = off_x,
            .in.block_offset_y = off_y,
            .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            .out.buffer = back,
            .out.buffer_size = p->buffer_size,
            .out.pic_w = p->display_width,
            .out.pic_h = p->display_height,
            .out.block_offset_x = 0,
            .out.block_offset_y = 0,
            .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            .rotation_angle = p->ppa_angle,
            .scale_x = q_scale,
            .scale_y = q_scale,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        if (ppa_do_scale_rotate_mirror(p->ppa_client, &srm) != ESP_OK) {
            horizontal_crop(camera_buf, back, camera_width, camera_height,
                            p->display_width, p->display_height);
        }
#ifdef CONFIG_CAM_PIPELINE_DEBUG
        int64_t ppa_t1 = esp_timer_get_time();

        static int64_t cam_ppa_sum = 0;
        static int64_t cam_ppa_max = 0;
        static int cam_ppa_count = 0;
        static int64_t cam_ppa_last_log = 0;
        int64_t cam_ppa_dur = ppa_t1 - ppa_t0;
        cam_ppa_sum += cam_ppa_dur;
        if (cam_ppa_dur > cam_ppa_max) cam_ppa_max = cam_ppa_dur;
        cam_ppa_count++;
        if (ppa_t1 - cam_ppa_last_log > 2000000) {  /* every 2s */
            ESP_LOGI(TAG, "CAM PPA: avg=%lld us  max=%lld us  n=%d",
                     (long long)(cam_ppa_sum / cam_ppa_count),
                     (long long)cam_ppa_max, cam_ppa_count);
            cam_ppa_sum = 0;
            cam_ppa_max = 0;
            cam_ppa_count = 0;
            cam_ppa_last_log = ppa_t1;
        }
#endif
    } else {
        horizontal_crop(camera_buf, back, camera_width, camera_height,
                        p->display_width, p->display_height);
    }
#else
    horizontal_crop(camera_buf, back, camera_width, camera_height,
                    p->display_width, p->display_height);
#endif
    uint8_t *display_src = back;

    // Push to display (non-blocking — skip if display is busy)
    if (!p->closing && !p->destruction_in_progress && p->display_handle) {
        bool pushed = p->display_driver->push_frame(
            p->display_handle, display_src,
            p->display_width, p->display_height);
        if (pushed) {
            p->front_consumed = true;  // display used this frame — process next
        }
#ifdef CONFIG_CAM_PIPELINE_DEBUG
        if (pushed) {
            __atomic_add_fetch(&p->display_frames, 1, __ATOMIC_RELAXED);
        } else {
            __atomic_add_fetch(&p->display_skips, 1, __ATOMIC_RELAXED);
        }
#endif
    }

    // Promote back → front
    xSemaphoreTake(p->buffer_mutex, portMAX_DELAY);
    p->front_buffer = back;
    p->front_consumed = false;
    p->frame_generation++;  // new front frame available to consumers
    xSemaphoreGive(p->buffer_mutex);

    __atomic_sub_fetch(&p->active_frame_operations, 1, __ATOMIC_SEQ_CST);
}

/* --- Public API --- */

const uint8_t *cam_pipeline_lock_frame_gen(cam_pipeline_handle_t handle,
                                           uint32_t *generation) {
    if (!handle || handle->closing || handle->frame_access_paused) {
        return NULL;
    }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    int64_t wait_start = esp_timer_get_time();
#endif

    xSemaphoreTake(handle->buffer_mutex, portMAX_DELAY);

    if (!handle->front_buffer || handle->locked_buffer) {
        xSemaphoreGive(handle->buffer_mutex);
        return NULL;
    }

    handle->locked_buffer = handle->front_buffer;
    handle->front_consumed = true;
    /* Report the locked frame's generation atomically with the lock, so a
     * consumer can detect (and skip) an unchanged frame it already processed. */
    if (generation) {
        *generation = handle->frame_generation;
    }
    xSemaphoreGive(handle->buffer_mutex);

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    int64_t wait_end = esp_timer_get_time();
    __atomic_add_fetch(&handle->consumer_lock_wait_us,
                       (uint64_t)(wait_end - wait_start), __ATOMIC_RELAXED);
    __atomic_add_fetch(&handle->consumer_frames, 1, __ATOMIC_RELAXED);
#endif

    return handle->locked_buffer;
}

const uint8_t *cam_pipeline_lock_frame(cam_pipeline_handle_t handle) {
    return cam_pipeline_lock_frame_gen(handle, NULL);
}

void cam_pipeline_release_frame(cam_pipeline_handle_t handle) {
    if (!handle) {
        return;
    }

    xSemaphoreTake(handle->buffer_mutex, portMAX_DELAY);
    handle->locked_buffer = NULL;
    xSemaphoreGive(handle->buffer_mutex);
}

cam_pipeline_handle_t
cam_pipeline_create(const cam_pipeline_config_t *config) {
    if (!config || !config->camera_driver || !config->display_driver) {
        ESP_LOGE(TAG, "Invalid config: camera and display drivers required");
        return NULL;
    }

    struct cam_pipeline *p = calloc(1, sizeof(struct cam_pipeline));
    if (!p) {
        ESP_LOGE(TAG, "Failed to allocate pipeline struct");
        return NULL;
    }

    p->display_width = config->display_width;
    p->display_height = config->display_height;
    p->camera_driver = config->camera_driver;
    p->display_driver = config->display_driver;

    // Allocate three frame buffers (RGB565: width * height * 2 bytes each)
    p->buffer_size = config->display_width * config->display_height * 2;
    for (int i = 0; i < 3; i++) {
        p->buffers[i] = allocate_buffer(p->buffer_size);
        if (!p->buffers[i]) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            goto error;
        }
    }
    // Initialize: front = buffer[0], back and locked start NULL
    p->front_buffer = p->buffers[0];
    p->back_buffer = NULL;
    p->locked_buffer = NULL;
    p->front_consumed = true;   // don't skip the very first frame
    p->skip_count = 0;

    p->buffer_mutex = xSemaphoreCreateMutex();
    if (!p->buffer_mutex) {
        ESP_LOGE(TAG, "Failed to create buffer mutex");
        goto error;
    }

    p->event_group = xEventGroupCreate();
    if (!p->event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        goto error;
    }

    // Initialize display driver
    p->display_handle = p->display_driver->init(
        config->display_parent, config->display_width,
        config->display_height, config->display_config);
    if (!p->display_handle) {
        ESP_LOGE(TAG, "Display driver init failed");
        goto error;
    }

    // Initialize camera driver
    p->camera_handle = p->camera_driver->init(config->camera_config);
    if (!p->camera_handle) {
        ESP_LOGE(TAG, "Camera driver init failed");
        goto error;
    }

#if SOC_PPA_SUPPORTED
    {
        ppa_client_config_t ppa_cfg = {
            .oper_type = PPA_OPERATION_SRM,
        };
        if (ppa_register_client(&ppa_cfg, &p->ppa_client) != ESP_OK) {
            ESP_LOGW(TAG, "PPA registration failed, falling back to software crop");
            p->ppa_client = NULL;
        }
        switch (config->rotation) {
        case 90:  p->ppa_angle = PPA_SRM_ROTATION_ANGLE_90;  break;
        case 180: p->ppa_angle = PPA_SRM_ROTATION_ANGLE_180; break;
        case 270: p->ppa_angle = PPA_SRM_ROTATION_ANGLE_270; break;
        default:  p->ppa_angle = PPA_SRM_ROTATION_ANGLE_0;   break;
        }
    }
#endif

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    p->last_log_time = esp_timer_get_time();
#endif

    // Start camera streaming — frame_cb runs on camera driver's core
    xEventGroupSetBits(p->event_group, EVENT_TASK_RUN);
    p->is_initialized = true;

    esp_err_t err =
        p->camera_driver->start(p->camera_handle, frame_cb, p, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera driver start failed: %s", esp_err_to_name(err));
        p->is_initialized = false;
        goto error;
    }

    ESP_LOGI(TAG, "Pipeline created: %" PRIu32 "x%" PRIu32,
             config->display_width, config->display_height);

    return p;

error:
    cam_pipeline_destroy(p);
    return NULL;
}

void cam_pipeline_destroy(cam_pipeline_handle_t handle) {
    if (!handle) {
        return;
    }

    struct cam_pipeline *p = handle;

    p->destruction_in_progress = true;
    p->closing = true;
    p->is_initialized = false;

    // Signal camera task to stop
    if (p->event_group) {
        xEventGroupClearBits(p->event_group, EVENT_TASK_RUN);
        xEventGroupSetBits(p->event_group, EVENT_DELETE);
    }

    // Wait for in-flight frame callbacks to drain (300ms max)
    int wait_count = 0;
    while (__atomic_load_n(&p->active_frame_operations, __ATOMIC_SEQ_CST) > 0 &&
           wait_count < 30) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_count++;
    }
    int remaining = __atomic_load_n(&p->active_frame_operations, __ATOMIC_SEQ_CST);
    if (remaining > 0) {
        ESP_LOGW(TAG, "Timeout waiting for frame operations (remaining: %d)",
                 remaining);
    }

    // Stop camera
    if (p->camera_handle) {
        p->camera_driver->stop(p->camera_handle);
        p->camera_driver->deinit(p->camera_handle);
        p->camera_handle = NULL;
    }

    // Clean up display
    if (p->display_handle) {
        p->display_driver->deinit(p->display_handle);
        p->display_handle = NULL;
    }

    // Free frame buffers
    for (int i = 0; i < 3; i++) {
        if (p->buffers[i]) {
            heap_caps_free(p->buffers[i]);
            p->buffers[i] = NULL;
        }
    }
    p->front_buffer = NULL;
    p->back_buffer = NULL;
    p->locked_buffer = NULL;

    // Free PPA resources
#if SOC_PPA_SUPPORTED
    if (p->ppa_client) {
        ppa_unregister_client(p->ppa_client);
        p->ppa_client = NULL;
    }
#endif

    if (p->buffer_mutex) {
        vSemaphoreDelete(p->buffer_mutex);
        p->buffer_mutex = NULL;
    }

    if (p->event_group) {
        vEventGroupDelete(p->event_group);
        p->event_group = NULL;
    }

    free(p);
    ESP_LOGI(TAG, "Pipeline destroyed");
}

/* --- Camera controls --- */

esp_err_t cam_pipeline_set_ae_target(cam_pipeline_handle_t handle,
                                     uint8_t target) {
    if (!handle || !handle->camera_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!handle->camera_driver->set_ae_target) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return handle->camera_driver->set_ae_target(handle->camera_handle, target);
}

esp_err_t cam_pipeline_set_focus(cam_pipeline_handle_t handle,
                                 uint16_t position) {
    if (!handle || !handle->camera_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!handle->camera_driver->set_focus) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return handle->camera_driver->set_focus(handle->camera_handle, position);
}

bool cam_pipeline_has_focus_motor(cam_pipeline_handle_t handle) {
    if (!handle || !handle->camera_handle) {
        return false;
    }
    if (!handle->camera_driver->has_focus_motor) {
        return false;
    }
    return handle->camera_driver->has_focus_motor(handle->camera_handle);
}

/* --- Frame access control --- */

void cam_pipeline_pause_frame_access(cam_pipeline_handle_t handle) {
    if (handle) {
        handle->frame_access_paused = true;
    }
}

void cam_pipeline_resume_frame_access(cam_pipeline_handle_t handle) {
    if (handle) {
        handle->frame_access_paused = false;
    }
}

/* --- Display freeze --- */

void cam_pipeline_freeze(cam_pipeline_handle_t handle) {
    if (handle) {
        handle->frozen = true;
    }
}

void cam_pipeline_unfreeze(cam_pipeline_handle_t handle) {
    if (handle) {
        handle->frozen = false;
    }
}

/* --- Display overlay --- */

void *cam_pipeline_get_overlay_parent(cam_pipeline_handle_t handle) {
    if (!handle || !handle->display_handle) {
        return NULL;
    }
    if (!handle->display_driver->get_overlay_parent) {
        return NULL;
    }
    return handle->display_driver->get_overlay_parent(handle->display_handle);
}

/* --- Debug stats --- */

#ifdef CONFIG_CAM_PIPELINE_DEBUG
esp_err_t cam_pipeline_get_debug_stats(cam_pipeline_handle_t handle,
                                       cam_pipeline_debug_stats_t *stats) {
    if (!handle || !stats) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t now = esp_timer_get_time();
    int64_t elapsed_us = now - handle->last_log_time;
    float elapsed_sec = elapsed_us / 1000000.0f;

    if (elapsed_sec <= 0) {
        memset(stats, 0, sizeof(*stats));
        return ESP_OK;
    }

    stats->camera_fps = handle->camera_frames / elapsed_sec;
    stats->display_fps = handle->display_frames / elapsed_sec;
    stats->display_skip_pct = 0;
    if (handle->camera_frames > 0) {
        stats->display_skip_pct =
            (handle->display_skips * 100.0f) / handle->camera_frames;
    }
    stats->consumer_fps = handle->consumer_frames / elapsed_sec;

    if (handle->consumer_frames > 0) {
        stats->avg_consumer_lock_wait_ms =
            (handle->consumer_lock_wait_us / handle->consumer_frames) /
            1000.0f;
        stats->avg_consumer_hold_time_ms =
            (handle->consumer_hold_time_us / handle->consumer_frames) /
            1000.0f;
    } else {
        stats->avg_consumer_lock_wait_ms = 0;
        stats->avg_consumer_hold_time_ms = 0;
    }

    return ESP_OK;
}
#endif
