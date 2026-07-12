/*
 * Camera Pipeline QR Decode Consumer
 *
 * Uses the pipeline's public lock_frame/release_frame interface --
 * exactly the same pattern any external consumer would use.
 */

#include "cam_pipeline_qr.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "cam_pipeline_qr";

/* --- RGB565 to grayscale LUT --- */

static uint8_t *rgb565_lut_build(void) {
    uint8_t *lut = heap_caps_malloc(65536, MALLOC_CAP_SPIRAM);
    if (!lut) {
        ESP_LOGW(TAG, "Failed to allocate RGB565 grayscale LUT");
        return NULL;
    }

    for (uint32_t i = 0; i < 65536; i++) {
        uint8_t r5 = (i >> 11) & 0x1F;
        uint8_t g6 = (i >> 5) & 0x3F;
        uint8_t b5 = i & 0x1F;
        uint8_t r8 = (r5 * 255 + 15) / 31;
        uint8_t g8 = (g6 * 255 + 31) / 63;
        uint8_t b8 = (b5 * 255 + 15) / 31;
        lut[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
    }

    return lut;
}

static void rgb565_to_grayscale(const uint8_t *rgb565_data, uint8_t *gray_data,
                                uint32_t width, uint32_t height,
                                const uint8_t *lut) {
    const uint16_t *pixels = (const uint16_t *)rgb565_data;
    uint32_t total = width * height;

    if (lut) {
        for (uint32_t i = 0; i < total; i++) {
            gray_data[i] = lut[pixels[i]];
        }
    } else {
        for (uint32_t i = 0; i < total; i++) {
            uint16_t pixel = pixels[i];
            uint8_t r5 = (pixel >> 11) & 0x1F;
            uint8_t g6 = (pixel >> 5) & 0x3F;
            uint8_t b5 = pixel & 0x1F;
            uint8_t r8 = (r5 * 255 + 15) / 31;
            uint8_t g8 = (g6 * 255 + 31) / 63;
            uint8_t b8 = (b5 * 255 + 15) / 31;
            gray_data[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
        }
    }
}

/**
 * Convert a centered square crop of an RGB565 frame to grayscale.
 * Only processes the crop region, writing crop_size * crop_size pixels.
 */
static void rgb565_to_grayscale_cropped(const uint8_t *rgb565_data,
                                        uint8_t *gray_data,
                                        uint32_t frame_width,
                                        uint32_t crop_size,
                                        uint32_t offset_x,
                                        uint32_t offset_y,
                                        const uint8_t *lut) {
    const uint16_t *pixels = (const uint16_t *)rgb565_data;

    for (uint32_t y = 0; y < crop_size; y++) {
        const uint16_t *src_row = pixels + (y + offset_y) * frame_width + offset_x;
        uint8_t *dst_row = gray_data + y * crop_size;

        if (lut) {
            for (uint32_t x = 0; x < crop_size; x++) {
                dst_row[x] = lut[src_row[x]];
            }
        } else {
            for (uint32_t x = 0; x < crop_size; x++) {
                uint16_t pixel = src_row[x];
                uint8_t r5 = (pixel >> 11) & 0x1F;
                uint8_t g6 = (pixel >> 5) & 0x3F;
                uint8_t b5 = pixel & 0x1F;
                uint8_t r8 = (r5 * 255 + 15) / 31;
                uint8_t g8 = (g6 * 255 + 31) / 63;
                uint8_t b8 = (b5 * 255 + 15) / 31;
                dst_row[x] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
            }
        }
    }
}

/* --- Internal state --- */

struct cam_pipeline_qr {
    cam_pipeline_handle_t pipeline;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t crop_size;      /* square crop dimension (min of w, h) */
    uint32_t crop_offset_x;  /* horizontal offset to center crop */
    uint32_t crop_offset_y;  /* vertical offset to center crop */
    cam_pipeline_qr_frame_cb_t on_frame;
    void *user_ctx;

    k_quirc_t *qr_decoder;
    uint8_t *rgb565_gray_lut;
    TaskHandle_t task_handle;
    SemaphoreHandle_t task_done_sem;
    volatile bool closing;

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    volatile uint32_t consumer_frames;
    volatile uint64_t grayscale_time_us;
    volatile uint64_t quirc_time_us;
    volatile uint32_t qr_detections;     /* valid decodes this window (codes) */
    volatile uint32_t frames_identified; /* frames where >=1 QR was located */
    volatile uint32_t frames_decoded;    /* frames where >=1 QR decoded valid */
    volatile uint32_t total_decodes;     /* monotonic valid decodes since create */
    volatile uint32_t missed_codes;      /* located-but-undecoded codes (window) */
    volatile float last_px_per_module;   /* last successful decode, for HUD */
    volatile uint16_t last_modules;      /* last successful decode module count */
    int64_t last_log_time;
    cam_pipeline_qr_debug_stats_t last_stats; /* latest snapshot for getter */
#endif
};

/* --- Debug logging --- */

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
static void log_debug_metrics(struct cam_pipeline_qr *qr) {
    int64_t now = esp_timer_get_time();
    int64_t elapsed_us = now - qr->last_log_time;

    if (elapsed_us < (CONFIG_CAM_PIPELINE_QR_DEBUG_LOG_INTERVAL_MS * 1000)) {
        return;
    }

    float elapsed_sec = elapsed_us / 1000000.0f;
    float consumer_fps = qr->consumer_frames / elapsed_sec;
    float avg_gray_ms = 0;
    float avg_quirc_ms = 0;
    float detections_per_sec = qr->qr_detections / elapsed_sec;
    /* Resolution-sweep reliability: identify% = frames where the QR was
     * located (finder patterns found); ok% = frames that fully decoded. A
     * high id% with a low ok% means the square is big enough to *find* the QR
     * but too small to *resolve* its modules -- the decode knee. */
    float identify_pct = 0;
    float decode_pct = 0;

    if (qr->consumer_frames > 0) {
        avg_gray_ms =
            (qr->grayscale_time_us / qr->consumer_frames) / 1000.0f;
        avg_quirc_ms =
            (qr->quirc_time_us / qr->consumer_frames) / 1000.0f;
        identify_pct =
            100.0f * qr->frames_identified / qr->consumer_frames;
        decode_pct =
            100.0f * qr->frames_decoded / qr->consumer_frames;
    }

    ESP_LOGI(TAG,
             "decode=%.1ffps(gray=%.1fms quirc=%.1fms) det/s=%.1f "
             "id=%.0f%% ok=%.0f%%",
             consumer_fps, avg_gray_ms, avg_quirc_ms, detections_per_sec,
             identify_pct, decode_pct);

    qr->last_stats = (cam_pipeline_qr_debug_stats_t){
        .decode_fps = consumer_fps,
        .gray_ms = avg_gray_ms,
        .quirc_ms = avg_quirc_ms,
        .detections_per_sec = detections_per_sec,
        .identify_pct = identify_pct,
        .decode_pct = decode_pct,
        .total_decodes = qr->total_decodes,
    };

    qr->consumer_frames = 0;
    qr->grayscale_time_us = 0;
    qr->quirc_time_us = 0;
    qr->qr_detections = 0;
    qr->frames_identified = 0;
    qr->frames_decoded = 0;
    qr->last_log_time = now;
}
#endif

/* --- Decode task --- */

static void qr_decode_task(void *pvParameters) {
    struct cam_pipeline_qr *qr = (struct cam_pipeline_qr *)pvParameters;
    k_quirc_result_t qr_result;

    while (!qr->closing) {

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        log_debug_metrics(qr);
#endif

        const uint8_t *frame = cam_pipeline_lock_frame(qr->pipeline);
        if (!frame) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        int64_t gray_start, gray_end, quirc_start, quirc_end;
#endif

        uint8_t *qr_buf = k_quirc_begin(qr->qr_decoder, NULL, NULL);
        if (qr_buf) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            gray_start = esp_timer_get_time();
#endif
            rgb565_to_grayscale_cropped(frame, qr_buf,
                                       qr->frame_width,
                                       qr->crop_size,
                                       qr->crop_offset_x,
                                       qr->crop_offset_y,
                                       qr->rgb565_gray_lut);
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            gray_end = esp_timer_get_time();
            quirc_start = esp_timer_get_time();
#endif
            k_quirc_end(qr->qr_decoder, false);
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            quirc_end = esp_timer_get_time();
#endif

            int num_codes = k_quirc_count(qr->qr_decoder);

            /* Per-frame outcome -- always computed (drives the unified callback),
             * independent of the debug build. any_identified = a QR was located
             * (finder patterns found); any_decoded = a code fully resolved. */
            bool any_identified = (num_codes > 0);
            bool any_decoded = false;
            const uint8_t *out_payload = NULL;
            size_t out_len = 0;
            const k_quirc_data_t *out_meta = NULL;

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            if (any_identified) {
                __atomic_add_fetch(&qr->frames_identified, 1,
                                   __ATOMIC_RELAXED);
            }
#endif
            for (int i = 0; i < num_codes; i++) {
                if (qr->closing) {
                    break;
                }

                k_quirc_error_t err =
                    k_quirc_decode(qr->qr_decoder, i, &qr_result);

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                /* QR side length in pixels (mean of the 4 edges from quirc's
                 * corners). quirc populates corners even on decode failure
                 * (corners-on-failure fix), so a located-but-undecoded code can
                 * still be measured. */
                float perim = 0.0f;
                for (int e = 0; e < 4; e++) {
                    int b = (e + 1) & 3;
                    float dx = (float)(qr_result.corners[b].x -
                                       qr_result.corners[e].x);
                    float dy = (float)(qr_result.corners[b].y -
                                       qr_result.corners[e].y);
                    perim += sqrtf(dx * dx + dy * dy);
                }
                float side_px = perim / 4.0f;
#endif

                if (err == K_QUIRC_SUCCESS && qr_result.valid) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                    __atomic_add_fetch(&qr->qr_detections, 1,
                                       __ATOMIC_RELAXED);
                    __atomic_add_fetch(&qr->total_decodes, 1,
                                       __ATOMIC_RELAXED);
                    /* Per-decode measured resolution; lets a distance sweep map
                     * decode success vs ACTUAL px/module, independent of any fill
                     * assumption. Short payload prefix carries the UR part header
                     * for animated dedup. Stash for the live HUD readout. */
                    int modules = 4 * qr_result.data.version + 17;
                    float px_per_mod = modules ? side_px / modules : 0.0f;
                    qr->last_px_per_module = px_per_mod;
                    qr->last_modules = (uint16_t)modules;
                    /* Per-decode detail at DEBUG so QR_DEBUG's INFO stream stays
                     * the clean 2s summary (`decode=..`) + the coordinator's
                     * `scan:` rate line, not one line per frame. Raise the tag to
                     * DEBUG (or esp_log_level_set) to recover the px/module sweep. */
                    ESP_LOGD(TAG,
                             "QRPX v%d mod%d side%.0fpx %.2fpx/mod len%d %.24s",
                             qr_result.data.version, modules, side_px,
                             px_per_mod, qr_result.data.payload_len,
                             (const char *)qr_result.data.payload);
#endif
                    /* First decoded code wins; one decode per frame is enough for
                     * scanning. qr_result is function-scope, so these pointers
                     * stay valid until the callback fires after the loop. */
                    any_decoded = true;
                    out_payload = qr_result.data.payload;
                    out_len = qr_result.data.payload_len;
                    out_meta = &qr_result.data;
                    break;
                }
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                else {
                    /* Located but not decoded: report measured pixel size + the
                     * quirc error so a distance sweep shows WHERE detection
                     * succeeds but decode fails (e.g. blown-out highlights up
                     * close). Module count is unknown — decode failed. */
                    __atomic_add_fetch(&qr->missed_codes, 1, __ATOMIC_RELAXED);
                    /* Per-miss detail at DEBUG (see QRPX note); the miss *rate*
                     * lives in the `scan:` line + `decode=..` id%/ok% summary. */
                    ESP_LOGD(TAG, "QRMISS side%.0fpx err=%d(%s)",
                             side_px, err, k_quirc_strerror(err));
                }
#endif
            }

            /* One unified per-frame outcome callback -- fired every processed
             * frame, including NOTHING, so a consumer can drop its indicator
             * when the QR leaves view. The consumer dedups; the engine never
             * suppresses (NEW vs REPEAT is a consumer-side distinction it cannot
             * see). payload/meta are valid only when DECODED. */
            cam_pipeline_qr_outcome_t outcome =
                any_decoded     ? CAM_QR_DECODED
                : any_identified ? CAM_QR_MISS
                                 : CAM_QR_NOTHING;
            qr->on_frame(outcome, out_payload, out_len, out_meta, qr->user_ctx);

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            if (any_decoded) {
                __atomic_add_fetch(&qr->frames_decoded, 1, __ATOMIC_RELAXED);
            }
            __atomic_add_fetch(&qr->consumer_frames, 1, __ATOMIC_RELAXED);
            __atomic_add_fetch(&qr->grayscale_time_us,
                               (uint64_t)(gray_end - gray_start),
                               __ATOMIC_RELAXED);
            __atomic_add_fetch(&qr->quirc_time_us,
                               (uint64_t)(quirc_end - quirc_start),
                               __ATOMIC_RELAXED);
#endif
        }

        cam_pipeline_release_frame(qr->pipeline);
    }

    if (qr->task_done_sem) {
        xSemaphoreGive(qr->task_done_sem);
    }

    /* Park -- do NOT self-delete. The stack/TCB came from
     * xTaskCreatePinnedToCoreWithCaps(), and a self vTaskDelete(NULL) would
     * leak them: the idle task never frees WithCaps memory. The give above is
     * this task's LAST touch of `qr` (and its last use of the frame -- released
     * at the bottom of the loop), so it is safe to reclaim us now. destroy()
     * takes the sem (orderly-shutdown handshake: it knows the frame is
     * released) then calls vTaskDeleteWithCaps(), which suspends us, waits until
     * we are off-core (cross-core-safe -- the exact guarantee the old
     * self-delete design was built to get), then deletes us and frees the
     * internal TCB + the PSRAM stack. */
    vTaskSuspend(NULL);
}

/* --- Public API --- */

cam_pipeline_qr_handle_t
cam_pipeline_qr_create(const cam_pipeline_qr_config_t *config) {
    if (!config || !config->pipeline || !config->on_frame) {
        ESP_LOGE(TAG, "Invalid config: pipeline and callback required");
        return NULL;
    }

    struct cam_pipeline_qr *qr = calloc(1, sizeof(struct cam_pipeline_qr));
    if (!qr) {
        ESP_LOGE(TAG, "Failed to allocate QR consumer struct");
        return NULL;
    }

    qr->pipeline = config->pipeline;
    qr->frame_width = config->frame_width;
    qr->frame_height = config->frame_height;
    qr->on_frame = config->on_frame;
    qr->user_ctx = config->user_ctx;

    /* Compute centered square crop (use shorter dimension) */
    uint32_t w = config->frame_width;
    uint32_t h = config->frame_height;
    qr->crop_size = (w < h) ? w : h;
    qr->crop_offset_x = (w - qr->crop_size) / 2;
    qr->crop_offset_y = (h - qr->crop_size) / 2;

    // Build RGB565->grayscale LUT (64KB, SPIRAM)
    qr->rgb565_gray_lut = rgb565_lut_build();
    // Non-fatal if LUT fails -- rgb565_to_grayscale falls back to per-pixel

    qr->qr_decoder = k_quirc_new();
    if (!qr->qr_decoder) {
        ESP_LOGE(TAG, "Failed to create QR decoder");
        goto error;
    }

    if (k_quirc_resize(qr->qr_decoder, qr->crop_size,
                       qr->crop_size) < 0) {
        ESP_LOGE(TAG, "Failed to resize QR decoder");
        goto error;
    }

    qr->task_done_sem = xSemaphoreCreateBinary();
    if (!qr->task_done_sem) {
        ESP_LOGE(TAG, "Failed to create task done semaphore");
        goto error;
    }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    qr->last_log_time = esp_timer_get_time();
#endif

    // Pin decode task to Core 1 to avoid competing with camera on Core 0.
    //
    // The stack is allocated in PSRAM (WithCaps puts only the stack under the
    // given caps; the TCB stays in internal RAM). The decode task is pure quirc
    // CPU over PSRAM-resident data and never runs while the flash cache is
    // disabled, so a PSRAM stack is safe. This removes the 16 KB *contiguous
    // internal* requirement that could not be met on SPI-display boards (ST7796,
    // e.g. P4 LCD 3.5): there the internal heap is squeezed by the internal-DMA
    // LVGL draw + camera stripe buffers that MIPI-DSI boards keep in PSRAM, so
    // re-opening the camera for a 2nd session (scan a SeedQR after a PSBT scan)
    // failed here even with plenty of free internal RAM (observed free=47 KB but
    // largest block=13 KB -- fragmentation, not exhaustion) and wedged the app.
    // Reclaimed via vTaskDeleteWithCaps() in cam_pipeline_qr_destroy().
    BaseType_t result = xTaskCreatePinnedToCoreWithCaps(
        qr_decode_task, "qr_decode",
        CONFIG_CAM_PIPELINE_QR_TASK_STACK_SIZE, qr,
        CONFIG_CAM_PIPELINE_QR_TASK_PRIORITY,
        &qr->task_handle, 1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create QR decode task (wanted %d stack; "
                 "SPIRAM free=%u largest=%u; INTERNAL free=%u largest=%u)",
                 (int)CONFIG_CAM_PIPELINE_QR_TASK_STACK_SIZE,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        goto error;
    }

    ESP_LOGI(TAG, "QR consumer created: %" PRIu32 "x%" PRIu32
             " (crop %" PRIu32 "x%" PRIu32 " at +%" PRIu32 ",+%" PRIu32 ")",
             config->frame_width, config->frame_height,
             qr->crop_size, qr->crop_size,
             qr->crop_offset_x, qr->crop_offset_y);

    return qr;

error:
    cam_pipeline_qr_destroy(qr);
    return NULL;
}

bool cam_pipeline_qr_get_debug_stats(cam_pipeline_qr_handle_t handle,
                                     cam_pipeline_qr_debug_stats_t *out) {
    if (!out) {
        return false;
    }
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    if (!handle) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    *out = handle->last_stats;
    out->total_decodes = handle->total_decodes; /* freshest monotonic count */
    out->last_px_per_module = handle->last_px_per_module; /* fresh, per-decode */
    out->last_modules = handle->last_modules;
    return true;
#else
    (void)handle;
    memset(out, 0, sizeof(*out));
    return false;
#endif
}

void cam_pipeline_qr_destroy(cam_pipeline_qr_handle_t handle) {
    if (!handle) {
        return;
    }

    struct cam_pipeline_qr *qr = handle;
    qr->closing = true;

    if (qr->task_handle && qr->task_done_sem) {
        /* Orderly-shutdown handshake: the task gives this sem once it has
         * released its last frame and is about to park (vTaskSuspend(NULL)).
         * Waiting on it here guarantees we delete the task at a safe point (no
         * frame ref held), not mid-decode. */
        if (xSemaphoreTake(qr->task_done_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
            /* No signal within 500 ms -> the task is hung mid-decode. Delete it
             * anyway; vTaskDeleteWithCaps() suspends it before freeing, so this
             * is safe even against a live task (a frame ref may leak, as with
             * the old force-delete, but that only fires on a real hang -- the
             * decode loop re-checks `closing` ~every 5 ms). */
            ESP_LOGW(TAG, "Timeout waiting for QR decode task; forcing delete");
        }
        /* Reclaim the task. vTaskDeleteWithCaps() suspends it, spins until it is
         * no longer running (cross-core-safe), then frees the internal TCB and
         * the PSRAM stack allocated by xTaskCreatePinnedToCoreWithCaps(). It is
         * synchronous, so no post-delete yield is needed before free(qr) below. */
        vTaskDeleteWithCaps(qr->task_handle);
        qr->task_handle = NULL;
    }

    if (qr->task_done_sem) {
        vSemaphoreDelete(qr->task_done_sem);
        qr->task_done_sem = NULL;
    }

    if (qr->qr_decoder) {
        k_quirc_destroy(qr->qr_decoder);
        qr->qr_decoder = NULL;
    }

    if (qr->rgb565_gray_lut) {
        heap_caps_free(qr->rgb565_gray_lut);
        qr->rgb565_gray_lut = NULL;
    }

    free(qr);
    ESP_LOGI(TAG, "QR consumer destroyed");
}
