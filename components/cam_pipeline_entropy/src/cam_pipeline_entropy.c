/*
 * Camera Pipeline Image-Entropy Consumer
 *
 * Uses the pipeline's public lock_frame/release_frame interface -- exactly the
 * same pattern any external consumer would use. See the header for the chain
 * semantics and its parity with the SeedSigner Pi Zero implementation.
 */

#include "cam_pipeline_entropy.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "mbedtls/platform_util.h"
#include "mbedtls/sha256.h"

#define ENTROPY_DIGEST_LEN 32

static const char *TAG = "cam_pipeline_entropy";

struct cam_pipeline_entropy {
    cam_pipeline_handle_t pipeline;
    uint32_t frame_width;
    uint32_t frame_height;
    size_t frame_len; /* frame_width * frame_height * 2 (RGB565) */
    cam_pipeline_entropy_frame_cb_t on_frame;
    void *user_ctx;

    /* Chain state -- written only by the consumer task */
    uint8_t chain[ENTROPY_DIGEST_LEN];
    bool has_chain;
    uint32_t frames_chained;
    uint32_t last_gen; /* last pipeline frame generation hashed (skip dups) */

    /* Latched final image (PSRAM), copied once at capture */
    uint8_t *latch;

    /* Shared flags/counters guarded by state_mutex */
    SemaphoreHandle_t state_mutex;
    volatile bool arm_capture;
    volatile bool captured;

    TaskHandle_t task_handle;
    SemaphoreHandle_t task_done_sem;
    volatile bool closing;
};

/*
 * Advance the chain by one frame: chain = SHA256(chain || frame). The task is
 * the sole writer of chain/has_chain, so it hashes into a local digest and
 * publishes under the mutex, keeping get_result() consistent while holding the
 * lock only for the small copy (not the full-frame hash).
 */
static void chain_advance(struct cam_pipeline_entropy *e, const uint8_t *frame) {
    uint8_t next[ENTROPY_DIGEST_LEN];
    mbedtls_sha256_context ctx;

    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0); /* 0 = SHA-256 */
    if (e->has_chain) {
        mbedtls_sha256_update(&ctx, e->chain, ENTROPY_DIGEST_LEN);
    }
    mbedtls_sha256_update(&ctx, frame, e->frame_len);
    mbedtls_sha256_finish(&ctx, next);
    mbedtls_sha256_free(&ctx);

    xSemaphoreTake(e->state_mutex, portMAX_DELAY);
    memcpy(e->chain, next, ENTROPY_DIGEST_LEN);
    e->has_chain = true;
    e->frames_chained++;
    xSemaphoreGive(e->state_mutex);

    mbedtls_platform_zeroize(next, sizeof(next));
}

static void entropy_task(void *pvParameters) {
    struct cam_pipeline_entropy *e =
        (struct cam_pipeline_entropy *)pvParameters;

    while (!e->closing) {
        /* Capture takes priority: freeze first so the front frame == what is
         * on screen, then lock + copy that exact frame. */
        if (e->arm_capture && !e->captured) {
            cam_pipeline_freeze(e->pipeline);
            /* Let any in-flight frame_cb finish promoting/pushing so the front
             * buffer and the displayed frame agree (WYSIWYG). */
            vTaskDelay(pdMS_TO_TICKS(2));

            const uint8_t *frame = cam_pipeline_lock_frame(e->pipeline);
            if (frame) {
                memcpy(e->latch, frame, e->frame_len);
                cam_pipeline_release_frame(e->pipeline);

                xSemaphoreTake(e->state_mutex, portMAX_DELAY);
                e->captured = true;
                e->arm_capture = false;
                uint32_t n = e->frames_chained;
                xSemaphoreGive(e->state_mutex);

                ESP_LOGI(TAG, "captured after %" PRIu32 " chained frames", n);
            } else {
                /* Front not ready yet -- retry next iteration */
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            continue;
        }

        if (e->captured) {
            /* Frozen, awaiting host accept (destroy) or reshoot (resume) */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint32_t gen = 0;
        const uint8_t *frame = cam_pipeline_lock_frame_gen(e->pipeline, &gen);
        if (!frame) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Skip frames we've already hashed: the consumer is far faster than the
         * camera, so most locks return the same (unchanged) front frame. Only
         * distinct frames carry new entropy, and re-hashing wastes PSRAM
         * bandwidth the display/PPA path needs. */
        if (gen == e->last_gen) {
            cam_pipeline_release_frame(e->pipeline);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        e->last_gen = gen;

        chain_advance(e, frame);
        cam_pipeline_release_frame(e->pipeline);

        if (e->on_frame) {
            e->on_frame(e->frames_chained, e->user_ctx);
        }
    }

    if (e->task_done_sem) {
        xSemaphoreGive(e->task_done_sem);
    }
    vTaskSuspend(NULL);
}

/* --- Public API --- */

cam_pipeline_entropy_handle_t
cam_pipeline_entropy_create(const cam_pipeline_entropy_config_t *config) {
    if (!config || !config->pipeline) {
        ESP_LOGE(TAG, "Invalid config: pipeline required");
        return NULL;
    }
    if (config->frame_width == 0 || config->frame_height == 0) {
        ESP_LOGE(TAG, "Invalid config: frame dimensions required");
        return NULL;
    }

    struct cam_pipeline_entropy *e =
        calloc(1, sizeof(struct cam_pipeline_entropy));
    if (!e) {
        ESP_LOGE(TAG, "Failed to allocate entropy consumer struct");
        return NULL;
    }

    e->pipeline = config->pipeline;
    e->frame_width = config->frame_width;
    e->frame_height = config->frame_height;
    e->frame_len = (size_t)config->frame_width * config->frame_height * 2;
    e->on_frame = config->on_frame;
    e->user_ctx = config->user_ctx;

    if (config->seed_hash) {
        memcpy(e->chain, config->seed_hash, ENTROPY_DIGEST_LEN);
        e->has_chain = true;
    }

    e->latch =
        heap_caps_malloc(e->frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!e->latch) {
        e->latch = heap_caps_malloc(e->frame_len,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!e->latch) {
        ESP_LOGE(TAG, "Failed to allocate %u-byte latch buffer",
                 (unsigned)e->frame_len);
        goto error;
    }

    e->state_mutex = xSemaphoreCreateMutex();
    if (!e->state_mutex) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        goto error;
    }

    e->task_done_sem = xSemaphoreCreateBinary();
    if (!e->task_done_sem) {
        ESP_LOGE(TAG, "Failed to create task done semaphore");
        goto error;
    }

    /* Pin to Core 1 to avoid competing with the camera on Core 0 */
    BaseType_t result = xTaskCreatePinnedToCore(
        entropy_task, "cam_entropy",
        CONFIG_CAM_PIPELINE_ENTROPY_TASK_STACK_SIZE, e,
        CONFIG_CAM_PIPELINE_ENTROPY_TASK_PRIORITY, &e->task_handle, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create entropy task");
        goto error;
    }

    ESP_LOGI(TAG,
             "Entropy consumer created: %" PRIu32 "x%" PRIu32
             " (%u-byte frames, seed=%s)",
             e->frame_width, e->frame_height, (unsigned)e->frame_len,
             config->seed_hash ? "yes" : "no");

    return e;

error:
    cam_pipeline_entropy_destroy(e);
    return NULL;
}

void cam_pipeline_entropy_capture(cam_pipeline_entropy_handle_t handle) {
    if (!handle) {
        return;
    }
    handle->arm_capture = true;
}

bool cam_pipeline_entropy_get_result(cam_pipeline_entropy_handle_t handle,
                                     const uint8_t **chain, size_t *chain_len,
                                     const uint8_t **frame, size_t *frame_len,
                                     uint32_t *frames_chained) {
    if (!handle) {
        return false;
    }

    bool ok = false;
    xSemaphoreTake(handle->state_mutex, portMAX_DELAY);
    if (handle->captured) {
        if (chain) {
            *chain = handle->chain;
        }
        if (chain_len) {
            *chain_len = ENTROPY_DIGEST_LEN;
        }
        if (frame) {
            *frame = handle->latch;
        }
        if (frame_len) {
            *frame_len = handle->frame_len;
        }
        if (frames_chained) {
            *frames_chained = handle->frames_chained;
        }
        ok = true;
    }
    xSemaphoreGive(handle->state_mutex);
    return ok;
}

void cam_pipeline_entropy_resume(cam_pipeline_entropy_handle_t handle) {
    if (!handle) {
        return;
    }
    xSemaphoreTake(handle->state_mutex, portMAX_DELAY);
    handle->captured = false;
    handle->arm_capture = false;
    xSemaphoreGive(handle->state_mutex);
    cam_pipeline_unfreeze(handle->pipeline);
}

void cam_pipeline_entropy_destroy(cam_pipeline_entropy_handle_t handle) {
    if (!handle) {
        return;
    }

    struct cam_pipeline_entropy *e = handle;
    e->closing = true;

    if (e->task_handle && e->task_done_sem) {
        if (xSemaphoreTake(e->task_done_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "Timeout waiting for entropy task");
        }
        vTaskDelete(e->task_handle);
        e->task_handle = NULL;
    }

    /* Leave the pipeline streaming if a capture had frozen it */
    if (e->pipeline) {
        cam_pipeline_unfreeze(e->pipeline);
    }

    if (e->task_done_sem) {
        vSemaphoreDelete(e->task_done_sem);
        e->task_done_sem = NULL;
    }
    if (e->state_mutex) {
        vSemaphoreDelete(e->state_mutex);
        e->state_mutex = NULL;
    }

    /* Zeroize sensitive material before freeing (chain feeds a private key) */
    mbedtls_platform_zeroize(e->chain, sizeof(e->chain));
    if (e->latch) {
        mbedtls_platform_zeroize(e->latch, e->frame_len);
        heap_caps_free(e->latch);
        e->latch = NULL;
    }

    free(e);
    ESP_LOGI(TAG, "Entropy consumer destroyed");
}
