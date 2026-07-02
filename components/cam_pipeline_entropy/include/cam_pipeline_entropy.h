/*
 * Camera Pipeline Image-Entropy Consumer
 *
 * A reference frame consumer that chains a SHA-256 digest across live camera
 * frames to gather user-supplied entropy (the user points the camera at a
 * scene of their choosing). Uses the pipeline's public lock/release interface
 * -- exactly the pattern any external consumer would use -- and engages no QR
 * decoder.
 *
 * Chain semantics (matches the SeedSigner Pi Zero implementation so the host
 * can finish the hash identically):
 *   h = seed_hash                    (optional 32-byte caller seed, else empty)
 *   per preview frame: h = SHA256(h || frame_rgb565_bytes)
 *   final image: handed to the host RAW (NOT mixed here); the host computes
 *                final = SHA256(h || final_image_bytes) as the entropy result.
 *
 * The seed is for caller-supplied *uniqueness* (e.g. a hash of device uptime),
 * not an entropy source -- entropy comes only from the user-chosen camera
 * frames. Nothing is persisted: frames live only in the pipeline buffers and a
 * single latch buffer, hashed and then discarded. The chain digest and latch
 * are zeroized on destroy.
 */

#pragma once

#include "esp_cam_pipeline.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct cam_pipeline_entropy *cam_pipeline_entropy_handle_t;

/**
 * Optional progress callback, fired once per chained preview frame (drives a
 * UI frame-count / progress indicator). May be NULL. Runs in the consumer task
 * context -- must return quickly.
 */
typedef void (*cam_pipeline_entropy_frame_cb_t)(uint32_t frames_chained,
                                                void *user_ctx);

typedef struct {
    cam_pipeline_handle_t pipeline; // pipeline to consume frames from
    uint32_t frame_width;           // frame dimensions (for latch sizing)
    uint32_t frame_height;
    const uint8_t *seed_hash;       // OPTIONAL 32-byte chain seed; NULL = none
    cam_pipeline_entropy_frame_cb_t on_frame; // OPTIONAL progress callback
    void *user_ctx;                 // passed to on_frame
} cam_pipeline_entropy_config_t;

/**
 * Create the entropy consumer. Allocates a latch buffer and spawns a task that
 * locks frames from the pipeline and chains their RGB565 bytes into a running
 * SHA-256 digest. Returns handle on success, NULL on failure.
 */
cam_pipeline_entropy_handle_t
cam_pipeline_entropy_create(const cam_pipeline_entropy_config_t *config);

/**
 * Stop the task and free all resources. Zeroizes the chain digest and latch
 * buffer, and unfreezes the pipeline if a capture left it frozen. The pipeline
 * itself is not destroyed.
 */
void cam_pipeline_entropy_destroy(cam_pipeline_entropy_handle_t handle);

/**
 * Arm capture. On its next iteration the consumer task freezes the pipeline
 * display (holding the current frame on screen), latches that exact frame as
 * the final image, and freezes the chain digest at its pre-final value (the
 * latched frame is NOT chained). Idempotent while already captured.
 */
void cam_pipeline_entropy_capture(cam_pipeline_entropy_handle_t handle);

/**
 * Retrieve the frozen result. On success:
 *   chain          -> 32-byte chained digest over seed + all preview frames
 *                     (EXCLUDING the latched final image)
 *   chain_len      -> 32
 *   frame          -> latched RGB565 final-image bytes
 *   frame_len      -> frame_width * frame_height * 2
 *   frames_chained -> number of preview frames mixed into the chain
 * The chain and frame pointers stay valid until resume() or destroy(). Any
 * out-parameter may be NULL. Returns false until the latch completes after
 * capture() (poll a few ms), or if not currently captured.
 */
bool cam_pipeline_entropy_get_result(cam_pipeline_entropy_handle_t handle,
                                     const uint8_t **chain, size_t *chain_len,
                                     const uint8_t **frame, size_t *frame_len,
                                     uint32_t *frames_chained);

/**
 * Cancel / reshoot: discard the latched final image, unfreeze the pipeline,
 * and resume chaining live frames. The chain digest continues from where it
 * was (accumulated preview entropy is retained).
 */
void cam_pipeline_entropy_resume(cam_pipeline_entropy_handle_t handle);
