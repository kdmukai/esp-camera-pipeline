/*
 * ESP Camera Pipeline
 * Triple-buffered camera → display → consumer pipeline engine
 *
 * Abstract camera and display driver interfaces allow this engine
 * to run on different hardware (P4 CSI, S3 DVP, various displays)
 * without modification.
 */

#pragma once

#include "cam_pipeline_camera_driver.h"
#include "cam_pipeline_display_driver.h"
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct cam_pipeline *cam_pipeline_handle_t;

typedef struct {
    uint32_t display_width;  // Cropped frame width for display + consumers
    uint32_t display_height; // Cropped frame height for display + consumers
    uint32_t rotation;       // PPA rotation in degrees: 0, 90, 180, 270 (P4 only)

    // Optional mirror of the PPA output, applied in BOTH the display pass and the
    // still pass (P4 only). Corrects a sensor that reads out reflected relative to
    // the pipeline geometry — a handedness change a rotation cannot undo. Default
    // false. Applied after scale+rotate, so mirror_x/mirror_y are the output
    // (landscape display) horizontal/vertical axes.
    bool mirror_x;
    bool mirror_y;

    // Optional high-resolution still (P4 only). When non-zero AND the SoC has a
    // PPA, the pipeline allocates a dedicated still buffer and, on request_still(),
    // runs a SECOND PPA pass over the raw sensor frame into it (see request_still).
    // Meant to be a SQUARE (still_width == still_height) at a higher resolution than
    // the display crop, for a legible entropy confirm image. Leave 0 to disable
    // (the consumer then latches the display-resolution frame instead).
    uint32_t still_width;
    uint32_t still_height;

    const cam_pipeline_camera_driver_t *camera_driver;
    const void *camera_config; // Opaque, passed to camera_driver->init()

    const cam_pipeline_display_driver_t *display_driver;
    const void *display_config; // Opaque, passed to display_driver->init()
    void *display_parent;       // e.g. lv_obj_t* for LVGL, NULL for raw
} cam_pipeline_config_t;

/**
 * Allocate all resources (triple buffer, display surface, camera hardware)
 * and begin streaming.
 * Returns handle on success, NULL on failure.
 */
cam_pipeline_handle_t
cam_pipeline_create(const cam_pipeline_config_t *config);

/**
 * Stop streaming, tear down all tasks, free all resources.
 * Handle is invalid after this call.
 */
void cam_pipeline_destroy(cam_pipeline_handle_t handle);

/* --- Frame access for external consumers --- */

/**
 * Lock the most recent complete frame buffer. Returns a direct pointer
 * to the RGB565 pixel data — no copy. The buffer will not be overwritten
 * by the camera while locked. Returns NULL if no frame is available yet
 * or if frame access is paused.
 *
 * The caller MUST call release_frame() when done. Holding the lock too
 * long does not cause corruption — the camera and display continue using
 * the other two buffers — but it reduces the buffer pool and may cause
 * the camera to overwrite the back buffer in place if both remaining
 * buffers are also in use.
 */
const uint8_t *cam_pipeline_lock_frame(cam_pipeline_handle_t handle);

/**
 * Like cam_pipeline_lock_frame(), but also reports the locked frame's
 * generation — a monotonic counter incremented on each new camera frame.
 * A consumer faster than the camera can compare it to the last generation it
 * processed and skip re-processing an unchanged frame. `generation` may be
 * NULL (equivalent to cam_pipeline_lock_frame()).
 */
const uint8_t *cam_pipeline_lock_frame_gen(cam_pipeline_handle_t handle,
                                           uint32_t *generation);

/**
 * Release a previously locked frame buffer back to the pool.
 */
void cam_pipeline_release_frame(cam_pipeline_handle_t handle);

/* --- Camera control (callable anytime between create/destroy) --- */

esp_err_t cam_pipeline_set_ae_target(cam_pipeline_handle_t handle,
                                     uint8_t target);
esp_err_t cam_pipeline_set_focus(cam_pipeline_handle_t handle,
                                 uint16_t position);
bool cam_pipeline_has_focus_motor(cam_pipeline_handle_t handle);

/* --- Frame access control --- */

/**
 * Pause frame access. lock_frame() will return NULL until resumed.
 * Camera streaming and display preview continue unaffected.
 * Useful while adjusting exposure/focus so consumers don't waste cycles
 * on transitional frames.
 */
void cam_pipeline_pause_frame_access(cam_pipeline_handle_t handle);

/**
 * Resume frame access after pause.
 */
void cam_pipeline_resume_frame_access(cam_pipeline_handle_t handle);

/* --- Display freeze (hold the current frame on screen) --- */

/**
 * Freeze the pipeline on the current frame: frame_cb stops promoting new
 * camera frames and stops pushing to the display, so the last completed frame
 * holds on screen. The front frame also stays stable, so a consumer can
 * lock_frame() and copy it out for a WYSIWYG capture. The camera keeps
 * running; frames are simply dropped while frozen.
 */
void cam_pipeline_freeze(cam_pipeline_handle_t handle);

/**
 * Resume normal streaming after freeze().
 */
void cam_pipeline_unfreeze(cam_pipeline_handle_t handle);

/* --- High-resolution still grab (P4 only) --- */

/**
 * True if this pipeline can produce a high-resolution still — i.e. still
 * dimensions were configured, the SoC has a PPA, and the still buffer allocated.
 * A consumer queries this to decide whether to request a still or fall back to
 * latching a display-resolution frame.
 */
bool cam_pipeline_still_supported(cam_pipeline_handle_t handle);

/**
 * Request a one-shot high-resolution still. On its next NON-frozen frame, frame_cb
 * runs a second PPA pass over the raw sensor frame (crop the central square, scale
 * to the configured still size, apply the same rotation as the display path) into
 * the dedicated still buffer, then marks it ready. Grab the still BEFORE freeze():
 * frame_cb skips all processing while frozen, so a frozen pipeline never fills it.
 * Idempotent; each call re-arms (a subsequent lock_still() returns the fresh grab).
 * No-op if the pipeline has no still support.
 */
void cam_pipeline_request_still(cam_pipeline_handle_t handle);

/**
 * Return the still buffer once a requested grab has completed, else NULL. The
 * pointer is the configured still_width x still_height RGB565 image and stays valid
 * (and stable) until the next request_still() or destroy. No matching release —
 * the still buffer is dedicated, not part of the display/consumer pool.
 */
const uint8_t *cam_pipeline_lock_still(cam_pipeline_handle_t handle);

/* --- Display overlay --- */

/**
 * Get overlay parent for app UI widgets (display-driver-specific).
 * Returns lv_obj_t* for LVGL driver, NULL for raw framebuffer drivers.
 * App creates children on this object — they render on top of video feed.
 */
void *cam_pipeline_get_overlay_parent(cam_pipeline_handle_t handle);

/* --- Debug stats (only available when CONFIG_CAM_PIPELINE_DEBUG is set) --- */

#ifdef CONFIG_CAM_PIPELINE_DEBUG
typedef struct {
    float camera_fps;
    float display_fps;
    float display_skip_pct;
    float consumer_fps;
    float avg_consumer_lock_wait_ms;
    float avg_consumer_hold_time_ms;
} cam_pipeline_debug_stats_t;

/**
 * Read current debug metrics. Stats are computed from counters since
 * the last internal log interval reset. App can poll this to render
 * metrics into LVGL labels on the overlay parent.
 */
esp_err_t cam_pipeline_get_debug_stats(cam_pipeline_handle_t handle,
                                       cam_pipeline_debug_stats_t *stats);
#endif
