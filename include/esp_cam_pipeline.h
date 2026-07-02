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
