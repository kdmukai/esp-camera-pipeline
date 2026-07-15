/*
 * Camera Pipeline QR Decode Consumer
 *
 * A reference frame consumer that uses the pipeline's public lock/release
 * interface to decode QR codes from camera frames. Demonstrates the same
 * pattern any external consumer would use.
 */

#pragma once

#include "esp_cam_pipeline.h"
#include <k_quirc.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct cam_pipeline_qr *cam_pipeline_qr_handle_t;

/**
 * Per-frame decode outcome, reported once per processed frame. The single
 * event collapses payload delivery (DECODED) and the located-but-undecoded
 * signal (MISS) so a consumer can drive a most-recent-frame indicator without
 * correlating separate callbacks.
 */
typedef enum {
    CAM_QR_NOTHING = 0,  // no QR located this frame
    CAM_QR_MISS,         // QR located (corners valid) but decode failed
    CAM_QR_DECODED,      // QR decoded -- payload/metadata valid
} cam_pipeline_qr_outcome_t;

/**
 * Called from the decode task once per processed frame.
 * `outcome` summarizes the frame; payload/len/metadata are valid ONLY when
 * outcome == CAM_QR_DECODED (NULL/0 otherwise). A multi-code frame delivers the
 * first decoded code's payload. Must return quickly -- runs in decode task context.
 */
typedef void (*cam_pipeline_qr_frame_cb_t)(cam_pipeline_qr_outcome_t outcome,
                                           const uint8_t *payload, size_t len,
                                           const k_quirc_data_t *metadata,
                                           void *user_ctx);

typedef struct {
    cam_pipeline_handle_t pipeline; // Pipeline to consume frames from
    uint32_t frame_width;           // Frame dimensions (for QR decoder sizing)
    uint32_t frame_height;
    cam_pipeline_qr_frame_cb_t on_frame; // Per-frame outcome callback
    void *user_ctx;                  // Passed to callback

    // Focus-assist mode: skip quirc entirely and compute a live software
    // sharpness (Laplacian edge-energy) metric over the same center-square crop
    // instead. The loop is no longer quirc-bound (~4 fps) so it runs at the
    // camera rate (~15-24 fps) for a smooth on-screen focus meter. When set,
    // on_frame is optional (may be NULL) and no QR decoder is allocated. Poll
    // the metric with cam_pipeline_qr_get_focus_metric(). Default false.
    bool focus_assist;

    // Number of parallel decode tasks (1 or 2; 0 => 1, clamped to 2). A second
    // decoder runs on a second core with its own k_quirc instance to raise
    // per-frame catch reliability on hard frames -- useful only where that core is
    // otherwise free (the portrait DSI scan). Ignored in focus_assist mode (always
    // single). The two decoders take different camera frames; the throughput
    // ceiling remains the animated-QR source frame rate, not the decode rate.
    uint8_t num_decoders;
} cam_pipeline_qr_config_t;

/**
 * Create the QR decode consumer. Allocates k_quirc, grayscale LUT,
 * and spawns a decode task that locks frames from the pipeline,
 * converts to grayscale, and runs QR detection.
 * Returns handle on success, NULL on failure.
 */
cam_pipeline_qr_handle_t
cam_pipeline_qr_create(const cam_pipeline_qr_config_t *config);

/**
 * Stop the decode task and free all QR resources.
 * The pipeline itself is not affected.
 */
void cam_pipeline_qr_destroy(cam_pipeline_qr_handle_t handle);

/**
 * Decode reliability + timing snapshot, refreshed once per debug-log interval.
 * Populated only when CONFIG_CAM_PIPELINE_QR_DEBUG is enabled.
 */
typedef struct {
    float decode_fps;          /* frames processed per second */
    float gray_ms;             /* avg grayscale conversion time */
    float quirc_ms;            /* avg quirc identify+decode time */
    float detections_per_sec;  /* valid decodes per second */
    float identify_pct;        /* % of frames where a QR was located */
    float decode_pct;          /* % of frames that fully decoded (reliability) */
    uint32_t total_decodes;    /* monotonic count of valid decodes since create */
    float last_px_per_module;  /* measured px/module of the latest decode (HUD) */
    uint16_t last_modules;     /* module count of the latest decode */
} cam_pipeline_qr_debug_stats_t;

/**
 * Copy the latest reliability/timing snapshot into *out.
 * Returns true if stats are available (debug build + at least one interval
 * elapsed), false otherwise (out is zeroed).
 */
bool cam_pipeline_qr_get_debug_stats(cam_pipeline_qr_handle_t handle,
                                     cam_pipeline_qr_debug_stats_t *out);

/**
 * Live focus-assist sharpness reading (focus_assist mode only). Unlike the
 * debug stats above this is ALWAYS compiled -- focus-assist is a user-facing
 * feature, not instrumentation. The producing decode task writes it every
 * frame; a HUD/consumer polls it a few times a second.
 *
 * `sharpness` is a brightness-normalized Laplacian edge-energy (arbitrary
 * units; higher = sharper), EMA-smoothed for a steady readout. `peak` is a
 * ~1.5 s peak-hold of that value so the operator can see the sharpest point
 * they just passed through. The units are scene-dependent, so a meter should
 * auto-range against the observed peak rather than assume a fixed scale.
 */
typedef struct {
    float    sharpness;  /* EMA-smoothed brightness-normalized edge energy */
    float    peak;       /* ~1.5 s peak-hold of sharpness (recent running max) */
    float    raw;        /* latest unsmoothed sample (for diagnostics) */
    uint8_t  luma_mean;  /* mean luminance of the sampled crop, 0..255 */
    bool     usable;     /* luma in a usable range (not too dark / blown out) */
    uint32_t frames;     /* monotonic focus frames processed (liveness) */
} cam_pipeline_qr_focus_t;

/**
 * Copy the latest focus-assist reading into *out. Returns true when the handle
 * is a focus-assist consumer with at least one frame processed; false
 * otherwise (out is zeroed). Safe to call from any task.
 */
bool cam_pipeline_qr_get_focus_metric(cam_pipeline_qr_handle_t handle,
                                      cam_pipeline_qr_focus_t *out);

/**
 * Miss-frame capture (DIAGNOSTIC; only produces frames on a
 * CONFIG_CAM_PIPELINE_QR_DEBUG build). Samples at most one CAM_QR_MISS frame per
 * second -- a frame where a QR was located (finder patterns found) but did NOT
 * decode -- copying its grayscale crop so it can be written to the SD card and
 * inspected offline to see why quirc rejected it (blur vs. resolution vs. ECC).
 */
typedef struct {
    uint32_t seq;          /* monotonic capture index since create */
    int64_t  timestamp_us; /* esp_timer time of capture */
    int      quirc_err;    /* k_quirc_error_t of the rejected code */
    float    side_px;      /* measured QR side length in the crop (px) */
    float    sharpness;    /* Laplacian edge energy of the crop (focus proxy) */
    uint8_t  luma_mean;    /* mean luminance of the crop, 0..255 */
    uint32_t width;        /* crop dimensions (grayscale is width*height bytes) */
    uint32_t height;
} cam_pipeline_qr_miss_meta_t;

/**
 * Drain the latest sampled miss frame. Returns true and sets *out_buf (a pointer
 * into an internal buffer valid ONLY until the next poll -- copy it out at once)
 * + *out_len (== width*height) + *meta when a new sampled miss is available;
 * false otherwise (nothing captured since the last poll, or not a debug build).
 * Safe to call from a consumer task.
 */
bool cam_pipeline_qr_poll_miss_frame(cam_pipeline_qr_handle_t handle,
                                     const uint8_t **out_buf, size_t *out_len,
                                     cam_pipeline_qr_miss_meta_t *meta);
