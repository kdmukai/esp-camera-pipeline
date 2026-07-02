/*
 * ESP Camera Pipeline — Internal Header
 *
 * Contains the full pipeline struct definition. Not part of the public API.
 */

#pragma once

#include "esp_cam_pipeline.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#if SOC_PPA_SUPPORTED
#include <driver/ppa.h>
#endif

#define EVENT_TASK_RUN BIT(0)
#define EVENT_DELETE   BIT(1)

struct cam_pipeline {
    // Config
    uint32_t display_width;
    uint32_t display_height;

    // Drivers
    const cam_pipeline_camera_driver_t *camera_driver;
    void *camera_handle;
    const cam_pipeline_display_driver_t *display_driver;
    void *display_handle;

    // Triple-buffered frame pool
    uint8_t *buffers[3];
    size_t buffer_size;
    uint8_t *front_buffer;  // most recently completed frame (display source)
    uint8_t *back_buffer;   // being written by camera crop
    uint8_t *locked_buffer; // held by consumer (NULL if none)
    // Monotonic frame counter, incremented under buffer_mutex on each promote
    // (new front frame). Lets a fast consumer skip re-processing an unchanged
    // frame: lock_frame_gen() reports the locked frame's generation.
    volatile uint32_t frame_generation;
    SemaphoreHandle_t buffer_mutex;

    // Frame access control
    volatile bool frame_access_paused;

    // Display + processing freeze: when set, frame_cb holds the current front
    // frame on screen (no promote, no display push) so a consumer can latch it
    // for a WYSIWYG capture. Camera keeps running; frames are dropped.
    volatile bool frozen;

    // Demand-driven frame skip: skip PPA processing when no consumer has
    // picked up the previous frame.  Skip at most 1 consecutive frame.
    volatile bool front_consumed;
    volatile int skip_count;

    // Lifecycle control
    EventGroupHandle_t event_group;
    volatile bool closing;
    volatile bool destruction_in_progress;
    volatile bool is_initialized;
    volatile int active_frame_operations;

    // PPA crop + scale + rotate (P4 only)
#if SOC_PPA_SUPPORTED
    ppa_client_handle_t ppa_client;
    ppa_srm_rotation_angle_t ppa_angle;
#endif

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    // Per-stage metrics (atomic counters, safe across cores)
    volatile uint32_t camera_frames;
    volatile uint32_t display_frames;
    volatile uint32_t display_skips;
    volatile uint32_t consumer_frames;
    volatile uint64_t consumer_lock_wait_us;
    volatile uint64_t consumer_hold_time_us;
    int64_t last_log_time;
#endif
};
