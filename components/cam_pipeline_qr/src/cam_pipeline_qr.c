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

/* ── Content-change decode gate ────────────────────────────────────────────
 * An animated QR held at a fixed rate (e.g. Sparrow's 5 fps => 200 ms/frame) is
 * captured by several camera frames, so without gating the two decoders spend
 * ~half their ~170 ms decode passes re-confirming the frame already on screen.
 *
 * The gate computes a cheap (~1 ms) perceptual hash of a center crop straight from
 * the RGB565 frame -- BEFORE the ~17 ms grayscale and ~170 ms quirc -- and decides:
 *   - content CHANGED vs the last decoded frame (hamming > THRESH)  -> decode
 *   - content SAME and we already decoded it (caught)               -> SKIP (~1 ms)
 *   - content SAME but the last decode of it FAILED (not caught)    -> decode (retry)
 *   - SAFETY-NET: force a decode if none happened for SAFETY_US, so a mis-tuned
 *     threshold can never stall the scan longer than one display period.
 * Skipping a re-sample frees the decoder to catch the NEXT distinct frame sooner.
 * Threshold validated in shadow mode (same-frame hamming ~12-23, changed ~44-85 for
 * decodable UR; FN ~0.2% at THRESH 40, all safety-net-recoverable).
 *
 * 1 = active gate; 0 = always-decode baseline (generation dispatch), for A/B. The
 * gate LOGIC is always compiled (a real feature); its counters/log are QR_DEBUG. */
#define CAM_QR_HASH_GATE      1
#if CAM_QR_HASH_GATE
#define CAM_QR_HASH_GRID      16     /* NxN aHash cells (256 bits = 4x uint64)      */
#define CAM_QR_HASH_WORDS     ((CAM_QR_HASH_GRID * CAM_QR_HASH_GRID + 63) / 64)
#define CAM_QR_HASH_CROP_PCT  100    /* center-crop % of the square used for the hash */
#define CAM_QR_HASH_THRESH    40     /* change threshold (Hamming distance)          */
#define CAM_QR_HASH_SAFETY_US 200000 /* force a decode at least this often (200 ms)  */
#endif

/* ── Parallel decoders (config num_decoders: 1 or 2) ───────────────────────
 * The consumer can run 1 or 2 decode tasks against ONE pipeline. Each task has
 * its OWN k_quirc instance; the 2nd is pinned to a second core and earns its keep
 * only where that core is otherwise free — the portrait DSI scan, which removes
 * the per-frame 90° display rotate from core 0.
 *
 * How two decoders share the single-locked_buffer pipeline: with 2 decoders each
 * task RELEASES the pipeline frame right after the grayscale COPY (not after the
 * ~170 ms decode), so quirc runs on the task's PRIVATE k_quirc buffer while the
 * peer locks the next frame. Only ONE pipeline frame is ever locked at a time, so
 * no 4th buffer is needed — the display never starves (validated on-device: disp
 * skip 0%, lock_wait 0). A monotonic frame-generation claim (last_dispatched_gen)
 * keeps the two decoders on DIFFERENT camera frames — provably no same-generation
 * double-decode, since the claim only ever advances.
 *
 * Ceiling note: throughput is bounded by the animated-QR SOURCE frame rate, not our
 * decode rate. Over a slower display (e.g. Sparrow's 5 fps) two decoders re-sample
 * the same on-screen frame — harmless (the coordinator dedups) — so the 2nd decoder
 * buys per-frame catch RELIABILITY up to that ceiling, not unbounded new-parts/s. */

/* Per-task decode context (the task fn is shared across both decoders). */
typedef struct {
    struct cam_pipeline_qr *qr;   /* shared pipeline/coordinator/stats owner */
    k_quirc_t             *dec;   /* THIS task's private k_quirc instance    */
    SemaphoreHandle_t      done_sem; /* orderly-shutdown handshake slot       */
    bool                   primary;  /* only the primary logs debug metrics   */
} qr_decode_arg_t;

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

/* --- Focus-assist sharpness metric --- */

/* Subsampled Laplacian edge-energy over the centered square crop, computed
 * FUSED with the RGB565->luma conversion (via the same LUT) so focus mode never
 * pays for a full-frame grayscale pass -- the whole point is a fast, non-quirc-
 * bound loop. Samples on a stride-S grid; each sample forms a 4-neighbour
 * Laplacian (4*c - left - right - up - down) and accumulates its square. A
 * sharper image has stronger high-frequency edges => larger energy.
 *
 * Returns the mean per-sample squared-Laplacian, brightness-normalized by the
 * mean luma squared so an exposure change doesn't masquerade as a focus change
 * (edge magnitude scales ~linearly with illumination; its square with luma^2).
 * Scaled into a friendly range; the absolute value is arbitrary (the meter
 * auto-ranges). *out_luma receives the mean crop luminance for the dark/blown
 * gate. */
#define FOCUS_SAMPLE_STRIDE 2u

/* Inline RGB565->luma (matches rgb565_to_grayscale), for the rare no-LUT path. */
static inline int rgb565_luma(uint16_t px) {
    uint8_t r8 = (uint8_t)((((px >> 11) & 0x1F) * 255 + 15) / 31);
    uint8_t g8 = (uint8_t)((((px >> 5) & 0x3F) * 255 + 31) / 63);
    uint8_t b8 = (uint8_t)(((px & 0x1F) * 255 + 15) / 31);
    return (int)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
}

static float rgb565_focus_energy_cropped(const uint8_t *rgb565_data,
                                         uint32_t frame_width,
                                         uint32_t crop_size,
                                         uint32_t offset_x,
                                         uint32_t offset_y,
                                         const uint8_t *lut,
                                         uint8_t *out_luma) {
    const uint16_t *pixels = (const uint16_t *)rgb565_data;
    const uint32_t S = FOCUS_SAMPLE_STRIDE;
    uint64_t energy = 0;
    uint64_t luma_sum = 0;
    uint32_t n = 0;

    if (crop_size <= 2 * S) {
        if (out_luma) *out_luma = 0;
        return 0.0f;
    }

    /* Interior only: each sample needs its +/-S neighbours in both axes. */
    for (uint32_t y = S; y + S < crop_size; y += S) {
        const uint16_t *row  = pixels + (offset_y + y) * frame_width + offset_x;
        const uint16_t *rowu = row - (size_t)S * frame_width;
        const uint16_t *rowd = row + (size_t)S * frame_width;
        for (uint32_t x = S; x + S < crop_size; x += S) {
            int c, l, r, u, d;
            if (lut) {
                c = lut[row[x]];
                l = lut[row[x - S]];
                r = lut[row[x + S]];
                u = lut[rowu[x]];
                d = lut[rowd[x]];
            } else {
                c = rgb565_luma(row[x]);
                l = rgb565_luma(row[x - S]);
                r = rgb565_luma(row[x + S]);
                u = rgb565_luma(rowu[x]);
                d = rgb565_luma(rowd[x]);
            }
            int lap = (c << 2) - l - r - u - d;
            energy += (uint32_t)(lap * lap);
            luma_sum += (uint32_t)c;
            n++;
        }
    }

    if (n == 0) {
        if (out_luma) *out_luma = 0;
        return 0.0f;
    }
    float luma_mean = (float)luma_sum / (float)n;
    if (out_luma) *out_luma = (uint8_t)luma_mean;
    float mean_energy = (float)energy / (float)n;
    /* Normalize by luma^2 (+eps guards a dark frame), scale to a readable range. */
    return mean_energy / (luma_mean * luma_mean + 1.0f) * 1000.0f;
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

    /* --- Parallel decoders (num_decoders == 2) --- */
    uint8_t           num_decoders;     /* 1 or 2 (clamped in create)     */
    qr_decode_arg_t   task_args[2];     /* [0]=primary, [1]=2nd decoder   */
    k_quirc_t        *qr_decoder2;      /* 2nd task's private instance    */
    TaskHandle_t      task_handle2;
    SemaphoreHandle_t task_done_sem2;
    volatile uint32_t last_dispatched_gen; /* frame-generation claim      */

    /* Focus-assist mode: no quirc, compute a live sharpness metric instead.
     * These are ALWAYS compiled (unlike the QR_DEBUG stats) -- the metric is a
     * user-facing feature. Written only by the decode task, read via the getter
     * on any task; word-aligned scalars, so torn reads are benign for a meter. */
    bool               focus_assist;
    volatile float     focus_ema;   /* EMA-smoothed sharpness */
    volatile float     focus_peak;  /* ~1.5 s peak-hold */
    volatile float     focus_raw;   /* latest unsmoothed sample */
    volatile uint8_t   focus_luma;  /* mean crop luminance */
    volatile bool      focus_usable;/* luma in usable range */
    volatile uint32_t  focus_frames;/* processed focus frames (liveness) */
    int64_t            focus_peak_hold_until; /* peak held until this time (us) */
    int64_t            focus_last_log_us;     /* periodic serial log throttle */

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
    /* Adaptive-decode (sweep+lock) cost/behaviour, per window. */
    volatile uint32_t adaptive_passes;   /* sum of identify passes over frames */
    volatile uint32_t adaptive_frames;   /* frames run through decode_adaptive */
    volatile uint32_t adaptive_local;    /* frames where the local pass fired */
    volatile int32_t  last_locked_offset;/* most recent locked threshold offset */
    int64_t last_log_time;
    cam_pipeline_qr_debug_stats_t last_stats; /* latest snapshot for getter */

    /* Miss-frame capture (diagnostic). The decode task grayscales a sampled
     * (<=1/s) MISS crop into miss_capture; poll_miss_frame copies it into
     * miss_poll (valid until the next poll) so the producer can immediately
     * reuse the capture buffer. Single-slot drop-oldest -- a diagnostic only
     * needs the latest sample. PSRAM; guarded by miss_lock. */
    uint8_t *miss_capture;               /* crop_size^2, decode-task writes  */
    uint8_t *miss_poll;                  /* crop_size^2, poll copies out      */
    cam_pipeline_qr_miss_meta_t miss_meta;
    SemaphoreHandle_t miss_lock;
    volatile bool miss_ready;            /* a fresh sample is waiting          */
    uint32_t miss_seq;                   /* monotonic capture index            */
    int64_t  miss_last_capture_us;       /* 1/s sampler                        */
#endif

#if CAM_QR_HASH_GATE
    /* Content-change gate: shared state across both decoders (µs-held mutex). */
    SemaphoreHandle_t g_lock;
    uint64_t g_last_hash[CAM_QR_HASH_WORDS];  /* hash of the last frame we decoded */
    uint32_t g_last_sig;                      /* payload sig of the last decode    */
    bool     g_caught;                        /* did the last-decoded QR succeed   */
    bool     g_primed;                        /* g_last_hash valid                 */
    int64_t  g_last_decode_us;                /* time of the last decode decision  */
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    /* Per-window instrumentation (both decoders accumulate atomically). */
    volatile uint32_t g_skip;       /* same+caught -> skipped (~1 ms)            */
    volatile uint32_t g_dec_chg;    /* decoded: content changed                  */
    volatile uint32_t g_dec_retry;  /* decoded: same content, not yet caught     */
    volatile uint32_t g_force;      /* decoded: safety-net forced                */
    volatile uint32_t g_force_new;  /* ...and it netted a NEW payload (a gate FN */
                                    /*    the safety-net recovered)              */
    volatile uint16_t g_ver;        /* last decoded QR version                   */
#endif
#endif
};

/* --- Debug logging --- */

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
/* Per-core CPU headroom probe (throughput-parallelization assessment).
 *
 * Reports, on the same ~2s cadence as the decode stats, how busy each HP core is
 * during a scan — derived from FreeRTOS run-time stats (per-task CPU counters).
 * The number that matters for lever #3 (a 2nd decode task): CORE-0 IDLE %, which
 * is the ceiling on decode a core-0 scavenger task could reclaim without stealing
 * from the camera/preview path. Core-1 (qr_decode) busy % confirms the current
 * decoder saturates its dedicated core (so a same-core 2nd instance can't help).
 *
 * Board-agnostic: the 4.3 (MIPI-DSI + PPA) offloads most preview work to hardware
 * so core 0 should show real idle; the 3.5 (ST7796 SPI) drives the panel from the
 * CPU, so expect less core-0 headroom there — measure both before committing.
 *
 * Compiled only when both the QR debug stream and FreeRTOS run-time stats are on
 * (enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS + CONFIG_FREERTOS_USE_TRACE_FACILITY).
 */
#if defined(CONFIG_CAM_PIPELINE_QR_DEBUG) &&                                    \
    defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS) &&                         \
    defined(CONFIG_FREERTOS_USE_TRACE_FACILITY)
#define CAM_PIPELINE_QR_CPU_PROBE 1
#define CPU_PROBE_MAX_TASKS 48
static inline int clamp_pct(int v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }

/* Per-core + per-task CPU load between calls, from FreeRTOS run-time stats.
 *
 * Summary line: core-0 / core-1 busy% (100 - that core's IDLE-task share). core-0
 * headroom is the ceiling on what a 2nd, core-0 decode task could reclaim.
 * Breakdown lines: the biggest consumers BY NAME + core, so a saturated core is
 * ATTRIBUTED (capture/frame_cb vs ISP vs LVGL vs the MicroPython poll vs qr_decode)
 * rather than inferred. Each pct is a share of ONE core's capacity over the interval
 * (a task pinned to a core at 60% prints 60%); the tasks on a given core sum to ~its
 * busy%. Deltas are matched to the prior snapshot by task handle, so a task created
 * or destroyed within the interval (e.g. qr_decode is recreated each scan) shows 0%
 * that sample instead of a spurious spike.
 *
 * Single caller (qr_decode_task), so file-static snapshots are safe. Needs
 * CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS + _USE_TRACE_FACILITY; the per-task core
 * column additionally needs CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID. */
static void log_cpu_load(void) {
    static TaskStatus_t cur[CPU_PROBE_MAX_TASKS];
    static void *prev_h[CPU_PROBE_MAX_TASKS];
    static uint64_t prev_rt[CPU_PROBE_MAX_TASKS];
    static int prev_n;
    static uint64_t last_total;
    static bool primed;

    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n == 0 || n > CPU_PROBE_MAX_TASKS)
        return;
    configRUN_TIME_COUNTER_TYPE total_raw = 0;
    UBaseType_t got = uxTaskGetSystemState(cur, CPU_PROBE_MAX_TASKS, &total_raw);
    uint64_t total = (uint64_t)total_raw;
    uint64_t dt = total - last_total;

    if (primed && dt) {
        int pct[CPU_PROBE_MAX_TASKS];
        uint64_t idle0_d = 0, idle1_d = 0;
        for (UBaseType_t i = 0; i < got; i++) {
            uint64_t prt = 0;
            bool found = false;
            for (int j = 0; j < prev_n; j++) {
                if (prev_h[j] == (void *)cur[i].xHandle) {
                    prt = prev_rt[j];
                    found = true;
                    break;
                }
            }
            uint64_t rt = (uint64_t)cur[i].ulRunTimeCounter;
            uint64_t d = (found && rt >= prt) ? (rt - prt) : 0;
            pct[i] = clamp_pct((int)(100ull * d / dt));
            if (!strcmp(cur[i].pcTaskName, "IDLE0"))
                idle0_d = d;
            else if (!strcmp(cur[i].pcTaskName, "IDLE1"))
                idle1_d = d;
        }
        int c0i = clamp_pct((int)(100ull * idle0_d / dt));
        int c1i = clamp_pct((int)(100ull * idle1_d / dt));
        ESP_LOGI(TAG,
                 "cpu_load: core0 busy=%d%% idle=%d%% | core1 busy=%d%% idle=%d%% "
                 "| core0 headroom=%d%%",
                 100 - c0i, c0i, 100 - c1i, c1i, c0i);

        /* biggest consumers by pct (repeated-max; skip the two IDLE tasks + <2%) */
        bool done[CPU_PROBE_MAX_TASKS] = {false};
        for (int k = 0; k < 12; k++) {
            int best = -1;
            for (UBaseType_t i = 0; i < got; i++) {
                if (done[i] || !strcmp(cur[i].pcTaskName, "IDLE0") ||
                    !strcmp(cur[i].pcTaskName, "IDLE1"))
                    continue;
                if (best < 0 || pct[i] > pct[best])
                    best = (int)i;
            }
            if (best < 0 || pct[best] < 2)
                break;
            done[best] = true;
#if (configTASKLIST_INCLUDE_COREID == 1)
            BaseType_t core = cur[best].xCoreID;
            const char *cstr = core == 0 ? "0" : (core == 1 ? "1" : "*");
#else
            const char *cstr = "?";
#endif
            ESP_LOGI(TAG, "  cpu[%-12s] core%s = %d%%", cur[best].pcTaskName, cstr,
                     pct[best]);
        }
    }

    /* save this snapshot for the next delta */
    prev_n = (int)got;
    for (UBaseType_t i = 0; i < got; i++) {
        prev_h[i] = (void *)cur[i].xHandle;
        prev_rt[i] = (uint64_t)cur[i].ulRunTimeCounter;
    }
    last_total = total;
    primed = true;
}
#endif

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

    /* Adaptive-decode phase monitor: passes/frame is the cost driver
     * (~1 once locked, up to the FAST cap on acquisition/misses); local% is how
     * often the THOROUGH second pass fired; lock is the current offset. */
    float avg_passes = 0;
    float local_pct = 0;
    if (qr->adaptive_frames > 0) {
        avg_passes = (float)qr->adaptive_passes / qr->adaptive_frames;
        local_pct = 100.0f * qr->adaptive_local / qr->adaptive_frames;
    }

    ESP_LOGI(TAG,
             "decode=%.1ffps(gray=%.1fms quirc=%.1fms) det/s=%.1f "
             "id=%.0f%% ok=%.0f%% | sweep=%.2fpass/f local=%.0f%% lock=%+d",
             consumer_fps, avg_gray_ms, avg_quirc_ms, detections_per_sec,
             identify_pct, decode_pct, avg_passes, local_pct,
             (int)qr->last_locked_offset);

#ifdef CAM_PIPELINE_QR_CPU_PROBE
    log_cpu_load();
#endif

#if CAM_QR_HASH_GATE
    /* GATE readout: skip = re-samples elided (~1 ms each, the win); dec chg/retry =
     * real decodes (new content / not-yet-caught retries); force = safety-net
     * decodes, force_new = those that netted NEW content (a hash miss the safety-net
     * recovered -- the number to keep ~0). ver/thr/grid/crop self-describe the run. */
    ESP_LOGI(TAG,
             "gate: skip=%u dec(chg=%u retry=%u force=%u) force_new=%u | ver=%u "
             "thr=%d grid=%d crop=%d%%",
             (unsigned)qr->g_skip, (unsigned)qr->g_dec_chg,
             (unsigned)qr->g_dec_retry, (unsigned)qr->g_force,
             (unsigned)qr->g_force_new, (unsigned)qr->g_ver,
             CAM_QR_HASH_THRESH, CAM_QR_HASH_GRID, CAM_QR_HASH_CROP_PCT);
    qr->g_skip = qr->g_dec_chg = qr->g_dec_retry = 0;
    qr->g_force = qr->g_force_new = 0;
#endif

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
    qr->adaptive_passes = 0;
    qr->adaptive_frames = 0;
    qr->adaptive_local = 0;
    qr->last_log_time = now;
}
#endif

/* --- Focus-assist metric update (decode task) --- */

/* EMA-smooth the raw sharpness and maintain a ~1.5 s peak-hold. The peak jumps
 * up instantly on a new max (and re-arms the hold window), holds while the
 * operator is at/near the sharpest point, then eases back down so a stale peak
 * from an old position doesn't linger. Brightness gate flags too-dark/blown
 * frames so the meter can say "fix the light" instead of reading noise. */
#define FOCUS_EMA_ALPHA        0.35f     /* handoff: modest alpha ~0.3-0.4 */
#define FOCUS_PEAK_HOLD_US     1500000   /* ~1.5 s */
#define FOCUS_PEAK_DECAY       0.05f     /* per-frame ease toward current once hold expires */
#define FOCUS_LUMA_MIN         18        /* below => too dark */
#define FOCUS_LUMA_MAX         245       /* above => blown out */

static void focus_update(struct cam_pipeline_qr *qr, float raw, uint8_t luma,
                         int64_t now) {
    float ema = (qr->focus_frames == 0)
                    ? raw
                    : FOCUS_EMA_ALPHA * raw + (1.0f - FOCUS_EMA_ALPHA) * qr->focus_ema;

    if (qr->focus_frames == 0 || ema >= qr->focus_peak) {
        qr->focus_peak = ema;
        qr->focus_peak_hold_until = now + FOCUS_PEAK_HOLD_US;
    } else if (now >= qr->focus_peak_hold_until) {
        qr->focus_peak += (ema - qr->focus_peak) * FOCUS_PEAK_DECAY;
    }

    qr->focus_ema = ema;
    qr->focus_raw = raw;
    qr->focus_luma = luma;
    qr->focus_usable = (luma >= FOCUS_LUMA_MIN && luma <= FOCUS_LUMA_MAX);
    qr->focus_frames++;

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    /* Periodic serial line for on-device calibration (the app-side meter is the
     * real surface; this just lets a dev read the numbers over the QR_DEBUG
     * stream while validating). ~1 Hz so it doesn't flood. */
    if (qr->focus_last_log_us == 0) {
        qr->focus_last_log_us = now;
    } else if (now - qr->focus_last_log_us >= 1000000) {
        ESP_LOGI(TAG, "focus: sharp=%.1f peak=%.1f raw=%.1f luma=%u %s",
                 qr->focus_ema, qr->focus_peak, qr->focus_raw,
                 (unsigned)qr->focus_luma, qr->focus_usable ? "" : "(light!)");
        qr->focus_last_log_us = now;
    }
#endif
}

/* --- Content-change gate helpers --- */

#if CAM_QR_HASH_GATE
/* aHash of a center crop, sampled straight from the RGB565 frame via the luma LUT
 * (NO full grayscale) so a gate SKIP costs ~1 ms, not ~17 ms. Downsample to GRID x
 * GRID cell means (strided within each cell), binarize each vs the grid mean.
 * Exposure-invariant (cell means and their mean threshold scale together). The
 * center crop concentrates on the densest, most jitter-stable part of the QR; a
 * finer grid over a smaller crop raises effective per-module resolution for dense
 * codes. `rgb565` is frame_w wide; the QR square sits at (offx,offy) size `crop`.
 * Stack-local means[] (NOT static): both decoders call this concurrently. */
static void gate_hash_rgb565(const uint8_t *rgb565, uint32_t frame_w, uint32_t crop,
                             uint32_t offx, uint32_t offy, const uint8_t *lut,
                             uint64_t out[CAM_QR_HASH_WORDS]) {
    const uint16_t *px = (const uint16_t *)rgb565;
    uint32_t side = crop * CAM_QR_HASH_CROP_PCT / 100;
    if (side < CAM_QR_HASH_GRID) side = crop;      /* degenerate guard */
    uint32_t o    = (crop - side) / 2;
    uint32_t bx0  = offx + o, by0 = offy + o;
    uint32_t cell = side / CAM_QR_HASH_GRID;       /* px per cell (>=1) */
    if (cell == 0) cell = 1;
    uint32_t step = (cell / 4) ? (cell / 4) : 1;   /* sample stride within a cell */

    uint32_t means[CAM_QR_HASH_GRID * CAM_QR_HASH_GRID];  /* 1 KB stack, per-call */
    uint64_t total = 0;
    for (uint32_t gy = 0; gy < CAM_QR_HASH_GRID; gy++) {
        for (uint32_t gx = 0; gx < CAM_QR_HASH_GRID; gx++) {
            uint32_t bx = bx0 + gx * cell, by = by0 + gy * cell;
            uint32_t sum = 0, n = 0;
            for (uint32_t y = 0; y < cell; y += step) {
                const uint16_t *row = px + (size_t)(by + y) * frame_w + bx;
                for (uint32_t x = 0; x < cell; x += step) {
                    sum += lut ? lut[row[x]] : (uint32_t)rgb565_luma(row[x]);
                    n++;
                }
            }
            uint32_t m = n ? sum / n : 0;
            means[gy * CAM_QR_HASH_GRID + gx] = m;
            total += m;
        }
    }
    uint32_t thresh = (uint32_t)(total / (CAM_QR_HASH_GRID * CAM_QR_HASH_GRID));
    for (int w = 0; w < CAM_QR_HASH_WORDS; w++) out[w] = 0;
    for (int i = 0; i < CAM_QR_HASH_GRID * CAM_QR_HASH_GRID; i++) {
        if (means[i] >= thresh) out[i >> 6] |= (uint64_t)1 << (i & 63);
    }
}

static inline int gate_hamming(const uint64_t a[CAM_QR_HASH_WORDS],
                               const uint64_t b[CAM_QR_HASH_WORDS]) {
    int d = 0;
    for (int w = 0; w < CAM_QR_HASH_WORDS; w++)
        d += __builtin_popcountll(a[w] ^ b[w]);
    return d;
}

/* FNV-1a over the decoded payload -- distinct fountain seqNum => distinct bytes =>
 * distinct sig (used only to audit whether a forced decode netted new content). */
static uint32_t gate_payload_sig(const uint8_t *p, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}
#endif /* CAM_QR_HASH_GATE */

/* --- Decode task --- */

static void qr_decode_task(void *pvParameters) {
    qr_decode_arg_t *arg = (qr_decode_arg_t *)pvParameters;
    struct cam_pipeline_qr *qr = arg->qr;
    k_quirc_t *dec = arg->dec;          /* this task's private decoder */
    const bool primary = arg->primary;  /* only the primary logs metrics */
    k_quirc_result_t qr_result;
    uint32_t last_gen = UINT32_MAX;  /* focus mode: skip re-processing a stale frame */

    while (!qr->closing) {

        /* Focus-assist: grayscale-crop + Laplacian, NO quirc. Gated on the
         * frame generation so we run at the camera rate rather than pegging the
         * core recomputing an unchanged buffer (the loop is ~10x faster than a
         * quirc decode). */
        if (qr->focus_assist) {
            uint32_t gen = 0;
            const uint8_t *fframe = cam_pipeline_lock_frame_gen(qr->pipeline, &gen);
            if (!fframe) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            if (gen == last_gen) {
                cam_pipeline_release_frame(qr->pipeline);
                vTaskDelay(pdMS_TO_TICKS(3));  /* wait for a fresh frame */
                continue;
            }
            last_gen = gen;
            uint8_t luma = 0;
            float raw = rgb565_focus_energy_cropped(fframe, qr->frame_width,
                                                    qr->crop_size,
                                                    qr->crop_offset_x,
                                                    qr->crop_offset_y,
                                                    qr->rgb565_gray_lut, &luma);
            cam_pipeline_release_frame(qr->pipeline);
            focus_update(qr, raw, luma, esp_timer_get_time());
            continue;
        }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        /* Single-caller instrument (file-static): primary decoder only. */
        if (primary) log_debug_metrics(qr);
#endif

        uint32_t gen = 0;
        const uint8_t *frame = cam_pipeline_lock_frame_gen(qr->pipeline, &gen);
        if (!frame) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

#if CAM_QR_HASH_GATE
        (void)gen;
        /* Content-change gate: cheap RGB565 hash + decode/skip decision BEFORE the
         * ~17 ms grayscale + ~170 ms decode. Skipping an already-caught re-sample
         * costs ~1 ms and frees this decoder to catch the next distinct frame. The
         * gate also serves as the 2-decoder dispatch (no separate generation claim):
         * two decoders that lock the same content converge to one decode + a skip. */
        bool g_forced = false, g_retry = false;
        {
            uint64_t g_h[CAM_QR_HASH_WORDS];
            gate_hash_rgb565(frame, qr->frame_width, qr->crop_size,
                             qr->crop_offset_x, qr->crop_offset_y,
                             qr->rgb565_gray_lut, g_h);
            bool g_do;
            int64_t g_now = esp_timer_get_time();
            xSemaphoreTake(qr->g_lock, portMAX_DELAY);
            if (!qr->g_primed) {
                g_do = true;
            } else {
                bool changed = gate_hamming(g_h, qr->g_last_hash) > CAM_QR_HASH_THRESH;
                bool stale = (g_now - qr->g_last_decode_us) > CAM_QR_HASH_SAFETY_US;
                if (changed)            g_do = true;                    /* new frame  */
                else if (!qr->g_caught){ g_do = true; g_retry  = true; } /* retry    */
                else if (stale)        { g_do = true; g_forced = true; } /* safety   */
                else                    g_do = false;                   /* skip      */
            }
            if (g_do) {
                memcpy(qr->g_last_hash, g_h, sizeof(g_h));
                qr->g_last_decode_us = g_now;
                qr->g_caught = false;   /* pending until this decode resolves */
                qr->g_primed = true;
            }
            xSemaphoreGive(qr->g_lock);
            if (!g_do) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                __atomic_add_fetch(&qr->g_skip, 1, __ATOMIC_RELAXED);
#endif
                cam_pipeline_release_frame(qr->pipeline);
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
        }
#else
        /* Dual-decode dispatch: claim this frame generation so the two decoders
         * take DIFFERENT frames. Store the claim before grayscale/release-early so
         * the peer sees it. If the peer already claimed this (or a newer)
         * generation, drop and wait for a fresh frame. (Single decoder: no-op.) */
        if (qr->num_decoders > 1) {
            uint32_t last = __atomic_load_n(&qr->last_dispatched_gen, __ATOMIC_RELAXED);
            if (gen != 0 && gen <= last) {
                cam_pipeline_release_frame(qr->pipeline);
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            __atomic_store_n(&qr->last_dispatched_gen, gen, __ATOMIC_RELAXED);
        } else {
            (void)gen;
        }
#endif

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        int64_t gray_start, gray_end, quirc_start, quirc_end;
#endif

        bool frame_released = false;
        uint8_t *qr_buf = k_quirc_begin(dec, NULL, NULL);
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
            if (qr->num_decoders > 1) {
                /* Release the pipeline frame right after the grayscale COPY so the
                 * peer decoder can lock the next frame while this task runs the
                 * ~170 ms quirc pass on its own (dec's) buffer. This is what makes
                 * two concurrent decoders possible against the single-locked_buffer
                 * pipeline. (Miss-frame capture, which re-reads `frame` post-decode,
                 * is disabled below when num_decoders > 1.) */
                cam_pipeline_release_frame(qr->pipeline);
                frame_released = true;
            }
            k_quirc_adaptive_stats_t astats;
            /* Adaptive-threshold decode: bootstrap offset sweep + lock, then a
             * failure-gated local (Bradley) pass under THOROUGH. FAST here bounds
             * the per-frame tax so an animated scan can't stall on undecodable
             * frames. Replaces the old fixed-offset end()+count()+decode() loop. */
            bool any_decoded =
                k_quirc_decode_adaptive(dec, &qr_result,
                                        K_QUIRC_EFFORT_FAST, &astats) != 0;
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            quirc_end = esp_timer_get_time();
#endif

            /* any_identified = a QR was located (finder patterns found). True on a
             * decode; on a full-sweep miss the last pass may still have left a
             * grid -- treat that as located-but-undecoded. */
            bool any_identified =
                any_decoded || (k_quirc_count(dec) > 0);
            const uint8_t *out_payload = NULL;
            size_t out_len = 0;
            const k_quirc_data_t *out_meta = NULL;
            if (any_decoded) {
                out_payload = qr_result.data.payload;
                out_len = qr_result.data.payload_len;
                out_meta = &qr_result.data;
            }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            /* Located-but-undecoded meta for the miss-frame capture below. */
            bool  frame_has_miss = false;
            int   frame_miss_err = 0;
            float frame_miss_side = 0.0f;

            /* QR side length in px: mean of the 4 edges from quirc's corners,
             * which are populated even on decode failure (corners-on-failure). */
            float side_px = 0.0f;
            {
                float perim = 0.0f;
                for (int e = 0; e < 4; e++) {
                    int b = (e + 1) & 3;
                    float dx = (float)(qr_result.corners[b].x -
                                       qr_result.corners[e].x);
                    float dy = (float)(qr_result.corners[b].y -
                                       qr_result.corners[e].y);
                    perim += sqrtf(dx * dx + dy * dy);
                }
                side_px = perim / 4.0f;
            }

            if (any_identified) {
                __atomic_add_fetch(&qr->frames_identified, 1, __ATOMIC_RELAXED);
            }
            if (any_decoded) {
                __atomic_add_fetch(&qr->qr_detections, 1, __ATOMIC_RELAXED);
                __atomic_add_fetch(&qr->total_decodes, 1, __ATOMIC_RELAXED);
                int modules = 4 * qr_result.data.version + 17;
                float px_per_mod = modules ? side_px / modules : 0.0f;
                qr->last_px_per_module = px_per_mod;
                qr->last_modules = (uint16_t)modules;
                /* Per-decode detail at DEBUG (win offset + passes make the lock
                 * behaviour visible); the clean INFO stream stays the 2s summary. */
                ESP_LOGD(TAG,
                         "QRPX v%d mod%d side%.0fpx %.2fpx/mod len%d pass%d off%+d %.24s",
                         qr_result.data.version, modules, side_px, px_per_mod,
                         qr_result.data.payload_len, astats.passes,
                         astats.win_offset, (const char *)qr_result.data.payload);
            } else if (any_identified) {
                /* Located but swept-and-failed: the whole offset ladder (+ the
                 * local pass under THOROUGH) could not resolve the modules. */
                __atomic_add_fetch(&qr->missed_codes, 1, __ATOMIC_RELAXED);
                frame_has_miss = true;
                frame_miss_err = (int)K_QUIRC_ERROR_DATA_ECC;
                frame_miss_side = side_px;
                ESP_LOGD(TAG, "QRMISS side%.0fpx pass%d lock%+d local%d",
                         side_px, astats.passes, astats.locked_offset,
                         (int)astats.used_local);
            }
            /* Adaptive-decode cost/behaviour readout (phase monitor). */
            __atomic_add_fetch(&qr->adaptive_passes, (uint32_t)astats.passes,
                               __ATOMIC_RELAXED);
            __atomic_add_fetch(&qr->adaptive_frames, 1, __ATOMIC_RELAXED);
            if (astats.used_local) {
                __atomic_add_fetch(&qr->adaptive_local, 1, __ATOMIC_RELAXED);
            }
            qr->last_locked_offset = astats.locked_offset;
#endif

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

#if CAM_QR_HASH_GATE
            /* Resolve the gate claim: record whether this frame's QR decoded (so a
             * future re-sample of it can be skipped), and -- for the safety-net
             * audit -- whether a FORCED decode netted a NEW payload, i.e. a real
             * content change the hash missed and the safety-net recovered (a gate
             * FN). g_forced/g_retry were set at the decision above. */
            {
                uint32_t g_sig = any_decoded
                    ? gate_payload_sig(qr_result.data.payload,
                                       qr_result.data.payload_len)
                    : 0;
                xSemaphoreTake(qr->g_lock, portMAX_DELAY);
                bool g_was_new = any_decoded && (g_sig != qr->g_last_sig);
                if (any_decoded) {
                    qr->g_caught   = true;
                    qr->g_last_sig = g_sig;
                }
                xSemaphoreGive(qr->g_lock);
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                if (g_forced) {
                    __atomic_add_fetch(&qr->g_force, 1, __ATOMIC_RELAXED);
                    if (g_was_new)
                        __atomic_add_fetch(&qr->g_force_new, 1, __ATOMIC_RELAXED);
                } else if (g_retry) {
                    __atomic_add_fetch(&qr->g_dec_retry, 1, __ATOMIC_RELAXED);
                } else {
                    __atomic_add_fetch(&qr->g_dec_chg, 1, __ATOMIC_RELAXED);
                }
                if (any_decoded) qr->g_ver = (uint16_t)qr_result.data.version;
#else
                (void)g_was_new; (void)g_forced; (void)g_retry;  /* instrumentation-only */
#endif
            }
#endif

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            /* Miss-frame capture (diagnostic): sample <=1/s a located-but-
             * undecoded frame's grayscale crop for offline analysis of WHY quirc
             * rejected it. Re-grayscale from the untouched RGB565 source (quirc
             * mutated qr_buf in place), tag with the miss error + measured size +
             * Laplacian sharpness + luma. The lock spans the grayscale so poll's
             * copy-out can't tear against it (~40 ms held, at most once/second). */
            if (qr->num_decoders <= 1 &&
                outcome == CAM_QR_MISS && qr->miss_capture && qr->miss_lock) {
                int64_t mnow = esp_timer_get_time();
                if (mnow - qr->miss_last_capture_us >= 1000000 &&
                    xSemaphoreTake(qr->miss_lock, 0) == pdTRUE) {
                    rgb565_to_grayscale_cropped(frame, qr->miss_capture,
                                                qr->frame_width, qr->crop_size,
                                                qr->crop_offset_x,
                                                qr->crop_offset_y,
                                                qr->rgb565_gray_lut);
                    uint8_t luma = 0;
                    float sharp = rgb565_focus_energy_cropped(
                        frame, qr->frame_width, qr->crop_size,
                        qr->crop_offset_x, qr->crop_offset_y,
                        qr->rgb565_gray_lut, &luma);
                    qr->miss_meta.seq = ++qr->miss_seq;
                    qr->miss_meta.timestamp_us = mnow;
                    qr->miss_meta.quirc_err = frame_has_miss ? frame_miss_err : -1;
                    qr->miss_meta.side_px = frame_has_miss ? frame_miss_side : 0.0f;
                    qr->miss_meta.sharpness = sharp;
                    qr->miss_meta.luma_mean = luma;
                    qr->miss_meta.width = qr->crop_size;
                    qr->miss_meta.height = qr->crop_size;
                    qr->miss_ready = true;
                    qr->miss_last_capture_us = mnow;
                    xSemaphoreGive(qr->miss_lock);
                }
            }
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

        /* Released early in dual-decode mode (right after grayscale); otherwise
         * (single decoder, or k_quirc_begin returned NULL) release here. */
        if (!frame_released) {
            cam_pipeline_release_frame(qr->pipeline);
        }
    }

    if (arg->done_sem) {
        xSemaphoreGive(arg->done_sem);
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
    /* on_frame is required for scanning; focus-assist mode has no decode result
     * to deliver (the metric is polled), so it is optional there. */
    if (!config || !config->pipeline ||
        (!config->on_frame && !config->focus_assist)) {
        ESP_LOGE(TAG, "Invalid config: pipeline (and on_frame unless focus_assist) required");
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
    qr->focus_assist = config->focus_assist;

    /* Parallel decoders: clamp to [1,2]; focus-assist is always single (nothing to
     * parallelize -- the metric is a fast per-frame pass, not a quirc decode). */
    qr->num_decoders = config->num_decoders < 1 ? 1
                     : (config->num_decoders > 2 ? 2 : config->num_decoders);
    if (qr->focus_assist) {
        qr->num_decoders = 1;
    }

    /* Compute centered square crop (use shorter dimension) */
    uint32_t w = config->frame_width;
    uint32_t h = config->frame_height;
    qr->crop_size = (w < h) ? w : h;
    qr->crop_offset_x = (w - qr->crop_size) / 2;
    qr->crop_offset_y = (h - qr->crop_size) / 2;

    // Build RGB565->grayscale LUT (64KB, SPIRAM). Used by the scan grayscale, the
    // focus-assist Laplacian, and the content-change gate hash; non-fatal if it
    // fails (all fall back to per-pixel luma).
    qr->rgb565_gray_lut = rgb565_lut_build();

#if CAM_QR_HASH_GATE
    // Content-change gate: mutex guarding the shared decision state (both decoders).
    qr->g_lock = xSemaphoreCreateMutex();
    if (!qr->g_lock) {
        ESP_LOGE(TAG, "Failed to create content-gate mutex");
        goto error;
    }
#endif

    // Focus-assist mode skips quirc entirely, so no decoder is allocated (saves
    // its ~crop_size^2 image buffers + identify structures).
    if (!qr->focus_assist) {
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
    }

    qr->task_done_sem = xSemaphoreCreateBinary();
    if (!qr->task_done_sem) {
        ESP_LOGE(TAG, "Failed to create task done semaphore");
        goto error;
    }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    qr->last_log_time = esp_timer_get_time();

    /* Miss-frame capture buffers (diagnostic). Two crop-sized grayscale buffers
     * in PSRAM (decode-task capture + poll copy-out) + a mutex. Non-fatal: if the
     * alloc fails, capture stays disabled (miss_capture NULL) and the scan runs
     * normally. */
    size_t miss_bytes = (size_t)qr->crop_size * qr->crop_size;
    qr->miss_capture = heap_caps_malloc(miss_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    qr->miss_poll    = heap_caps_malloc(miss_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    qr->miss_lock    = xSemaphoreCreateMutex();
    if (!qr->miss_capture || !qr->miss_poll || !qr->miss_lock) {
        ESP_LOGW(TAG, "miss-frame capture disabled (alloc failed, %u B each)",
                 (unsigned)miss_bytes);
        if (qr->miss_capture) { heap_caps_free(qr->miss_capture); qr->miss_capture = NULL; }
        if (qr->miss_poll)    { heap_caps_free(qr->miss_poll);    qr->miss_poll = NULL; }
        if (qr->miss_lock)    { vSemaphoreDelete(qr->miss_lock);  qr->miss_lock = NULL; }
    }
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
    qr->task_args[0] = (qr_decode_arg_t){
        .qr = qr, .dec = qr->qr_decoder,
        .done_sem = qr->task_done_sem, .primary = true,
    };
    BaseType_t result = xTaskCreatePinnedToCoreWithCaps(
        qr_decode_task, "qr_decode",
        CONFIG_CAM_PIPELINE_QR_TASK_STACK_SIZE, &qr->task_args[0],
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

    /* 2nd decoder on the second core (num_decoders == 2). Its own k_quirc instance
     * + done-sem. Pinned to core 0, which the portrait DSI scan frees (no per-frame
     * rotate). Non-fatal: if any alloc/create fails, the scan runs with the single
     * decoder. num_decoders is already forced to 1 in focus-assist mode. */
    if (qr->num_decoders > 1) {
        qr->qr_decoder2 = k_quirc_new();
        if (qr->qr_decoder2 &&
            k_quirc_resize(qr->qr_decoder2, qr->crop_size, qr->crop_size) < 0) {
            k_quirc_destroy(qr->qr_decoder2);
            qr->qr_decoder2 = NULL;
        }
        qr->task_done_sem2 = xSemaphoreCreateBinary();
        if (qr->qr_decoder2 && qr->task_done_sem2) {
            qr->task_args[1] = (qr_decode_arg_t){
                .qr = qr, .dec = qr->qr_decoder2,
                .done_sem = qr->task_done_sem2, .primary = false,
            };
            BaseType_t r2 = xTaskCreatePinnedToCoreWithCaps(
                qr_decode_task, "qr_decode2",
                CONFIG_CAM_PIPELINE_QR_TASK_STACK_SIZE, &qr->task_args[1],
                CONFIG_CAM_PIPELINE_QR_TASK_PRIORITY,
                &qr->task_handle2, 0,   /* core 0 -- free during the portrait scan */
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (r2 != pdPASS) {
                ESP_LOGW(TAG, "2nd decode task create failed -- single decoder");
                qr->task_handle2 = NULL;
            } else {
                ESP_LOGI(TAG, "2nd decoder created on core 0 (parallel decode)");
            }
        } else {
            ESP_LOGW(TAG, "2nd decoder alloc failed -- single decoder");
            if (qr->qr_decoder2)   { k_quirc_destroy(qr->qr_decoder2);  qr->qr_decoder2 = NULL; }
            if (qr->task_done_sem2){ vSemaphoreDelete(qr->task_done_sem2); qr->task_done_sem2 = NULL; }
        }
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

bool cam_pipeline_qr_get_focus_metric(cam_pipeline_qr_handle_t handle,
                                      cam_pipeline_qr_focus_t *out) {
    if (!out) {
        return false;
    }
    /* Always compiled -- focus-assist is a feature, not debug instrumentation.
     * Valid only for a focus-assist consumer with >=1 frame processed. */
    if (!handle || !handle->focus_assist || handle->focus_frames == 0) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    out->sharpness = handle->focus_ema;
    out->peak      = handle->focus_peak;
    out->raw       = handle->focus_raw;
    out->luma_mean = handle->focus_luma;
    out->usable    = handle->focus_usable;
    out->frames    = handle->focus_frames;
    return true;
}

bool cam_pipeline_qr_poll_miss_frame(cam_pipeline_qr_handle_t handle,
                                     const uint8_t **out_buf, size_t *out_len,
                                     cam_pipeline_qr_miss_meta_t *meta) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    if (!handle || !out_buf || !out_len || !meta ||
        !handle->miss_capture || !handle->miss_lock) {
        return false;
    }
    bool got = false;
    xSemaphoreTake(handle->miss_lock, portMAX_DELAY);
    if (handle->miss_ready) {
        size_t n = (size_t)handle->miss_meta.width * handle->miss_meta.height;
        memcpy(handle->miss_poll, handle->miss_capture, n);
        *meta = handle->miss_meta;
        *out_buf = handle->miss_poll;
        *out_len = n;
        handle->miss_ready = false;
        got = true;
    }
    xSemaphoreGive(handle->miss_lock);
    return got;
#else
    (void)handle; (void)out_buf; (void)out_len; (void)meta;
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

    /* Same orderly-shutdown handshake for the 2nd decoder, when present (closing was
     * set above, so its loop is already exiting). All NULL in single-decoder mode. */
    if (qr->task_handle2 && qr->task_done_sem2) {
        if (xSemaphoreTake(qr->task_done_sem2, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "timeout waiting for 2nd QR decode task; forcing delete");
        }
        vTaskDeleteWithCaps(qr->task_handle2);
        qr->task_handle2 = NULL;
    }
    if (qr->task_done_sem2) {
        vSemaphoreDelete(qr->task_done_sem2);
        qr->task_done_sem2 = NULL;
    }
    if (qr->qr_decoder2) {
        k_quirc_destroy(qr->qr_decoder2);
        qr->qr_decoder2 = NULL;
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

#if CAM_QR_HASH_GATE
    if (qr->g_lock) {
        vSemaphoreDelete(qr->g_lock);
        qr->g_lock = NULL;
    }
#endif

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    if (qr->miss_capture) { heap_caps_free(qr->miss_capture); qr->miss_capture = NULL; }
    if (qr->miss_poll)    { heap_caps_free(qr->miss_poll);    qr->miss_poll = NULL; }
    if (qr->miss_lock)    { vSemaphoreDelete(qr->miss_lock);  qr->miss_lock = NULL; }
#endif

    free(qr);
    ESP_LOGI(TAG, "QR consumer destroyed");
}
