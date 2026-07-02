# A fast frame consumer can collapse the display pipeline via PSRAM contention

## Symptom

With the image-entropy consumer (`cam_pipeline_entropy`) attached, on-device (ESP32-P4,
Waveshare LCD 4.3, 480×480 square, OV5647) the live preview ran at only **~4.5 fps** display /
~9 fps camera, and `CAM PPA` timing showed **~206 ms per PPA scale/rotate op** — roughly 15–20×
slower than the PPA hardware should need for a 1280×960 → 480×480 RGB565 transform.

The QR consumer never triggered this because it is *slower* than the camera (grayscale +
quirc), so it locks/processes at well under the camera rate.

## Root cause

The entropy consumer's frame loop had no "is this a new frame?" check. It called
`cam_pipeline_lock_frame()` in a tight loop and SHA-256'd the full RGB565 front buffer every
iteration. Being far faster than the camera, it re-locked and **re-hashed the same unchanged
front buffer ~125×/s** — about **57 MB/s of PSRAM reads** (460,800 bytes × 125).

That saturated PSRAM bandwidth. The camera pipeline's PPA op reads the ~2.4 MB camera frame and
writes the 460 KB output through the same PSRAM, so it stalled — 206 ms/op — which capped the
whole camera→PPA→display path at ~4.5 fps. It was **memory-bandwidth contention, not CPU
contention** (the consumer is pinned to Core 1, the camera path to Core 0; priorities don't
cross cores, so lowering task priority would not have helped).

Re-hashing duplicates also adds **zero entropy** (deterministic) and inflates the
`frames_chained` count so it no longer means "distinct frames."

## Fix

Add a **frame-generation gate** so a consumer only processes distinct frames:

- Pipeline core: a `volatile uint32_t frame_generation` in `struct cam_pipeline`, incremented
  under `buffer_mutex` on each promote (new front frame), and
  `cam_pipeline_lock_frame_gen(handle, uint32_t *gen)` that reports the locked frame's
  generation atomically with the lock (`cam_pipeline_lock_frame()` is now a wrapper passing
  NULL). See `src/esp_cam_pipeline.c`, `include/esp_cam_pipeline.h`.
- Consumer: record the last generation hashed; if a lock returns the same generation, release
  immediately and `vTaskDelay` a few ms instead of hashing. See
  `components/cam_pipeline_entropy/src/cam_pipeline_entropy.c`.

## Result

Display **4.5 → ~15 fps**, camera **9 → 30 fps**, PPA **206 ms → ~40 ms** (5×). The consumer
now hashes ~10 distinct frames/s (still locks ~170×/s, but the duplicate locks are cheap:
mutex + integer compare + release, no 460 KB hash). Frame chaining and capture/freeze behavior
are unchanged and still correct.

## Takeaway

Any consumer that is faster than the camera and touches the whole frame (hashing, copying,
full-frame analysis) must gate on `cam_pipeline_lock_frame_gen()` — otherwise it will re-process
the same buffer and can starve the display via PSRAM bandwidth, even while sitting on a separate
core. The generation counter (init 0; first real frame is generation 1) also naturally skips the
uninitialized initial front buffer.
