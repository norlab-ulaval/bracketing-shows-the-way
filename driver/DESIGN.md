# Driver design

C++17 library driving a hardware-synchronized pair of Basler a2A1920-51gcPRO
GigE cameras with a software exposure controller implementing the paper's
adaptive bracketing policy, **B-Adapt** (three exposures: low, mid, high).
The ROS 2 wrapper lives in `../driver_ros`.

## Components

| File | Role |
|---|---|
| `CameraGate` | RAII wrapper around one pylon camera (PIMPL: only `CameraGate.cpp` includes pylon); device enumeration. |
| `ExposureTrack` | One zone's exposure/gain controller (EMA, exposure/gain sequencing, convergence). |
| `ZoneManager` | One `ExposureTrack` per zone; frame-to-zone attribution (`closestZone`). |
| `BracketingSequencer` | Zone order: ping-pong `0,1,…,N-1,…,1`. |
| `BracketingDriver` | Bootstrap, lockstep capture loop, measurement, B-Adapt control, callbacks. |
| `Config` | `DriverConfig` and `validate()`. |

The camera's own auto-exposure is disabled (`ExposureAuto`/`GainAuto = Off`)
because it cannot switch targets every frame. Gamma is forced to 1.0 and
`BslColorSpace` to Off, so frames are linear raw BayerRG8.

## Camera selection

`camera_1_serial` / `camera_2_serial` select the cameras: both set = stereo,
only `camera_1_serial` = mono (see below), only `camera_2_serial` = a
configuration error. When both are empty, the ROS node calls
`CameraGate::enumerateDevices()` (pylon `CTlFactory::EnumerateDevices`)
before constructing the driver. With one camera visible it runs mono with
it. With two, they are sorted by serial number and the lowest becomes
camera_1 (left, trigger master); the choice is logged together with the
serials to set to swap them. Any other count (0 or more than 2) is a fatal
error listing the cameras found (serial, model, IP).

## Mono mode

With `camera2SerialNumber` empty (after auto-detection) the driver runs
camera_1 alone, for B-Adapt acquisition only: BVSLAM needs the stereo pair.
camera_1 is driven exactly as in stereo (same bootstrap, software trigger,
exposure/gain sequence, measurement, controller updates and frame metadata);
everything involving camera_2 is skipped: it is never connected or written,
there is no trigger-ready wait or grab, camera_1's `ExposureActive` is not
routed to an output line, and the camera_2 callback is never called. The
one difference in output: stereo drops camera_1's frame when camera_2's
grab fails (pairs only), mono always delivers it.

## Stereo hardware trigger and lockstep capture

(Mono runs steps 1, 2, the trigger of 3, camera_1's grab of 4, 5 and the
sequencer advance of 6.)

camera_1 (left) is software-triggered. Its `ExposureActive` signal is routed
to an output line (`Line3`), wired to camera_2's (right) trigger input line
(`Line3`, rising edge, `TriggerOverlap = ReadOut`). Both cameras therefore
start exposing together, with the same zone's exposure/gain.

One capture step (main thread):

1. Read the current zone's exposure/gain and write them to camera_1.
2. Sleep to hold the `acquisition_frame_rate` cadence (host-side pacing; a
   triggered camera ignores its own frame-rate node).
3. Wait until camera_2 is ready for a trigger, fire camera_1's software trigger.
4. Grab camera_1, then camera_2. If camera_1 fails, camera_2's frame is
   drained and discarded so the pair stays aligned.
5. Timestamp (host `system_clock`, after both grabs) and hand the frame pair
   to the processing thread.
6. Advance the sequencer and write the *next* zone's exposure/gain to camera_2
   now: writing camera_2's registers right before a trigger can make it miss
   the pulse, so camera_2 is always programmed one step ahead.

A background processing thread measures camera_1's frame, updates the
controllers and invokes the callbacks while the main thread grabs the next
pair. Pixel buffers are double-buffered with a single job in flight
(back-pressure); the zone state is protected by a mutex. Only camera_1's
frames feed the controllers.

Each frame is attributed to a zone by `ZoneManager::closestZone()`: the zone
whose current exposure estimate is closest to the frame's chunk-data
exposure, with the requested zone breaking ties (within
`exposure_tie_tolerance_us`).

## Bootstrap (AE-Once seeding)

Both cameras are connected and camera_1 runs the camera's own
`ExposureAuto = Once`. The settled exposure (clamped to the limits) seeds
every zone's exposure; gain starts at `min_gain_db`. camera_1 is then switched to software trigger, camera_2's queue is
flushed, and capture starts.

## Measurement

Frames are sampled as 2x2 Bayer blocks (every `zone_sort_stride` blocks),
each reduced to `gray = (R + (G1+G2)/2 + B) / 3`. Low measures the
`ae_bridge_low_percentile` value of the whole frame, high the
`ae_bridge_high_percentile` value (mid's median is informational only).

## Controller (`ExposureTrack`)

With target `B*` and measurement `B`:

```
B' = 255 * (1 + ae_saturation_boost_fraction)   if B >= 255   (saturation boost)
   = max(B, 0.5)                                 otherwise
r  = B* / B'
```

The saturation boost exists because a clipped reading carries no magnitude:
with a near-white target (250) a fully clipped frame would otherwise give
`r = 250/255` and barely move.

Every move is a fixed-rate EMA, `new = (1 - λ) · old + λ · proposal`, with
λ = `ae_zone_alpha[zone]`. Exposure and gain are strictly sequenced so only
one of them moves in a given frame:

* **Need more light (r ≥ 1), exposure first.** Proposal
  `E' = clamp(E·r, min_exposure_us, max_exposure_us)`. Gain rises only when
  exposure saturates, with proposal `clamp(G + 20·log10(E·r / E'))`;
  otherwise the gain proposal is `min_gain_db`.
* **Need less light (r < 1), gain released first.** While gain is more than
  0.05 dB above `min_gain_db`, exposure is frozen and only gain moves (proposal
  `clamp(G + 20·log10 r)`). Once gain is at the floor it snaps to it and
  exposure moves (proposal `clamp(E·r)`).

This keeps gain (noise) minimal and avoids double-applying a correction. A
zone is *converged* when its last three readings are within `epsilon`
(relative) of the target, and *unreachable* when it is not converged with
both exposure and gain pinned at a bound.

## B-Adapt (N = 3)

* **Low (zone 0)** targets `ae_bridge_low_percentile → ae_bridge_low_target`
  on its own frame (99th percentile → 250: "no pixel overexposed").
* **High (zone 2)** targets `ae_bridge_high_percentile → ae_bridge_high_target`
  on its own frame (1st percentile → 5: "no pixel underexposed").
* Low and high are updated only on their own frames, with the controller above.
* **Δmin floor.** After every frame, if the separation of low's and high's
  total light `T = E · 10^(G/20)` is below `ae_bridge_min_stops_gap` stops,
  both are set to `midpoint ∓ Δmin/2` (in stops) directly, bypassing the EMA.
* **Mid (zone 1)** has no controller. Every frame it is set to the geometric
  mean `sqrt(T_low · T_high)`, split exposure-first with minimum gain within
  the limits.
* Targets are passed to the controller as integer percentages of 255, so the
  effective targets are 249.9 (98 %) and 5.1 (2 %).
* With the ping-pong order `low, mid, high, mid`, every frame is adjacent to a
  mid (bridge) frame and low and high are each updated once per period.

## Known limitations

* Attribution uses the zone estimates at processing time, which the
  processing thread may already have moved since the frame was triggered.
  During fast transients a frame can therefore be labeled (and fed back) as a
  different zone than the one requested. A tie resolves to the requested
  zone, or to the highest tied index if the requested zone is not among them.
* The exposure of frame *t+1* is normally chosen before frame *t* has been
  processed (one frame of control latency; the order is not enforced).
* B-Adapt supports exactly three zones (`ae_zone_count: 3`).
* `gain_db` is the programmed value, not read back from the camera.
* There is no reconnect logic: a camera error ends `run()` with an exception.
