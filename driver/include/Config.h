#pragma once
#include <string>
#include <vector>

/**
 * @file Config.h
 *
 * DriverConfig holds every tunable parameter of the driver. The ROS 2 node
 * fills it from ROS parameters of the same (snake_case) name and calls
 * validate() before constructing BracketingDriver. See DESIGN.md for the
 * control law these fields parameterize.
 */

/**
 * The exposure policy is B-Adapt (paper): three zones. Zone 0 (low) and
 * zone 2 (high) run closed-loop control on a tail percentile of their own
 * frame; zone 1 (mid) is derived every cycle as the geometric midpoint of
 * low's and high's exposure*gain. See DESIGN.md.
 */
struct DriverConfig {
    // ── B-Adapt ─────────────────────────────────────────────────────────────
    int aeZoneCount = 3;  ///< Number of zones N (bracket length). Must be 3.

    /// Sampling stride in 2x2 Bayer blocks (x and y) for brightness
    /// measurement. 1 = every block.
    int zoneSortStride = 1;

    double aeBridgeLowPercentile  = 99.0;   ///< Low zone's own tail percentile, in (0, 100) (100 - alpha).
    double aeBridgeLowTarget      = 250.0;  ///< Target value for that percentile, in [0, pixelMax] (255 - beta).
    double aeBridgeHighPercentile = 1.0;    ///< High zone's own tail percentile, in (0, 100) (alpha).
    double aeBridgeHighTarget     = 5.0;    ///< Target value for that percentile, in [0, pixelMax] (beta).
    double aeBridgeMinStopsGap    = 1.0;    ///< Floor on low/high separation in stops (Delta_min), >= 0.

    // ── ExposureTrack tuning ────────────────────────────────────────────────
    double pixelMax = 255.0;  ///< Max pixel value (BayerRG8).

    /// Per-zone EMA rate (lambda) in (0, 1], exactly aeZoneCount entries
    /// (low, mid, high; mid is derived and its entry is unused):
    /// new = (1 - lambda) * old + lambda * proposal.
    std::vector<double> aeZoneAlpha{0.8, 0.4, 0.8};

    double epsilon           = 0.02;      ///< Convergence threshold (fractional brightness error).
    double defaultExposureUs = 10'000.0;  ///< Exposure before the AE-Once seed.

    /// A clipped reading (>= pixelMax) is treated as pixelMax * (1 + this)
    /// so a saturated frame still yields a real corrective step. >= 0.
    double aeSaturationBoostFraction = 0.5;

    /// µs tolerance for ZoneManager::closestZone() ties.
    double exposureTieToleranceUs = 5.0;

    double minExposureUs = 19.0;
    double maxExposureUs = 5'000.0;
    double minGainDb = 0.0;
    double maxGainDb = 48.0;

    // ── Camera / GigE (per camera) ──────────────────────────────────────────
    int camera1HeartbeatTimeoutMs = 5000;
    int camera2HeartbeatTimeoutMs = 5000;
    int camera1PacketSizeBytes    = 1440;  ///< GevSCPSPacketSize.
    int camera2PacketSizeBytes    = 1440;
    int camera1InterPacketDelayTicks = 0;  ///< GevSCPD; written explicitly (factory default is 512).
    int camera2InterPacketDelayTicks = 0;
    std::string camera1BandwidthReserveMode;  ///< "" = camera default.
    std::string camera2BandwidthReserveMode;
    int camera1NumGrabBuffers = 10;
    int camera2NumGrabBuffers = 10;
    int camera1FrameRetentionMs = -1;  ///< -1 = Pylon default.
    int camera2FrameRetentionMs = -1;

    /// Host-side trigger pacing (Hz) of the lockstep capture loop. 0 = as
    /// fast as the hardware allows.
    double acquisitionFrameRate = 20.0;

    // ── AE-Once bootstrap ───────────────────────────────────────────────────
    int aeOnceMaxIters  = 100;
    int aeOnceTimeoutMs = 5000;
    int aeOncePollMs    = 5;

    // ── Grab timeouts ───────────────────────────────────────────────────────
    int grabTimeoutMs        = 5000;  ///< camera_1 (and camera_2 trigger-ready wait).
    int camera2GrabTimeoutMs = 5000;  ///< camera_2; a timeout usually means a trigger-cable fault.

    // ── Stereo rig ──────────────────────────────────────────────────────────
    /// camera_1 = left, software-triggered master; camera_2 = right,
    /// hardware-triggered slave. camera_2 empty = mono (camera_1 only).
    /// Both empty = auto-detect (the caller fills them from
    /// CameraGate::enumerateDevices() before constructing BracketingDriver).
    std::string camera1SerialNumber;
    std::string camera2SerialNumber;

    std::string triggerOutputLine   = "Line3";           ///< camera_1 output line.
    std::string triggerOutputSource = "ExposureActive";  ///< Signal routed to it.
    std::string triggerInputLine    = "Line3";           ///< camera_2 input line.
    std::string triggerSource       = "Line3";
    std::string triggerActivation   = "RisingEdge";

    /// White-balance ratios applied at connect time (BalanceWhiteAuto = Off).
    double camera1WbRed = 1.0, camera1WbGreen = 1.0, camera1WbBlue = 1.0;
    double camera2WbRed = 1.0, camera2WbGreen = 1.0, camera2WbBlue = 1.0;

    /// Output size; smaller than 1920x1200 crops a centered AOI (stays BayerRG8).
    int width  = 1920;
    int height = 1200;

    /// Print a per-frame AE log line to stdout. Does not affect control.
    bool verbose = false;
};

/**
 * Check a DriverConfig for consistency.
 *
 * @throws std::runtime_error if: ae_zone_count != 3; zone_sort_stride < 1;
 *         a bridge percentile outside (0, 100), a bridge target outside
 *         [0, pixel_max], or ae_bridge_min_stops_gap < 0;
 *         ae_saturation_boost_fraction < 0; ae_zone_alpha not having
 *         ae_zone_count entries in (0, 1]; camera_2_serial set
 *         without camera_1_serial; or width/height not positive, even, and <= 1920x1200.
 */
void validate(const DriverConfig& cfg);
