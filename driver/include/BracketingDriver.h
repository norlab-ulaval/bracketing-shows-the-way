#pragma once
#include "CameraGate.h"
#include "Config.h"
#include "ZoneManager.h"
#include "BracketingSequencer.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

/**
 * Per-frame data delivered to the frame callbacks. The pixel pointer is
 * valid only for the duration of the callback.
 */
struct CapturedFrame {
    const uint8_t* pixels;           ///< Row-major BayerRG8, 1 byte/pixel, configured width x height.
    size_t         size;             ///< Byte count = width * height.
    int64_t        timestamp_ns;     ///< Host system_clock time taken right after both grabs returned.
    double         exposure_used_us; ///< Exposure actually applied to this camera's frame (chunk data).
    double         gain_db;          ///< Gain programmed on THIS camera for this frame (dB). camera_2's
                                     ///< registers are preset one step ahead, so its value is the one
                                     ///< written at the previous step's tail. Not read back from the camera.
    double         brightness;       ///< camera_1's AE measurement for this frame's zone (0..255): the
                                     ///< zone's tail-percentile value of the whole frame (low:
                                     ///< ae_bridge_low_percentile, mid: 50th, high: ae_bridge_high_percentile).
                                     ///< camera_2 frames carry camera_1's value.
    int            ae_zone;          ///< Zone index 0..N-1 the frame was attributed to (closestZone()).
    uint8_t        sequence_index;   ///< Position in the ping-pong cycle 0,1,2,1: 0..3.
    bool           converged;        ///< This zone's track has converged (mid: low and high both converged).
    bool           track_unreachable;///< Target unreachable with exposure and gain pinned at a bound.
};

/// Frame delivery callback, invoked from the background processing thread.
using FrameCallback = std::function<void(const CapturedFrame&)>;

/**
 * @file BracketingDriver.h
 *
 * Top-level stereo bracketing driver. See DESIGN.md.
 *
 * camera_1 is software-triggered once per step; its ExposureActive output
 * hardware-triggers camera_2, so both expose simultaneously with the same
 * zone's exposure/gain (camera_2's registers are written one step ahead).
 * Only camera_1's frames feed the AE controllers.
 *
 * Mono mode (cfg.camera2SerialNumber empty): camera_1 alone, driven by the
 * same steps; camera_2 is never connected, written or delivered.
 *
 * Threads: the caller's thread runs run() (trigger + grab, lockstep); a
 * background thread measures, updates the zone controllers and invokes the
 * callbacks, overlapping with the next grab. Pixel buffers are
 * double-buffered with single-job back-pressure; zones_ is guarded by
 * tracksMutex_.
 */
class BracketingDriver {
public:
    /// Store the config and allocate buffers. Does not touch the cameras.
    /// The config must already pass validate().
    explicit BracketingDriver(const DriverConfig& cfg);

    /// Stop the processing thread and disconnect both cameras.
    ~BracketingDriver();

    /// Callback for each camera_1 frame (stereo: only when both cameras delivered). Set before run().
    void setCamera1Callback(FrameCallback cb);

    /// Callback for each camera_2 frame (only when both cameras delivered; never in mono). Set before run().
    void setCamera2Callback(FrameCallback cb);

    /**
     * Connect the camera(s), run the AE-Once bootstrap, then capture until
     * running becomes false. Blocks.
     *
     * A failed grab skips that step (warning on stderr) and the loop
     * continues; a missing or disconnected camera throws.
     *
     * @throws Pylon::GenericException on camera errors (e.g. camera not found).
     */
    void run(const std::atomic<bool>& running);

private:
    /// Connect and configure the camera(s), AE-Once seed, arm the triggers.
    void bootstrap();

    /// One lockstep step: set exposure/gain, trigger, grab (both), hand off.
    void captureStep();

    /// Metadata for one grabbed step, handed to the processing thread.
    struct ProcessingJob {
        int     bufIdx;
        int     requestedZone;
        int     stepIndex;
        double  actualExposureUsCam1;
        double  actualExposureUsCam2;
        double  gainDb;         ///< camera_1's gain for this frame.
        double  camera2GainDb;  ///< camera_2's gain for this frame (written one step earlier).
        bool    cam2Grabbed;
        int64_t timestampNs;
    };

    /// Wait until the previous job is done, then hand off a new one.
    void submitProcessingJob(const ProcessingJob& job);

    /// Background thread body.
    void processingLoop();

    /// Measure, update the zone controllers, invoke callbacks.
    void processJob(const ProcessingJob& job);

    /// Brightness statistics of one zone, measured on the whole frame.
    struct ZoneMeasurement {
        double  mean;         ///< Percentile value (named "mean" for the verbose log).
        uint8_t minVal;
        uint8_t maxVal;
        size_t  sampleCount;
    };

    /**
     * Sample the frame as 2x2 BayerRG8 blocks (every `stride` blocks in x
     * and y) and reduce each to gray = (R + (G1 + G2) / 2 + B) / 3.
     */
    static std::vector<uint8_t> sampleGrayBlocks(const uint8_t* pixels, int width, int height, int stride);

    /// Whole-frame value at `percentile` (0..100) of the sampled blocks (stored in .mean).
    static ZoneMeasurement measurePercentileBrightness(const uint8_t* pixels, int width, int height,
                                                          int stride, double percentile);

    /// Per-zone target percent for ZoneManager: the bridge targets of low
    /// and high as % of pixelMax (clamped to [1, 99], rounded); mid's entry
    /// (50) is unused since mid is derived.
    static std::vector<int> computeZoneTargetPercents(const DriverConfig& cfg);

    struct DerivedBridge {
        double exposureUs;
        double gainDb;
        bool   unreachable;  ///< Gain clamped with a residual correction left.
    };

    /**
     * Mid zone: geometric mean of low's and high's total
     * light (exposure * 10^(gain/20)), split exposure-first / minimum-gain
     * within the limits.
     */
    static DerivedBridge deriveBridgeMid(
            double lowExposureUs, double lowGainDb,
            double highExposureUs, double highGainDb,
            double minExp, double maxExp, double minGain, double maxGain);

    /// exposureUs * 10^(gainDb / 20).
    static double totalLight(double exposureUs, double gainDb);

    /// Exposure-first, minimum-gain split of a total light value, clamped.
    static DerivedBridge splitTotalMinGain(
            double total, double minExp, double maxExp, double minGain, double maxGain);

    /// Nanoseconds since the Unix epoch (system_clock).
    static int64_t nowNs();

    CameraGate camera1_;
    CameraGate camera2_;
    DriverConfig cfg_;
    const bool mono_;          ///< camera_1 only (camera2SerialNumber empty).
    BracketingSequencer seq_;  ///< Ping-pong zone order, drives both cameras.
    const int midZone_;        ///< N / 2 = 1; the derived zone.
    ZoneManager zones_;        ///< Fed only by camera_1's measurements.

    FrameCallback camera1Callback_;
    FrameCallback camera2Callback_;

    /// Double-buffered grab targets; bufIdx_ alternates each step.
    std::vector<uint8_t> camera1PixelBuf_[2];
    std::vector<uint8_t> camera2PixelBuf_[2];
    int bufIdx_ = 0;

    /// Gain currently programmed on camera_2 (main thread only).
    double camera2GainDb_ = 0.0;

    /// Next trigger time for acquisitionFrameRate pacing (main thread only).
    std::chrono::steady_clock::time_point nextTriggerTime_{};

    // Background processing pipeline.
    std::thread             processingThread_;
    std::mutex              jobMutex_;
    std::condition_variable jobCv_;
    bool                    jobInFlight_ = false;
    bool                    stopProcessing_ = false;
    ProcessingJob           pendingJob_{};
    std::mutex              tracksMutex_;
};
