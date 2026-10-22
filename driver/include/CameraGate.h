#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * @file CameraGate.h
 *
 * Thin RAII wrapper around the Basler pylon SDK for one a2A1920-51gcPRO GigE
 * camera. Pylon types are hidden behind a PIMPL; only CameraGate.cpp
 * includes pylon. Pylon::PylonInitialize() must be called before
 * construction and Pylon::PylonTerminate() after destruction.
 * Pylon::GenericException propagates from all methods.
 */
class CameraGate {
public:
    CameraGate();
    ~CameraGate();  ///< Calls disconnect().

    /// One camera found by enumerateDevices().
    struct DeviceInfo {
        std::string serial;
        std::string model;
        std::string ip;  ///< "" if not a GigE device.
    };

    /// All Basler cameras pylon can see, sorted by serial number ascending
    /// (numerically for digit-only serials). Pylon must be initialized.
    static std::vector<DeviceInfo> enumerateDevices();

    /**
     * Open the camera and apply the fixed configuration: UserSet Default,
     * ExposureAuto/GainAuto Off, Gamma 1.0 and BslColorSpace Off (linear raw
     * BayerRG8), initial Gain, GigE transport settings, centered AOI crop,
     * and ExposureTime chunk data (so grab() reports the exposure actually
     * used per frame).
     *
     * @param serialNumber          Camera serial; empty = first device found.
     * @param heartbeatTimeoutMs    GevHeartbeatTimeout.
     * @param packetSizeBytes       GevSCPSPacketSize (1440 fits a 1500 MTU).
     * @param gainDb                Initial gain (= min_gain_db).
     * @param interPacketDelayTicks GevSCPD (1 tick = 8 ns), written explicitly.
     * @param numGrabBuffers        Host-side grab buffers.
     * @param frameRetentionMs      Stream-grabber frame retention, -1 = default.
     * @param outputWidth           Output width; < 1920 crops a centered AOI.
     * @param outputHeight          Output height; < 1200 crops a centered AOI.
     * @param bandwidthReserveMode  BandwidthReserveMode, "" = camera default.
     *
     * @throws Pylon::GenericException if no matching camera is found.
     */
    void connect(const std::string& serialNumber    = "",
                int heartbeatTimeoutMs             = 5000,
                int packetSizeBytes                = 1440,
                double gainDb                      = 0.0,
                int interPacketDelayTicks          = 0,
                int numGrabBuffers                 = 10,
                int frameRetentionMs               = -1,
                int outputWidth                    = 1920,
                int outputHeight                   = 1200,
                const std::string& bandwidthReserveMode = "");

    /// Stop grabbing and close the camera. Safe to call repeatedly.
    void disconnect();

    /**
     * Start grabbing.
     *
     * @param latestOnly  true: GrabStrategy_LatestImageOnly (free-running
     *                    AE-Once phase). false: GrabStrategy_OneByOne
     *                    (triggered capture, keeps every triggered frame).
     */
    void startAcquisition(bool latestOnly = true);

    /// Stop grabbing. Safe to call when not grabbing.
    void stopAcquisition();

    /**
     * camera_1: route lineSource (ExposureActive) to output lineSelector so
     * it hardware-triggers camera_2. Call after connect(), before
     * startAcquisition().
     */
    void configureTriggerOutput(const std::string& lineSelector = "Line3",
                                const std::string& lineSource   = "ExposureActive");

    /**
     * camera_2: FrameStart trigger from input lineSelector (TriggerMode On,
     * TriggerOverlap ReadOut if available). A grab() timeout afterwards
     * usually means the trigger cable is disconnected.
     */
    void configureHardwareTrigger(const std::string& lineSelector      = "Line3",
                                  const std::string& triggerActivation = "RisingEdge",
                                  const std::string& triggerSource     = "Line3");

    /// BalanceWhiteAuto Off and per-channel BalanceRatio (skipped on mono cameras).
    void setWhiteBalance(double r, double g, double b);

    /// camera_1: FrameStart trigger from software (executeSoftwareTrigger()).
    void configureSoftwareTrigger();

    /// Block until the camera can accept the next frame trigger.
    /// @throws Pylon::GenericException on timeout.
    void waitForTriggerReady(unsigned int timeoutMs);

    /// Fire one software trigger (waits up to 5 s for trigger-ready first).
    void executeSoftwareTrigger();

    /// Set ExposureTime (µs); skipped if within 0.5 µs of the last value written.
    void setExposure(double us);

    /// Set Gain (dB, clamped to the camera range); skipped if within 0.001 dB
    /// of the last value written.
    void setGain(double db);

    /**
     * Run the camera's ExposureAuto = Once and return the settled
     * ExposureTime (µs). Acquisition must be running. Grabs until the camera
     * resets ExposureAuto to Off or maxIters frames have been grabbed.
     */
    double runAEOnce(int maxIters = 100, unsigned int timeoutMs = 5000, int pollIntervalMs = 5);

    /**
     * Grab one frame into dst (row-major BayerRG8, 1 byte/pixel).
     *
     * @param actualExposureUs  Out: exposure (µs) from chunk data, -1 if absent.
     * @return false on timeout or an incomplete frame (dst unmodified).
     * @throws std::runtime_error if dstSize is smaller than the frame.
     */
    bool grab(uint8_t* dst, size_t dstSize, double& actualExposureUs, unsigned int timeoutMs = 5000);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
