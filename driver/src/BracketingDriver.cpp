#include "BracketingDriver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

// ── helpers ────────────────────────────────────────────────────────────────

int64_t BracketingDriver::nowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(
        system_clock::now().time_since_epoch()).count();
}

std::vector<uint8_t> BracketingDriver::sampleGrayBlocks(
        const uint8_t* pixels, int width, int height, int stride) {
    // Whole 2x2 blocks, so every sample mixes R, G, G, B regardless of the
    // (always even) pixel stride.
    const int pixelStride = 2 * stride;
    std::vector<uint8_t> samples;
    samples.reserve(static_cast<size_t>((width / pixelStride + 1) * (height / pixelStride + 1)));
    for (int y = 0; y + 1 < height; y += pixelStride) {
        const uint8_t* row0 = pixels + y * width;
        const uint8_t* row1 = row0 + width;
        for (int x = 0; x + 1 < width; x += pixelStride) {
            const int r  = row0[x];
            const int g1 = row0[x + 1];
            const int g2 = row1[x];
            const int b  = row1[x + 1];
            const double gray = (r + 0.5 * (g1 + g2) + b) / 3.0;
            samples.push_back(static_cast<uint8_t>(std::lround(std::clamp(gray, 0.0, 255.0))));
        }
    }
    return samples;
}

BracketingDriver::ZoneMeasurement BracketingDriver::measurePercentileBrightness(
        const uint8_t* pixels, int width, int height, int stride, double percentile) {
    std::vector<uint8_t> samples = sampleGrayBlocks(pixels, width, height, stride);
    if (samples.empty()) return {0.0, 0, 0, 0};

    const size_t n = samples.size();
    size_t idx = static_cast<size_t>(std::lround(percentile / 100.0 * static_cast<double>(n - 1)));
    idx = std::min(idx, n - 1);
    std::nth_element(samples.begin(), samples.begin() + static_cast<long>(idx), samples.end());
    const double value = static_cast<double>(samples[idx]);

    const auto [minIt, maxIt] = std::minmax_element(samples.begin(), samples.end());
    return {value, *minIt, *maxIt, samples.size()};
}

std::vector<int> BracketingDriver::computeZoneTargetPercents(const DriverConfig& cfg) {
    // Zone 1 (mid)'s entry is unused: mid never calls update().
    const double lowPct  = std::clamp(cfg.aeBridgeLowTarget  / cfg.pixelMax * 100.0, 1.0, 99.0);
    const double highPct = std::clamp(cfg.aeBridgeHighTarget / cfg.pixelMax * 100.0, 1.0, 99.0);
    return {static_cast<int>(std::lround(lowPct)), 50, static_cast<int>(std::lround(highPct))};
}

double BracketingDriver::totalLight(double exposureUs, double gainDb) {
    // exposureUs is linear, gainDb is logarithmic, so 10^(gainDb/20)
    // converts gain to the same linear multiplier exposure already is.
    return exposureUs * std::pow(10.0, gainDb / 20.0);
}

BracketingDriver::DerivedBridge BracketingDriver::splitTotalMinGain(
        double total, double minExp, double maxExp, double minGain, double maxGain) {
    // Exposure-first, minimize-gain: how much raw exposure would supply
    // total on its own, with gain held at the floor. Only once that's
    // clamped does gain rise off the floor to cover the leftover.
    const double rawExpAtFloor = total / std::pow(10.0, minGain / 20.0);
    const double clampedExp    = std::clamp(rawExpAtFloor, minExp, maxExp);

    double gain = minGain;
    bool unreachable = false;
    if (clampedExp != rawExpAtFloor) {
        const double leftover  = rawExpAtFloor / clampedExp;
        const double idealGain = minGain + 20.0 * std::log10(leftover);
        gain = std::clamp(idealGain, minGain, maxGain);
        unreachable = (idealGain != gain);
    }
    return {clampedExp, gain, unreachable};
}

BracketingDriver::DerivedBridge BracketingDriver::deriveBridgeMid(
        double lowExposureUs, double lowGainDb,
        double highExposureUs, double highGainDb,
        double minExp, double maxExp, double minGain, double maxGain) {
    // Total light (brightness multiplier) each end zone is actually producing.
    const double totalLow  = totalLight(lowExposureUs, lowGainDb);
    const double totalHigh = totalLight(highExposureUs, highGainDb);

    // Geometric mean of the two totals -- mid's target combined brightness.
    const double desiredTotal = std::sqrt(totalLow * totalHigh);

    return splitTotalMinGain(desiredTotal, minExp, maxExp, minGain, maxGain);
}

// ── callback setters ───────────────────────────────────────────────────────

void BracketingDriver::setCamera1Callback(FrameCallback cb) { camera1Callback_ = std::move(cb); }
void BracketingDriver::setCamera2Callback(FrameCallback cb) { camera2Callback_ = std::move(cb); }

// ── construction / destruction ─────────────────────────────────────────────

BracketingDriver::BracketingDriver(const DriverConfig& cfg)
    : cfg_(cfg),
      mono_(cfg.camera2SerialNumber.empty()),
      seq_(cfg.aeZoneCount, /*pingPong=*/true),
      midZone_(cfg.aeZoneCount / 2),
      zones_(cfg.aeZoneCount, computeZoneTargetPercents(cfg),
             cfg.pixelMax, cfg.aeZoneAlpha, cfg.epsilon, cfg.defaultExposureUs,
             cfg.exposureTieToleranceUs,
             cfg.minExposureUs, cfg.maxExposureUs, cfg.minGainDb, cfg.maxGainDb,
             cfg.aeSaturationBoostFraction) {
    const size_t pixelBytes = static_cast<size_t>(cfg_.width) * static_cast<size_t>(cfg_.height);
    for (int i = 0; i < 2; ++i) {
        camera1PixelBuf_[i].resize(pixelBytes);
        if (!mono_)
            camera2PixelBuf_[i].resize(pixelBytes);
    }

    processingThread_ = std::thread([this]() { processingLoop(); });
}

BracketingDriver::~BracketingDriver() {
    {
        std::lock_guard<std::mutex> lock(jobMutex_);
        stopProcessing_ = true;
    }
    jobCv_.notify_all();
    if (processingThread_.joinable())
        processingThread_.join();

    camera1_.stopAcquisition();
    camera1_.disconnect();
    camera2_.stopAcquisition();
    camera2_.disconnect();
}

// ── bootstrap ─────────────────────────────────────────────────────────────

void BracketingDriver::bootstrap() {
    // camera_1: free-running first, so AE-Once can grab without triggers.
    std::cout << "[bootstrap] connecting to camera 1...\n";
    camera1_.connect(cfg_.camera1SerialNumber, cfg_.camera1HeartbeatTimeoutMs,
                     cfg_.camera1PacketSizeBytes, cfg_.minGainDb,
                     cfg_.camera1InterPacketDelayTicks, cfg_.camera1NumGrabBuffers,
                     cfg_.camera1FrameRetentionMs, cfg_.width, cfg_.height,
                     cfg_.camera1BandwidthReserveMode);
    if (!mono_)  // mono: no slave to drive, leave the output line alone
        camera1_.configureTriggerOutput(cfg_.triggerOutputLine, cfg_.triggerOutputSource);
    camera1_.setWhiteBalance(cfg_.camera1WbRed, cfg_.camera1WbGreen, cfg_.camera1WbBlue);
    camera1_.startAcquisition();

    // camera_2: armed now; frames it captures during AE-Once are discarded below.
    if (!mono_) {
        std::cout << "[bootstrap] connecting to camera 2...\n";
        camera2_.connect(cfg_.camera2SerialNumber, cfg_.camera2HeartbeatTimeoutMs,
                         cfg_.camera2PacketSizeBytes, cfg_.minGainDb,
                         cfg_.camera2InterPacketDelayTicks, cfg_.camera2NumGrabBuffers,
                         cfg_.camera2FrameRetentionMs, cfg_.width, cfg_.height,
                         cfg_.camera2BandwidthReserveMode);
        camera2_.configureHardwareTrigger(cfg_.triggerInputLine, cfg_.triggerActivation,
                                          cfg_.triggerSource);
        camera2_.setWhiteBalance(cfg_.camera2WbRed, cfg_.camera2WbGreen, cfg_.camera2WbBlue);
        camera2_.startAcquisition(false);  // OneByOne — hardware triggered
    }

    // AE-Once seed (all zones start from the camera's own mid-tone exposure).
    std::cout << "[bootstrap] running AE-Once on camera 1 to seed all " << cfg_.aeZoneCount << " zones...\n";
    double aeSeedUs = camera1_.runAEOnce(cfg_.aeOnceMaxIters, cfg_.aeOnceTimeoutMs,
                                         cfg_.aeOncePollMs);
    aeSeedUs = std::clamp(aeSeedUs, cfg_.minExposureUs, cfg_.maxExposureUs);
    std::cout << "[bootstrap] AE seed = " << aeSeedUs << " µs\n";
    zones_.seed(aeSeedUs, cfg_.minExposureUs, cfg_.maxExposureUs);

    // Switch camera_1 to software-triggered lockstep. Stopping it first
    // guarantees no ExposureActive pulses reach camera_2 meanwhile.
    camera1_.stopAcquisition();
    camera1_.configureSoftwareTrigger();

    if (!mono_) {
        // Flush the frames camera_2 captured during AE-Once (they would
        // otherwise exhaust its OneByOne queue) by restarting its acquisition.
        camera2_.stopAcquisition();
        camera2_.startAcquisition(false);   // OneByOne — hardware triggered, fresh buffers

        // Preset camera_2 for the first step (afterwards set at each step's tail).
        camera2_.setExposure(zones_.exposureUs(seq_.currentZone()));
        camera2GainDb_ = zones_.gainDb(seq_.currentZone());
        camera2_.setGain(camera2GainDb_);
    }

    // Restart camera_1 only once camera_2 is armed, so no trigger is lost.
    camera1_.startAcquisition(false);  // OneByOne — software triggered

    std::cout << "[bootstrap] done. starting capture loop.\n";
}

// ── main capture loop ──────────────────────────────────────────────────────

void BracketingDriver::captureStep() {
    const int    requestedZone = seq_.currentZone();
    const int    stepIndex     = seq_.currentStepIndex();

    double expUs, gainDb;
    {
        std::lock_guard<std::mutex> lock(tracksMutex_);
        expUs  = zones_.exposureUs(requestedZone);
        gainDb = zones_.gainDb(requestedZone);
    }

    camera1_.setExposure(expUs);
    camera1_.setGain(gainDb);
    // camera_2's exposure/gain were written at the end of the previous step:
    // writing them while it is armed briefly disturbs its trigger input, so
    // doing it right before the trigger would make it miss the pulse.

    // Host-side pacing to a fixed 1/acquisitionFrameRate cadence (a
    // triggered camera ignores AcquisitionFrameRate). If we fall behind,
    // resync to now + period instead of bursting catch-up triggers.
    if (cfg_.acquisitionFrameRate > 0.0) {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.acquisitionFrameRate));
        const auto now = std::chrono::steady_clock::now();
        if (nextTriggerTime_ < now) {
            nextTriggerTime_ = now + period;
        } else {
            std::this_thread::sleep_until(nextTriggerTime_);
            nextTriggerTime_ += period;
        }
    }

    // camera_2 must have finished its previous frame before it can latch
    // the next hardware trigger.
    if (!mono_)
        camera2_.waitForTriggerReady(cfg_.grabTimeoutMs);

    // One software trigger on camera_1; its ExposureActive output
    // hardware-triggers camera_2.
    camera1_.executeSoftwareTrigger();

    uint8_t* cam1Pixels = camera1PixelBuf_[bufIdx_].data();
    uint8_t* cam2Pixels = camera2PixelBuf_[bufIdx_].data();

    const size_t pixelBytes = static_cast<size_t>(cfg_.width) * static_cast<size_t>(cfg_.height);

    double actualExposureUsCam1 = 0.0;
    const bool cam1Grabbed = camera1_.grab(cam1Pixels, pixelBytes,
                                           actualExposureUsCam1, cfg_.grabTimeoutMs);
    if (!cam1Grabbed) {
        std::cerr << "[warn] camera 1 grab failed on zone=" << requestedZone
                  << " step=" << stepIndex << "\n";
        seq_.advance();
        if (mono_) return;
        // camera_2 was triggered anyway: drain its frame so the next step
        // does not pair camera_1's new frame with camera_2's stale one.
        double dummy = 0.0;
        camera2_.grab(cam2Pixels, pixelBytes, dummy, cfg_.camera2GrabTimeoutMs);
        {
            std::lock_guard<std::mutex> lock(tracksMutex_);
            camera2_.setExposure(zones_.exposureUs(seq_.currentZone()));
            camera2GainDb_ = zones_.gainDb(seq_.currentZone());
            camera2_.setGain(camera2GainDb_);
        }
        return;
    }

    double actualExposureUsCam2 = 0.0;
    const bool cam2Grabbed = !mono_ && camera2_.grab(cam2Pixels, pixelBytes,
                                                     actualExposureUsCam2, cfg_.camera2GrabTimeoutMs);
    if (!mono_ && !cam2Grabbed) {
        std::cerr << "[warn] camera 2 grab failed (check trigger cable) on zone=" << requestedZone
                  << " step=" << stepIndex << "\n";
    }

    // Host timestamp right after both grabs returned.
    const int64_t ts = nowNs();

    ProcessingJob job;
    job.bufIdx              = bufIdx_;
    job.requestedZone       = requestedZone;
    job.stepIndex           = stepIndex;
    job.actualExposureUsCam1 = actualExposureUsCam1;
    job.actualExposureUsCam2 = actualExposureUsCam2;
    job.gainDb              = gainDb;
    job.camera2GainDb       = camera2GainDb_;  // must read before the tail below overwrites it
    job.cam2Grabbed         = cam2Grabbed;
    job.timestampNs         = ts;
    submitProcessingJob(job);
    bufIdx_ = 1 - bufIdx_;

    seq_.advance();
    if (mono_) return;

    // Preset camera_2 for the NEXT step now, well before its trigger.
    {
        std::lock_guard<std::mutex> lock(tracksMutex_);
        camera2_.setExposure(zones_.exposureUs(seq_.currentZone()));
        camera2GainDb_ = zones_.gainDb(seq_.currentZone());
        camera2_.setGain(camera2GainDb_);
    }
}

void BracketingDriver::submitProcessingJob(const ProcessingJob& job) {
    std::unique_lock<std::mutex> lock(jobMutex_);
    // Back-pressure: at most one job in flight, so the double-buffered
    // pixel slot is never reused while the processing thread owns it.
    jobCv_.wait(lock, [this] { return !jobInFlight_; });
    pendingJob_  = job;
    jobInFlight_ = true;
    lock.unlock();
    jobCv_.notify_all();
}

void BracketingDriver::processingLoop() {
    for (;;) {
        ProcessingJob job;
        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobCv_.wait(lock, [this] { return jobInFlight_ || stopProcessing_; });
            if (!jobInFlight_)
                return;  // stopProcessing_ and no work left to drain
            job = pendingJob_;
        }
        processJob(job);
        {
            std::lock_guard<std::mutex> lock(jobMutex_);
            jobInFlight_ = false;
        }
        jobCv_.notify_all();
    }
}

void BracketingDriver::processJob(const ProcessingJob& job) {
    uint8_t* cam1Pixels = camera1PixelBuf_[job.bufIdx].data();
    uint8_t* cam2Pixels = camera2PixelBuf_[job.bufIdx].data();
    const size_t pixelBytes = static_cast<size_t>(cfg_.width) * static_cast<size_t>(cfg_.height);

    // The frame is attributed to the zone whose CURRENT exposure estimate is
    // closest to its chunk-data exposure; the requested zone breaks ties.
    int zone;
    uint8_t convergedFlag;
    bool    unreachableFlag;
    std::vector<ZoneMeasurement> allZones;
    {
        std::lock_guard<std::mutex> lock(tracksMutex_);
        zone = zones_.closestZone(job.actualExposureUsCam1, job.requestedZone);
        // Each end zone measures its OWN tail of the whole frame: low
        // (zone 0) its bright tail, high (zone 2) its dark tail. Mid's
        // median is informational only (mid is derived, not measured).
        allZones.reserve(static_cast<size_t>(cfg_.aeZoneCount));
        allZones.push_back(measurePercentileBrightness(
                cam1Pixels, cfg_.width, cfg_.height, cfg_.zoneSortStride, cfg_.aeBridgeLowPercentile));  // zone 0 = low
        allZones.push_back(measurePercentileBrightness(
                cam1Pixels, cfg_.width, cfg_.height, cfg_.zoneSortStride, 50.0));                        // zone 1 = mid (informational only)
        allZones.push_back(measurePercentileBrightness(
                cam1Pixels, cfg_.width, cfg_.height, cfg_.zoneSortStride, cfg_.aeBridgeHighPercentile)); // zone 2 = high

        const int lowZone  = 0;
        const int highZone = 2;  // midZone_ == 1, enforced by validate()

        // Plain single-objective update for low/high; mid has no track
        // of its own to update, it is derived below.
        if (zone == lowZone || zone == highZone) {
            const ZoneMeasurement& zm = allZones[static_cast<size_t>(zone)];
            zones_.update(zone, zm.mean, cfg_.minExposureUs, cfg_.maxExposureUs,
                          cfg_.minGainDb, cfg_.maxGainDb);
        }

        // Flags reflect each track's own state from BEFORE the min-gap
        // floor below.
        if (zone == lowZone || zone == highZone) {
            convergedFlag   = zones_.converged(zone) ? 1u : 0u;
            unreachableFlag = zones_.unreachable(zone);
        } else {
            convergedFlag   = (zones_.converged(lowZone) && zones_.converged(highZone)) ? 1u : 0u;
        }

        // Min-stops-gap floor (Delta_min): if low/high are closer than
        // aeBridgeMinStopsGap stops of total light, push both apart
        // symmetrically around their midpoint.
        const double totalLow  = totalLight(zones_.exposureUs(lowZone), zones_.gainDb(lowZone));
        const double totalHigh = totalLight(zones_.exposureUs(highZone), zones_.gainDb(highZone));
        const double stopsLow  = std::log2(totalLow);
        const double stopsHigh = std::log2(totalHigh);
        const double naturalGap = stopsHigh - stopsLow;
        if (naturalGap < cfg_.aeBridgeMinStopsGap) {
            const double midStops = (stopsLow + stopsHigh) / 2.0;
            const double newStopsLow  = midStops - cfg_.aeBridgeMinStopsGap / 2.0;
            const double newStopsHigh = midStops + cfg_.aeBridgeMinStopsGap / 2.0;
            const DerivedBridge lowSplit = splitTotalMinGain(
                    std::exp2(newStopsLow), cfg_.minExposureUs, cfg_.maxExposureUs, cfg_.minGainDb, cfg_.maxGainDb);
            const DerivedBridge highSplit = splitTotalMinGain(
                    std::exp2(newStopsHigh), cfg_.minExposureUs, cfg_.maxExposureUs, cfg_.minGainDb, cfg_.maxGainDb);
            zones_.setDerived(lowZone, lowSplit.exposureUs, lowSplit.gainDb);
            zones_.setDerived(highZone, highSplit.exposureUs, highSplit.gainDb);
            if (cfg_.verbose)
                std::cout << "[bridge] low/high gap " << naturalGap << " stops below floor "
                          << cfg_.aeBridgeMinStopsGap << " -- pushed apart symmetrically\n";
        }

        // Mid is re-derived every frame from low's and high's state.
        const DerivedBridge bridge = deriveBridgeMid(
                zones_.exposureUs(lowZone), zones_.gainDb(lowZone),
                zones_.exposureUs(highZone), zones_.gainDb(highZone),
                cfg_.minExposureUs, cfg_.maxExposureUs, cfg_.minGainDb, cfg_.maxGainDb);
        zones_.setDerived(midZone_, bridge.exposureUs, bridge.gainDb);

        if (zone != lowZone && zone != highZone) {
            unreachableFlag = bridge.unreachable;
        }
    }
    const double brightnessCam1 = allZones[static_cast<size_t>(zone)].mean;

    // Stereo publishes a pair or nothing; mono always publishes camera_1.
    const bool deliver = mono_ || job.cam2Grabbed;

    if (deliver && camera1Callback_) {
        CapturedFrame f;
        f.pixels           = cam1Pixels;
        f.size             = pixelBytes;
        f.timestamp_ns     = job.timestampNs;
        f.exposure_used_us = job.actualExposureUsCam1;
        f.gain_db          = job.gainDb;
        f.brightness       = brightnessCam1;
        f.ae_zone          = zone;
        f.sequence_index   = static_cast<uint8_t>(job.stepIndex);
        f.converged        = (convergedFlag == 1u);
        f.track_unreachable = unreachableFlag;
        camera1Callback_(f);
    }

    if (job.cam2Grabbed && camera2Callback_) {  // never in mono
        CapturedFrame f;
        f.pixels           = cam2Pixels;
        f.size             = pixelBytes;
        f.timestamp_ns     = job.timestampNs;
        f.exposure_used_us = job.actualExposureUsCam2;
        f.gain_db          = job.camera2GainDb;  // actually applied to camera_2 -- see docs
        f.brightness       = brightnessCam1;  // camera_2 mirrors camera_1's AE
        f.ae_zone          = zone;
        f.sequence_index   = static_cast<uint8_t>(job.stepIndex);
        f.converged        = (convergedFlag == 1u);
        f.track_unreachable = unreachableFlag;
        camera2Callback_(f);
    }

    if (!cfg_.verbose) return;
    std::cout << "[zone " << zone << "/" << cfg_.aeZoneCount << "] "
              << "exp=" << job.actualExposureUsCam1 << "µs  "
              << "gain=" << job.gainDb << "dB  "
              << "bright=" << brightnessCam1 << "  "
              << (convergedFlag ? "CONV" : "    ")
              << (unreachableFlag ? "  UNREACHABLE" : "")
              << (deliver ? "" : "  [camera 2 miss]") << "\n"
              << "  zones:";
    for (size_t z = 0; z < allZones.size(); ++z) {
        const bool active = (static_cast<int>(z) == zone);
        std::cout << (active ? "  *" : "   ") << "z" << z
                   << "=[" << static_cast<int>(allZones[z].minVal)
                   << "," << static_cast<int>(allZones[z].maxVal) << "]"
                   << " n=" << allZones[z].sampleCount
                   << (active ? "*" : "");
    }
    std::cout << "\n";
}

void BracketingDriver::run(const std::atomic<bool>& running) {
    bootstrap();
    while (running.load(std::memory_order_relaxed))
        captureStep();
}
