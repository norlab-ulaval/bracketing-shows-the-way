#include "CameraGate.h"

#include <pylon/PylonIncludes.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace Pylon;
using namespace GenApi;

// Native active sensor resolution (a2A1920-51gcPRO). Matches validate().
namespace { constexpr int kSensorWidth = 1920, kSensorHeight = 1200; }

// ── PIMPL ─────────────────────────────────────────────────────────────────

struct CameraGate::Impl {
    CInstantCamera camera;
    INodeMap*      nodemap        = nullptr;
    int            numBuffers     = 10;
    int            frameRetentMs  = -1;
    double         lastExposureUs = -1.0;
    double         lastGainDb     = -1.0;
};

// ── enumeration ────────────────────────────────────────────────────────────

std::vector<CameraGate::DeviceInfo> CameraGate::enumerateDevices() {
    DeviceInfoList_t list;
    CTlFactory::GetInstance().EnumerateDevices(list);
    std::vector<DeviceInfo> out;
    for (const CDeviceInfo& d : list)
        out.push_back({d.GetSerialNumber().c_str(), d.GetModelName().c_str(),
                       d.IsIpAddressAvailable() ? d.GetIpAddress().c_str() : ""});
    // Shorter first, then lexicographic: numeric order for digit-only serials.
    std::sort(out.begin(), out.end(), [](const DeviceInfo& a, const DeviceInfo& b) {
        return a.serial.size() != b.serial.size() ? a.serial.size() < b.serial.size()
                                                  : a.serial < b.serial;
    });
    return out;
}

// ── lifecycle ──────────────────────────────────────────────────────────────

CameraGate::CameraGate() : impl_(std::make_unique<Impl>()) {}
CameraGate::~CameraGate() { disconnect(); }

void CameraGate::connect(const std::string& serialNumber,
                         int heartbeatTimeoutMs, int packetSizeBytes,
                         double gainDb,
                         int interPacketDelayTicks,
                         int numGrabBuffers, int frameRetentionMs,
                         int outputWidth, int outputHeight,
                         const std::string& bandwidthReserveMode) {
    CTlFactory& factory = CTlFactory::GetInstance();

    if (serialNumber.empty()) {
        impl_->camera.Attach(factory.CreateFirstDevice());
    } else {
        CDeviceInfo info;
        info.SetSerialNumber(serialNumber.c_str());
        impl_->camera.Attach(factory.CreateDevice(info));
    }

    impl_->camera.Open();
    impl_->nodemap = &impl_->camera.GetNodeMap();
    INodeMap& nm   = *impl_->nodemap;

    // Wipe any parameters left by Pylon Viewer or a previous session before
    // applying the driver's own configuration.
    CEnumerationPtr(nm.GetNode("UserSetSelector"))->FromString("Default");
    CCommandPtr(nm.GetNode("UserSetLoad"))->Execute();

    // Force the rated active resolution 1920x1200 (WidthMax/HeightMax report
    // 1936x1216 incl. border pixels) before any crop below. Offsets first so
    // widening never exceeds sensor bounds.
    auto setInt = [&nm](const char* name, int64_t value) {
        CIntegerPtr node(nm.GetNode(name));
        if (node.IsValid() && IsWritable(node))
            node->SetValue(std::clamp(value, node->GetMin(), node->GetMax()));
    };

    setInt("OffsetX", 0);
    setInt("OffsetY", 0);
    setInt("Width",  kSensorWidth);
    setInt("Height", kSensorHeight);

    CEnumerationPtr(nm.GetNode("ExposureAuto"))->FromString("Off");
    CEnumerationPtr(nm.GetNode("GainAuto"))->FromString("Off");
    CEnumerationPtr(nm.GetNode("AcquisitionMode"))->FromString("Continuous");

    // Linear raw BayerRG8: disable gamma and the sRGB color-space transform
    // (which bakes in ~0.4 gamma). Skipped silently if a model lacks the node.
    CFloatPtr gamma(nm.GetNode("Gamma"));
    if (gamma.IsValid() && IsWritable(gamma))
        gamma->SetValue(1.0);

    CEnumerationPtr colorSpace(nm.GetNode("BslColorSpace"));
    if (colorSpace.IsValid() && IsWritable(colorSpace))
        colorSpace->FromString("Off");

    // Other processing nodes (BslContrast, BslSaturation, BslHue,
    // BslColorAdjustmentEnable, BslLightSourcePreset, BlackLevel,
    // DigitalShift) default to neutral on this model and are left untouched.
    // PixelFormat stays at the UserSet default, BayerRG8.

    // Set initial gain (= minGainDb). The AEG controller adjusts it per-frame
    // via setGain() — this just establishes the starting state.
    CFloatPtr gain(nm.GetNode("Gain"));
    if (gain.IsValid() && IsWritable(gain))
        gain->SetValue(std::clamp(gainDb, gain->GetMin(), gain->GetMax()));

    CIntegerPtr heartbeat(nm.GetNode("GevHeartbeatTimeout"));
    if (heartbeat.IsValid() && IsWritable(heartbeat))
        heartbeat->SetValue(heartbeatTimeoutMs);

    // packetSizeBytes=1440 (the default) fits inside a standard 1500-byte MTU
    // after IP/UDP/GVSP headers, so frames aren't fragmented on adapters
    // without jumbo frames enabled.
    CIntegerPtr packetSize(nm.GetNode("GevSCPSPacketSize"));
    if (packetSize.IsValid() && IsWritable(packetSize))
        packetSize->SetValue(packetSizeBytes);

    // Inter-packet delay paces the camera's outgoing UDP bursts; increase to
    // 1000-4000 ticks on congested links. Written unconditionally: the
    // camera's factory default is 512 ticks (~4.1 µs of dead time after
    // every packet), and the UserSetLoad(Default) above restores it — so a
    // requested 0 must be written explicitly or the camera silently keeps
    // pacing itself ~25-30% below wire rate.
    CIntegerPtr gevSCPD(nm.GetNode("GevSCPD"));
    if (gevSCPD.IsValid() && IsWritable(gevSCPD))
        gevSCPD->SetValue(interPacketDelayTicks);

    // BandwidthReserveMode: "Standard" (camera default) holds back ~10% of
    // the GigE link for packet retransmissions, capping full-res BayerRG8 at
    // ~48 fps. "Performance" shrinks the reserve, raising the camera's own
    // ResultingFrameRate prediction to ~52 fps at 8192-byte packets — the
    // rated 51 fps is unreachable without it. Empty = leave camera default.
    if (!bandwidthReserveMode.empty()) {
        CEnumerationPtr bwReserve(nm.GetNode("BandwidthReserveMode"));
        if (bwReserve.IsValid() && IsWritable(bwReserve))
            bwReserve->FromString(bandwidthReserveMode.c_str());
    }

    // Centered AOI crop (no-op at native size). Shrink first, then move the
    // offsets, so the AOI never exceeds sensor bounds. PixelFormat is
    // untouched, so output stays BayerRG8 at any size.
    if (outputWidth != kSensorWidth || outputHeight != kSensorHeight) {
        setInt("Width",  outputWidth);
        setInt("Height", outputHeight);
        setInt("OffsetX", (kSensorWidth  - outputWidth)  / 2);
        setInt("OffsetY", (kSensorHeight - outputHeight) / 2);
    }

    // Cache buffer/retention settings so startAcquisition() can apply them.
    impl_->numBuffers     = numGrabBuffers;
    impl_->frameRetentMs  = frameRetentionMs;
    impl_->lastExposureUs = -1.0;
    impl_->lastGainDb     = -1.0;

    // setExposure() takes effect on the next triggered frame, so a frame
    // grabbed right after it may still reflect a previous exposure setting.
    // Chunk data lets grab() report the exposure actually used per frame.
    CBooleanPtr chunkModeActive(nm.GetNode("ChunkModeActive"));
    if (chunkModeActive.IsValid() && IsWritable(chunkModeActive)) {
        chunkModeActive->SetValue(true);
        CEnumerationPtr chunkSelector(nm.GetNode("ChunkSelector"));
        if (chunkSelector.IsValid()) {
            chunkSelector->FromString("ExposureTime");
            CBooleanPtr chunkEnable(nm.GetNode("ChunkEnable"));
            if (chunkEnable.IsValid() && IsWritable(chunkEnable))
                chunkEnable->SetValue(true);
        }
    }
}

void CameraGate::disconnect() {
    if (!impl_) return;
    if (impl_->camera.IsOpen()) {
        if (impl_->camera.IsGrabbing()) impl_->camera.StopGrabbing();
        impl_->camera.Close();
    }
    impl_->nodemap = nullptr;
}

void CameraGate::startAcquisition(bool latestOnly) {
    impl_->camera.MaxNumBuffer = impl_->numBuffers;
    impl_->camera.StartGrabbing(latestOnly ? GrabStrategy_LatestImageOnly
                                           : GrabStrategy_OneByOne);

    // FrameRetention is a stream-grabber parameter; the stream grabber is only
    // open after StartGrabbing(), so it must be set here rather than in connect().
    if (impl_->frameRetentMs > 0) {
        try {
            INodeMap& sgNodeMap = impl_->camera.GetStreamGrabberNodeMap();
            CIntegerPtr fr(sgNodeMap.GetNode("FrameRetention"));
            if (fr.IsValid() && IsWritable(fr))
                fr->SetValue(static_cast<int64_t>(impl_->frameRetentMs) * 1000);
        } catch (const GenericException& e) {
            std::cerr << "[warn] could not set FrameRetention: " << e.what() << "\n";
        }
    }
}

void CameraGate::stopAcquisition() {
    if (impl_->camera.IsGrabbing()) impl_->camera.StopGrabbing();
}

// ── stereo trigger I/O ──────────────────────────────────────────────────────

void CameraGate::configureTriggerOutput(const std::string& lineSelector,
                                        const std::string& lineSource) {
    INodeMap& nm = *impl_->nodemap;

    CEnumerationPtr lineSelectorNode(nm.GetNode("LineSelector"));
    lineSelectorNode->FromString(lineSelector.c_str());

    CEnumerationPtr lineMode(nm.GetNode("LineMode"));
    if (lineMode.IsValid() && IsWritable(lineMode))
        lineMode->FromString("Output");

    CEnumerationPtr lineSourceNode(nm.GetNode("LineSource"));
    if (lineSourceNode.IsValid() && IsWritable(lineSourceNode))
        lineSourceNode->FromString(lineSource.c_str());
}

void CameraGate::configureHardwareTrigger(const std::string& lineSelector,
                                          const std::string& triggerActivation,
                                          const std::string& triggerSource) {
    INodeMap& nm = *impl_->nodemap;

    CEnumerationPtr(nm.GetNode("TriggerSelector"))->FromString("FrameStart");

    CEnumerationPtr lineSelectorNode(nm.GetNode("LineSelector"));
    lineSelectorNode->FromString(lineSelector.c_str());

    CEnumerationPtr lineMode(nm.GetNode("LineMode"));
    if (lineMode.IsValid() && IsWritable(lineMode))
        lineMode->FromString("Input");

    CEnumerationPtr(nm.GetNode("TriggerSource"))->FromString(triggerSource.c_str());
    CEnumerationPtr(nm.GetNode("TriggerActivation"))->FromString(triggerActivation.c_str());

    // Set last so the camera isn't armed (waiting on a trigger pulse) while
    // TriggerSource/TriggerActivation are still being configured above.
    CEnumerationPtr(nm.GetNode("TriggerMode"))->FromString("On");

    // Accept the next trigger once sensor readout starts rather than after
    // the GigE transfer completes; otherwise retransmits can make camera_2
    // reject every other trigger. Skipped if the node is unavailable.
    CEnumerationPtr triggerOverlap(nm.GetNode("TriggerOverlap"));
    if (triggerOverlap.IsValid() && IsWritable(triggerOverlap))
        triggerOverlap->FromString("ReadOut");
}

void CameraGate::setWhiteBalance(double r, double g, double b) {
    INodeMap& nm = *impl_->nodemap;

    CEnumerationPtr autoWb(nm.GetNode("BalanceWhiteAuto"));
    if (autoWb.IsValid() && IsWritable(autoWb))
        autoWb->FromString("Off");

    auto setChannel = [&nm](const char* channel, double ratio) {
        CEnumerationPtr sel(nm.GetNode("BalanceRatioSelector"));
        if (!sel.IsValid() || !IsWritable(sel)) return;
        sel->FromString(channel);
        CFloatPtr ratioNode(nm.GetNode("BalanceRatio"));
        if (ratioNode.IsValid() && IsWritable(ratioNode))
            ratioNode->SetValue(std::clamp(ratio, ratioNode->GetMin(), ratioNode->GetMax()));
    };

    setChannel("Red",   r);
    setChannel("Green", g);
    setChannel("Blue",  b);
}

void CameraGate::configureSoftwareTrigger() {
    INodeMap& nm = *impl_->nodemap;
    CEnumerationPtr(nm.GetNode("TriggerSelector"))->FromString("FrameStart");
    CEnumerationPtr(nm.GetNode("TriggerSource"))->FromString("Software");
    // Set last so the camera isn't armed before source is configured.
    CEnumerationPtr(nm.GetNode("TriggerMode"))->FromString("On");

}

void CameraGate::waitForTriggerReady(unsigned int timeoutMs) {
    impl_->camera.WaitForFrameTriggerReady(timeoutMs, TimeoutHandling_ThrowException);
}

void CameraGate::executeSoftwareTrigger() {
    // Must wait for the camera to be ready before firing the next trigger;
    // without this, ExecuteSoftwareTrigger() may succeed (no exception) but
    // the camera silently ignores the trigger and the grab never delivers a frame.
    waitForTriggerReady(5000);
    impl_->camera.ExecuteSoftwareTrigger();
}

// ── exposure control ───────────────────────────────────────────────────────

void CameraGate::setExposure(double us) {
    if (std::abs(us - impl_->lastExposureUs) < 0.5)
        return;
    CFloatPtr(impl_->nodemap->GetNode("ExposureTime"))->SetValue(us);
    impl_->lastExposureUs = us;
}

void CameraGate::setGain(double db) {
    if (std::abs(db - impl_->lastGainDb) < 0.001)
        return;
    CFloatPtr gain(impl_->nodemap->GetNode("Gain"));
    if (gain.IsValid() && IsWritable(gain)) {
        db = std::clamp(db, gain->GetMin(), gain->GetMax());
        gain->SetValue(db);
        impl_->lastGainDb = db;
    }
}

double CameraGate::runAEOnce(int maxIters, unsigned int timeoutMs, int pollIntervalMs) {
    INodeMap& nm = *impl_->nodemap;
    CEnumerationPtr aeNode(nm.GetNode("ExposureAuto"));
    aeNode->FromString("Once");

    // Grab until camera sets ExposureAuto back to "Off" (Basler auto-resets).
    CGrabResultPtr result;
    for (int i = 0; i < maxIters; ++i) {
        impl_->camera.RetrieveResult(timeoutMs, result, TimeoutHandling_ThrowException);
        result = CGrabResultPtr(); // release back to pool

        if (std::string(aeNode->ToString()) == "Off")
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }

    return CFloatPtr(nm.GetNode("ExposureTime"))->GetValue();
}

// ── frame grab ────────────────────────────────────────────────────────────

bool CameraGate::grab(uint8_t* dst, size_t dstSize, double& actualExposureUs, unsigned int timeoutMs) {
    CGrabResultPtr result;
    impl_->camera.RetrieveResult(timeoutMs, result, TimeoutHandling_Return);

    if (!result.IsValid()) {
        std::cerr << "[grab] no result within " << timeoutMs << " ms (timeout)\n";
        return false;
    }
    if (!result->GrabSucceeded()) {
        std::cerr << "[grab] failed: 0x" << std::hex << result->GetErrorCode()
                   << std::dec << " " << result->GetErrorDescription() << "\n";
        return false;
    }

    const size_t frameBytes = static_cast<size_t>(result->GetImageSize());
    if (frameBytes > dstSize)
        throw std::runtime_error("grab: destination buffer too small");

    std::memcpy(dst, result->GetBuffer(), frameBytes);

    CFloatPtr chunkExposure(result->GetChunkDataNodeMap().GetNode("ChunkExposureTime"));
    actualExposureUs = chunkExposure.IsValid() ? chunkExposure->GetValue() : -1.0;

    return true;
}
