#include "Config.h"

#include <stdexcept>

void validate(const DriverConfig& cfg) {
    if (cfg.aeZoneCount != 3)
        throw std::runtime_error("config: ae_zone_count must be exactly 3 "
                                  "(zone 0 = low, zone 1 = derived mid, zone 2 = high)");
    if (cfg.zoneSortStride < 1)
        throw std::runtime_error("config: zone_sort_stride must be >= 1");
    if (cfg.aeBridgeLowPercentile <= 0.0 || cfg.aeBridgeLowPercentile >= 100.0)
        throw std::runtime_error("config: ae_bridge_low_percentile must be in (0, 100)");
    if (cfg.aeBridgeHighPercentile <= 0.0 || cfg.aeBridgeHighPercentile >= 100.0)
        throw std::runtime_error("config: ae_bridge_high_percentile must be in (0, 100)");
    if (cfg.aeBridgeLowTarget < 0.0 || cfg.aeBridgeLowTarget > cfg.pixelMax)
        throw std::runtime_error("config: ae_bridge_low_target must be in [0, pixel_max]");
    if (cfg.aeBridgeHighTarget < 0.0 || cfg.aeBridgeHighTarget > cfg.pixelMax)
        throw std::runtime_error("config: ae_bridge_high_target must be in [0, pixel_max]");
    if (cfg.aeBridgeMinStopsGap < 0.0)
        throw std::runtime_error("config: ae_bridge_min_stops_gap must be >= 0");
    if (cfg.aeSaturationBoostFraction < 0.0)
        throw std::runtime_error("config: ae_saturation_boost_fraction must be >= 0");
    if (cfg.aeZoneAlpha.size() != static_cast<size_t>(cfg.aeZoneCount))
        throw std::runtime_error("config: ae_zone_alpha must have exactly ae_zone_count entries");
    for (const double a : cfg.aeZoneAlpha) {
        if (a <= 0.0 || a > 1.0)
            throw std::runtime_error("config: every ae_zone_alpha entry must be in (0.0, 1.0]");
    }

    if (cfg.camera1SerialNumber.empty() && !cfg.camera2SerialNumber.empty())
        throw std::runtime_error("config: camera_2_serial is set but camera_1_serial is not; "
                                  "set both (stereo), only camera_1_serial (mono), "
                                  "or neither (auto-detect)");

    constexpr int kSensorWidth = 1920, kSensorHeight = 1200;
    if (cfg.width <= 0 || cfg.height <= 0 ||
        cfg.width % 2 != 0 || cfg.height % 2 != 0 ||
        cfg.width > kSensorWidth || cfg.height > kSensorHeight)
        throw std::runtime_error("config: width/height must be positive, even, and not exceed "
                                  "the native sensor resolution 1920x1200 (cropped, centered, "
                                  "if smaller)");
}
