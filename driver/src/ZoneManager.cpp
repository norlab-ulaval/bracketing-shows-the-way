#include "ZoneManager.h"
#include <algorithm>
#include <stdexcept>

namespace {
/// Gain is an EMA asymptotically approaching its bound and essentially never
/// satisfies an exact floating-point equality check -- matches the
/// tolerance ExposureTrack::update() already uses for the same reason
/// (kGainFloorToleranceDb in ExposureTrack.cpp) for its own, different
/// purpose (detecting "at the floor" to resume exposure movement); here
/// it's reused for detecting "pinned" at EITHER bound, for unreachable().
constexpr double kGainPinToleranceDb = 0.05;
}  // namespace

ZoneManager::ZoneManager(int zoneCount, const std::vector<int>& zoneTargetPercents,
                         double pixelMax, const std::vector<double>& zoneAlphas, double epsilon,
                         double defaultExposureUs, double exposureTieToleranceUs,
                         double minExposureUs, double maxExposureUs,
                         double minGainDb, double maxGainDb,
                         double saturationBoostFraction)
    : exposureTieToleranceUs_(exposureTieToleranceUs),
      minExp_(minExposureUs), maxExp_(maxExposureUs),
      minGainDb_(minGainDb), maxGainDb_(maxGainDb) {
    if (zoneCount < 1)
        throw std::invalid_argument("ZoneManager: zoneCount must be >= 1");
    if (zoneTargetPercents.size() != static_cast<size_t>(zoneCount))
        throw std::invalid_argument("ZoneManager: zoneTargetPercents must have exactly zoneCount entries");
    if (zoneAlphas.size() != static_cast<size_t>(zoneCount))
        throw std::invalid_argument("ZoneManager: zoneAlphas must have exactly zoneCount entries");
    tracks_.reserve(static_cast<size_t>(zoneCount));
    for (int i = 0; i < zoneCount; ++i)
        tracks_.emplace_back(zoneTargetPercents[static_cast<size_t>(i)], pixelMax,
                              zoneAlphas[static_cast<size_t>(i)], epsilon,
                              defaultExposureUs, minGainDb, saturationBoostFraction);
}

void ZoneManager::seed(double exposureUs, double minExp, double maxExp) {
    minExp_ = minExp;
    maxExp_ = maxExp;
    const double clamped = std::clamp(exposureUs, minExp, maxExp);
    for (auto& track : tracks_)
        track.seed(clamped);
}

double ZoneManager::exposureUs(int zone) const { return tracks_.at(zone).exposureUs(); }
double ZoneManager::gainDb(int zone) const { return tracks_.at(zone).gainDb(); }
bool   ZoneManager::converged(int zone) const { return tracks_.at(zone).converged(); }

bool ZoneManager::unreachable(int zone) const {
    const ExposureTrack& track = tracks_.at(zone);
    if (track.converged()) return false;
    // Both exposureUs_ and gainDb_ are EMAs that asymptotically approach
    // their clamp and essentially never reach it at exact floating-point
    // equality (each step only closes (1-alpha) of the remaining gap) --
    // an exact <=/>= check here would almost never fire for a genuinely
    // pinned zone, silently hiding the exact case this function exists to
    // report. Exposure's tolerance is relative to its own configured range
    // (a fixed µs value would be meaningless across very different
    // min/max_exposure_us configs); gain reuses the same absolute
    // tolerance ExposureTrack::update() already uses for the analogous
    // "at the floor" check.
    const double expTolUs = std::max(1.0, 0.001 * (maxExp_ - minExp_));
    const bool expPinned  = (track.exposureUs() <= minExp_ + expTolUs ||
                              track.exposureUs() >= maxExp_ - expTolUs);
    const bool gainPinned = (track.gainDb() <= minGainDb_ + kGainPinToleranceDb ||
                              track.gainDb() >= maxGainDb_ - kGainPinToleranceDb);
    return expPinned && gainPinned;
}

void ZoneManager::update(int zone, double brightness, double minExp, double maxExp,
                          double minGain, double maxGain) {
    minExp_    = minExp;
    maxExp_    = maxExp;
    minGainDb_ = minGain;
    maxGainDb_ = maxGain;
    tracks_.at(zone).update(brightness, minExp, maxExp, minGain, maxGain);
}

void ZoneManager::setDerived(int zone, double exposureUs, double gainDb) {
    tracks_.at(zone).setExposureGain(exposureUs, gainDb);
}

int ZoneManager::closestZone(double actualExposureUs, int expectedZone) const {
    const int n = static_cast<int>(tracks_.size());
    std::vector<double> diffs(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        diffs[static_cast<size_t>(i)] = std::abs(tracks_[static_cast<size_t>(i)].exposureUs() - actualExposureUs);

    const double bestDiff = *std::min_element(diffs.begin(), diffs.end());

    int fallback = 0;
    for (int i = 0; i < n; ++i) {
        if (diffs[static_cast<size_t>(i)] - bestDiff <= exposureTieToleranceUs_) {
            fallback = i;
            if (i == expectedZone) return expectedZone;
        }
    }
    return fallback;
}
