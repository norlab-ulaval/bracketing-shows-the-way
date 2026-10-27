#include "ExposureTrack.h"
#include <algorithm>
#include <cmath>

namespace {
/// Floor applied to measured brightness before computing ratio,
/// so a fully black frame (brightness == 0) still produces a large-but-finite
/// correction instead of an infinite one or a skipped update.
constexpr double kMinBrightness = 0.5;

/// Gain is an EMA asymptotically approaching minGain and would otherwise
/// never satisfy an exact floating-point equality check -- within this many
/// dB of minGain, gain is treated as "at the floor" for the purpose of
/// releasing exposure to move (see the gain-release-first control law in
/// update()).
constexpr double kGainFloorToleranceDb = 0.05;
} // namespace

ExposureTrack::ExposureTrack(int targetPercent, double pixelMax, double alpha, double epsilon,
                             double defaultExposureUs, double initialGainDb,
                             double saturationBoostFraction)
    : targetBrightness_(targetPercent / 100.0 * pixelMax)
    , exposureUs_(defaultExposureUs) // overwritten by seed()
    , gainDb_(initialGainDb)
    , alpha_(alpha)
    , epsilon_(epsilon)
    , pixelMax_(pixelMax)
    , saturationBoostFraction_(saturationBoostFraction)
{}

void ExposureTrack::update(double brightness, double minExp, double maxExp,
                            double minGain, double maxGain) {
    // Saturation boost on clipped readings, floor on near-black ones.
    const double measuredBrightness = (brightness >= pixelMax_)
        ? pixelMax_ * (1.0 + saturationBoostFraction_)
        : std::max(brightness, kMinBrightness);
    const double ratio = targetBrightness_ / measuredBrightness;

    if (ratio >= 1.0) {
        // Need MORE light: exposure first; gain only covers what the
        // exposure clamp could not supply this same frame.
        const double oldExp       = exposureUs_;
        const double unclampedExp = oldExp * ratio;
        const double clampedExp   = std::clamp(unclampedExp, minExp, maxExp);
        exposureUs_ = oldExp * (1.0 - alpha_) + clampedExp * alpha_;

        const bool   saturated     = (unclampedExp != clampedExp);
        const double leftover      = unclampedExp / clampedExp;
        const double targetGainDb  = saturated
            ? std::clamp(gainDb_ + 20.0 * std::log10(leftover), minGain, maxGain)
            : minGain;
        gainDb_ = gainDb_ * (1.0 - alpha_) + targetGainDb * alpha_;
    } else {
        // Need LESS light: release gain to the floor first, with exposure
        // frozen.
        if (gainDb_ > minGain + kGainFloorToleranceDb) {
            const double targetGainDb = std::clamp(
                gainDb_ + 20.0 * std::log10(ratio), minGain, maxGain);
            gainDb_ = gainDb_ * (1.0 - alpha_) + targetGainDb * alpha_;
            // exposureUs_ intentionally untouched this frame.
        } else {
            gainDb_ = minGain;  // within tolerance of the floor -- snap the rest of the way
            const double oldExp       = exposureUs_;
            const double unclampedExp = oldExp * ratio;
            const double clampedExp   = std::clamp(unclampedExp, minExp, maxExp);
            exposureUs_ = oldExp * (1.0 - alpha_) + clampedExp * alpha_;
        }
    }

    // Rolling 3-entry history (oldest entry overwritten). historyIndex_ wraps
    // every 3 calls and historyCount_ saturates at 3, so neither can overflow
    // no matter how long the track runs.
    history_[historyIndex_] = brightness;
    historyIndex_ = (historyIndex_ + 1) % 3;
    if (historyCount_ < 3) ++historyCount_;

    if (historyCount_ < 3) {
        converged_ = false;
        return;
    }

    converged_ = true;
    for (double b : history_) {
        if (std::abs(b / targetBrightness_ - 1.0) >= epsilon_) {
            converged_ = false;
            break;
        }
    }
}
