#pragma once
#include <array>

/**
 * @file ExposureTrack.h
 *
 * Per-zone software auto-exposure/gain controller: a fixed-rate EMA with
 * exposure/gain priority (more light: exposure first, gain only for what the
 * exposure limit cannot supply; less light: gain released first with exposure
 * frozen). See DESIGN.md.
 *
 * Every frame:
 *   measured' = pixelMax * (1 + saturationBoostFraction)   if B >= pixelMax
 *             = max(B, kMinBrightness)                      otherwise
 *   ratio     = B_target / measured'
 *
 * ratio >= 1 (need more light), exposure first:
 *   E_raw  = E * ratio,  E_prop = clamp(E_raw, minExp, maxExp)
 *   G_prop = (E_raw != E_prop) ? clamp(G + 20 log10(E_raw / E_prop), minGain, maxGain)
 *                              : minGain
 *   E      = (1 - alpha) * E + alpha * E_prop
 *   G      = (1 - alpha) * G + alpha * G_prop
 *
 * ratio < 1 (need less light), gain released first:
 *   if G > minGain + kGainFloorToleranceDb:   (exposure frozen)
 *     G = (1 - alpha) * G + alpha * clamp(G + 20 log10(ratio), minGain, maxGain)
 *   else:
 *     G = minGain
 *     E = (1 - alpha) * E + alpha * clamp(E * ratio, minExp, maxExp)
 *
 * Moving exposure and gain in the same frame would double-apply the
 * correction; releasing gain first keeps gain (noise) minimal.
 *
 * The saturation boost compensates for the fact that a clipped reading
 * carries no magnitude information: without it, a target close to pixelMax
 * (e.g. B-Adapt's low bracket, 250/255) would give ratio ~ 0.98
 * on a fully overexposed frame and barely move.
 *
 * Converged when the last three readings are all within epsilon (relative)
 * of the target.
 */
class ExposureTrack {
public:
    /**
     * @param targetPercent           Target brightness, % of pixelMax.
     * @param pixelMax                Max pixel value (255 for BayerRG8).
     * @param alpha                   EMA rate in (0, 1].
     * @param epsilon                 Convergence threshold (fractional).
     * @param defaultExposureUs       Exposure until seed() is called.
     * @param initialGainDb           Starting gain (= minGainDb).
     * @param saturationBoostFraction See file docs. 0 disables.
     */
    ExposureTrack(int targetPercent, double pixelMax, double alpha, double epsilon,
                  double defaultExposureUs, double initialGainDb = 0.0,
                  double saturationBoostFraction = 0.0);

    /// Set the exposure estimate directly (AE-Once bootstrap seed).
    void seed(double exposureUs) { exposureUs_ = exposureUs; }

    /// Overwrite exposure and gain directly, bypassing the control law (no
    /// EMA, no clamping, no history update). Used for the derived mid zone
    /// and the min-gap floor. The caller clamps.
    void setExposureGain(double exposureUs, double gainDb) {
        exposureUs_ = exposureUs;
        gainDb_ = gainDb;
    }

    /**
     * Apply one control step from a newly measured brightness (0..pixelMax)
     * and update the convergence state. See the file docs for the law.
     */
    void update(double brightness, double minExp, double maxExp,
                double minGain, double maxGain);

    double exposureUs() const { return exposureUs_; }  ///< Exposure to program next (µs).
    double gainDb() const { return gainDb_; }          ///< Gain to program next (dB).
    bool converged() const { return converged_; }      ///< Last 3 readings within epsilon.

private:
    double targetBrightness_;
    double exposureUs_;
    double gainDb_;
    double alpha_;
    double epsilon_;
    double pixelMax_;
    double saturationBoostFraction_;
    bool   converged_ = false;

    std::array<double, 3> history_{};
    int historyIndex_ = 0;
    int historyCount_ = 0;
};
