#pragma once
#include "ExposureTrack.h"
#include <vector>

/**
 * @file ZoneManager.h
 *
 * Owns one ExposureTrack per zone (N = ae_zone_count). Closed-loop zones are
 * driven through update(); the derived mid zone and the min-gap floor are
 * written through setDerived().
 * closestZone() attributes a grabbed frame to a zone from its chunk-data
 * exposure time.
 */
class ZoneManager {
public:
    /**
     * Construct the manager and initialise all N zone tracks.
     *
     * @param zoneCount              N, number of zones. Must be >= 1.
     * @param zoneTargetPercents     Per-zone target brightness (% of pixelMax),
     *                               size zoneCount -- see
     *                               BracketingDriver::computeZoneTargetPercents().
     * @param pixelMax               Maximum pixel value for the active pixel format
     *                               (255 for BayerRG8).
     * @param zoneAlphas             Per-zone EMA rate, size zoneCount
     *                               (DriverConfig::aeZoneAlpha).
     * @param epsilon                Convergence threshold forwarded to every ExposureTrack.
     * @param defaultExposureUs      Initial exposure estimate forwarded to every
     *                               ExposureTrack, used until seed() is called.
     * @param exposureTieToleranceUs Tolerance forwarded to closestZone(); see its docs.
     * @param minExposureUs          Hardware exposure floor, shared by every zone.
     * @param maxExposureUs          Hardware exposure ceiling, shared by every zone.
     * @param minGainDb              Gain floor, shared by every zone.
     * @param maxGainDb              Gain ceiling, shared by every zone.
     * @param saturationBoostFraction  Forwarded to every ExposureTrack.
     */
    ZoneManager(int zoneCount, const std::vector<int>& zoneTargetPercents,
                double pixelMax, const std::vector<double>& zoneAlphas, double epsilon,
                double defaultExposureUs, double exposureTieToleranceUs,
                double minExposureUs, double maxExposureUs,
                double minGainDb, double maxGainDb,
                double saturationBoostFraction = 0.0);

    /**
     * Seed every zone's exposure from the AE-Once bootstrap result.
     *
     * @param exposureUs  Exposure time in µs, typically the value returned by
     *                    CameraGate::runAEOnce(). Clamped to [minExp, maxExp].
     * @param minExp      Minimum allowable exposure in µs (hardware floor);
     *                    also stored for closestZone() bookkeeping.
     * @param maxExp      Maximum allowable exposure in µs (hardware ceiling).
     */
    void seed(double exposureUs, double minExp, double maxExp);

    /**
     * @param zone  Zone index in [0, zoneCount).
     * @return Current exposure estimate in µs to program on the camera
     *         before the next grab at this zone.
     * @throws std::out_of_range if zone is out of range.
     */
    double exposureUs(int zone) const;

    /**
     * @param zone  Zone index in [0, zoneCount).
     * @return Current gain estimate in dB to program on the camera before
     *         the next grab at this zone.
     * @throws std::out_of_range if zone is out of range.
     */
    double gainDb(int zone) const;

    /**
     * @param zone  Zone index in [0, zoneCount).
     * @return true if this zone's last three frames all had brightness
     *         within epsilon of the target.
     * @throws std::out_of_range if zone is out of range.
     */
    bool converged(int zone) const;

    /**
     * @param zone  Zone index in [0, zoneCount).
     * @return true if this zone has not converged and both its exposure and
     *         gain are pinned (within a small tolerance) at a bound.
     * @throws std::out_of_range if zone is out of range.
     */
    bool unreachable(int zone) const;

    /**
     * Submit a brightness observation from a newly captured frame and
     * forward it to that zone's ExposureTrack::update().
     *
     * @param zone        Zone index in [0, zoneCount) — must match the zone
     *                     that was captured.
     * @param brightness  Measured brightness for that zone (0..pixelMax).
     * @param minExp      Minimum allowable exposure in µs (camera hardware limit).
     * @param maxExp      Maximum allowable exposure in µs (camera hardware limit).
     * @param minGain     Minimum allowable gain in dB.
     * @param maxGain     Maximum allowable gain in dB.
     * @throws std::out_of_range if zone is out of range.
     */
    void update(int zone, double brightness, double minExp, double maxExp,
                double minGain, double maxGain);

    /**
     * Overwrite a zone's exposure/gain, bypassing its control law (no EMA,
     * no convergence update). Used for the derived mid zone and the
     * min-gap floor.
     *
     * @param zone        Zone index in [0, zoneCount).
     * @param exposureUs  Exposure estimate in µs, already clamped by the caller.
     * @param gainDb      Gain estimate in dB, already clamped by the caller.
     * @throws std::out_of_range if zone is out of range.
     */
    void setDerived(int zone, double exposureUs, double gainDb);

    /**
     * Return the zone whose CURRENT exposure estimate is closest to a
     * frame's chunk-data exposure time.
     *
     * Zones within exposureTieToleranceUs of the best match are tied; the
     * tie resolves to expectedZone if it is among them, else to the HIGHEST
     * tied index. Note that in lockstep capture the estimates may already
     * have moved since the frame was triggered (the background thread
     * updates them), so a frame can be attributed to a zone other than the
     * one requested -- see DESIGN.md "Known limitations".
     *
     * @param actualExposureUs  Exposure (µs) the frame was captured with.
     * @param expectedZone      Tie-break hint (the zone requested for this frame).
     */
    int closestZone(double actualExposureUs, int expectedZone) const;

    /// @return N, the number of zones this manager owns.
    int zoneCount() const { return static_cast<int>(tracks_.size()); }

private:
    std::vector<ExposureTrack> tracks_;
    double exposureTieToleranceUs_;
    double minExp_;
    double maxExp_;
    double minGainDb_;
    double maxGainDb_;
};
