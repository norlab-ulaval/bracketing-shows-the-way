#pragma once
#include <vector>

/**
 * @file BracketingSequencer.h
 *
 * Cycles through zone indices indefinitely. BracketingDriver constructs it
 * with pingPong = true: a triangle-wave sweep 0,1,...,N-1,N-2,...,1, i.e.
 * 0,1,2,1,0,1,2,1,... for N = 3 (period 4, sequence_index 0..3). With the
 * ping-pong order, B-Adapt's low and high each get one fresh update per
 * period, and every frame is adjacent to the mid (bridge) frame.
 *
 * Example (N = 3):
 *   BracketingSequencer seq(3, true);
 *   seq.currentZone();       // 0, 1, 2, 1, 0, ...
 *   seq.currentStepIndex();  // 0, 1, 2, 3, 0, ...
 *   seq.advance();
 */
class BracketingSequencer {
public:
    /**
     * @param zoneCount  N >= 1.
     * @param pingPong   false: round-robin 0..N-1. true: triangle wave
     *                   0..N-1..1.
     */
    explicit BracketingSequencer(int zoneCount, bool pingPong = false) {
        sequence_.reserve(static_cast<size_t>(zoneCount));
        for (int i = 0; i < zoneCount; ++i)
            sequence_.push_back(i);
        if (pingPong)
            for (int i = zoneCount - 2; i >= 1; --i)
                sequence_.push_back(i);
    }

    /// Zone index for the current step. Does not advance.
    int currentZone() const { return sequence_[static_cast<size_t>(stepIndex_)]; }

    /// 0-based position within the cycle, in [0, sequence length).
    int currentStepIndex() const { return stepIndex_; }

    /// Advance one step, wrapping to step 0 at the end of the cycle.
    void advance() { stepIndex_ = (stepIndex_ + 1) % static_cast<int>(sequence_.size()); }

private:
    std::vector<int> sequence_;
    int stepIndex_ = 0;
};
