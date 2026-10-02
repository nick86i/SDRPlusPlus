#pragma once
#include "../processor.h"
#include <algorithm>
#include <cmath>

namespace dsp::noise_reduction {
    // A lightweight adaptive downward expander for demodulated audio.  Unlike
    // FMIF, it does not discard modulation sidebands and is therefore safe for
    // AM and SSB-family demodulators.
    class AdaptiveAudio : public Processor<stereo_t, stereo_t> {
        using base_type = Processor<stereo_t, stereo_t>;
    public:
        void init(stream<stereo_t>* in, int strength) {
            setStrengthInternal(strength);
            base_type::init(in);
        }

        void setStrength(int strength) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            setStrengthInternal(strength);
        }

        void reset() {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            envelope = noiseFloor = 0.0f;
            gain = 1.0f;
        }

        int process(int count, const stereo_t* in, stereo_t* out) {
            for (int i = 0; i < count; i++) {
                const float leftMagnitude = std::abs(in[i].l);
                const float rightMagnitude = std::abs(in[i].r);
                const float magnitude = leftMagnitude > rightMagnitude ? leftMagnitude : rightMagnitude;
                if (envelope == 0.0f) envelope = noiseFloor = magnitude;

                const float envRate = magnitude > envelope ? 0.08f : 0.0015f;
                envelope += envRate * (magnitude - envelope);

                // Follow reductions quickly, but increases very slowly so that
                // speech and music are not learned as noise.
                const float noiseRate = envelope < noiseFloor ? 0.01f : 0.0000025f;
                noiseFloor += noiseRate * (envelope - noiseFloor);

                const float threshold = 1.15f + (0.035f * level);
                const float minimumGain = 1.0f - (0.0095f * level);
                const float divisor = envelope > 1.0e-9f ? envelope : 1.0e-9f;
                const float rawGain = (envelope - (noiseFloor * threshold)) / divisor;
                const float wanted = rawGain < minimumGain ? minimumGain : (rawGain > 1.0f ? 1.0f : rawGain);
                const float gainRate = wanted > gain ? 0.025f : 0.0025f;
                gain += gainRate * (wanted - gain);

                out[i].l = in[i].l * gain;
                out[i].r = in[i].r * gain;
            }
            return count;
        }

        DEFAULT_PROC_RUN

    private:
        void setStrengthInternal(int strength) { level = strength < 0 ? 0 : (strength > 100 ? 100 : strength); }

        int level = 45;
        float envelope = 0.0f;
        float noiseFloor = 0.0f;
        float gain = 1.0f;
    };
}
