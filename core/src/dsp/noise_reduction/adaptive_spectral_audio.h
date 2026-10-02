#pragma once
#include "../processor.h"
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace dsp::noise_reduction {
    // Decision-directed spectral speech enhancer with speech-presence-aware
    // noise learning.  It follows the low-latency architecture documented by
    // UHSDR, adapted for SDR++'s 48 kHz stereo audio chain.
    class AdaptiveSpectralAudio : public Processor<stereo_t, stereo_t> {
        using base_type = Processor<stereo_t, stereo_t>;
    public:
        AdaptiveSpectralAudio() {}
        ~AdaptiveSpectralAudio() {
            if (!base_type::_block_init) return;
            base_type::stop();
            destroyBuffers();
        }

        void init(stream<stereo_t>* in, int strength, int preset) {
            this->strength = clampInt(strength, 0, 200);
            this->preset = clampInt(preset, 0, 6);
            configurePreset();
            initBuffers();
            base_type::init(in);
        }

        void setStrength(int value) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            strength = clampInt(value, 0, 200);
        }

        void setPreset(int value) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            preset = clampInt(value, 0, 6);
            configurePreset();
            clearState();
        }

        void setToneMode(bool value) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (toneMode == value) return;
            toneMode = value;
            clearState();
        }

        void reset() {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            clearState();
        }

        int process(int count, const stereo_t* in, stereo_t* out) {
            for (int sample = 0; sample < count; sample++) {
                inputRing[inputPos] = 0.5f * (in[sample].l + in[sample].r);
                inputPos = (inputPos + 1) % FFT_SIZE;
                if (filled < FFT_SIZE) filled++;

                const float value = olaRing[olaPos];
                olaRing[olaPos] = 0.0f;
                olaPos = (olaPos + 1) % FFT_SIZE;
                out[sample] = { value, value };

                if (filled == FFT_SIZE && ++sinceFrame >= HOP_SIZE) {
                    sinceFrame = 0;
                    processFrame();
                }
            }
            return count;
        }

        DEFAULT_PROC_RUN

    private:
        static constexpr int FFT_SIZE = 1024;
        static constexpr int HOP_SIZE = FFT_SIZE / 2;
        static constexpr int BINS = (FFT_SIZE / 2) + 1;

        static int clampInt(int value, int low, int high) {
            return value < low ? low : (value > high ? high : value);
        }

        void configurePreset() {
            // threshold bias, temporal smoothing, frequency smoothing, base floor
            static constexpr float settings[7][4] = {
                { 0.82f, 0.90f, 0.48f, 0.20f }, // Broadcast / music
                { 1.08f, 0.86f, 0.58f, 0.10f }, // Voice
                { 1.30f, 0.82f, 0.68f, 0.05f }, // Narrow band
                { 0.95f, 0.90f, 0.62f, 0.13f }, // Shortwave soft
                { 1.45f, 0.82f, 0.72f, 0.025f },// Shortwave hard
                { 1.24f, 0.91f, 0.78f, 0.035f },// London Shortwave
                { 0.72f, 0.93f, 0.38f, 0.26f }  // Hi-Fi
            };
            thresholdBias = settings[preset][0];
            timeSmooth = settings[preset][1];
            freqSmooth = settings[preset][2];
            baseFloor = settings[preset][3];
        }

        void initBuffers() {
            fftIn = (float*)fftwf_malloc(FFT_SIZE * sizeof(float));
            fftBins = (fftwf_complex*)fftwf_malloc(BINS * sizeof(fftwf_complex));
            ifftOut = (float*)fftwf_malloc(FFT_SIZE * sizeof(float));
            forwardPlan = fftwf_plan_dft_r2c_1d(FFT_SIZE, fftIn, fftBins, FFTW_ESTIMATE);
            backwardPlan = fftwf_plan_dft_c2r_1d(FFT_SIZE, fftBins, ifftOut, FFTW_ESTIMATE);
            window.resize(FFT_SIZE);
            inputRing.resize(FFT_SIZE);
            olaRing.resize(FFT_SIZE);
            power.resize(BINS);
            noisePower.resize(BINS);
            previousPosterior.assign(BINS, 1.0f);
            previousGain.assign(BINS, 1.0f);
            rawGain.resize(BINS);
            for (int i = 0; i < FFT_SIZE; i++) {
                const float phase = (2.0f * 3.14159265358979323846f * i) / (FFT_SIZE - 1);
                window[i] = std::sqrt(0.5f - 0.5f * std::cos(phase));
            }
            clearState();
        }

        void destroyBuffers() {
            fftwf_destroy_plan(forwardPlan);
            fftwf_destroy_plan(backwardPlan);
            fftwf_free(fftIn);
            fftwf_free(fftBins);
            fftwf_free(ifftOut);
        }

        void clearState() {
            std::fill(inputRing.begin(), inputRing.end(), 0.0f);
            std::fill(olaRing.begin(), olaRing.end(), 0.0f);
            std::fill(noisePower.begin(), noisePower.end(), 0.0f);
            std::fill(previousPosterior.begin(), previousPosterior.end(), 1.0f);
            std::fill(previousGain.begin(), previousGain.end(), 1.0f);
            inputPos = olaPos = filled = sinceFrame = 0;
        }

        void processFrame() {
            for (int i = 0; i < FFT_SIZE; i++) {
                fftIn[i] = inputRing[(inputPos + i) % FFT_SIZE] * window[i];
            }
            fftwf_execute(forwardPlan);

            float powerSum = 0.0f;
            float logPowerSum = 0.0f;
            for (int i = 0; i < BINS; i++) {
                power[i] = fftBins[i][0] * fftBins[i][0] + fftBins[i][1] * fftBins[i][1] + 1.0e-20f;
                powerSum += power[i];
                logPowerSum += std::log(power[i]);
            }
            const float arithmeticMean = powerSum / BINS;
            const float geometricMean = std::exp(logPowerSum / BINS);
            const float flatness = geometricMean / (arithmeticMean + 1.0e-20f);
            const float flatNoiseProbability = std::clamp((flatness - 0.30f) / 0.42f, 0.0f, 1.0f);

            const float normalized = strength / 200.0f;
            const float alpha = 0.94f + 0.055f * normalized;
            const float oversubtraction = thresholdBias * (0.55f + 3.45f * normalized);
            const float gainFloor = baseFloor * (1.0f - 0.92f * normalized);
            const float wetMix = std::sqrt(normalized);

            for (int i = 0; i < BINS; i++) {
                if (noisePower[i] == 0.0f) noisePower[i] = power[i] * 0.12f;
                const float posterior = power[i] / (noisePower[i] + 1.0e-20f);
                const float excess = posterior > 1.0f ? posterior - 1.0f : 0.0f;
                const float prior = alpha * previousGain[i] * previousGain[i] * previousPosterior[i]
                                  + (1.0f - alpha) * excess;

                // A compact speech-presence probability. Spectral flatness
                // suppresses false speech decisions on receiver hiss.
                const float binSpeech = prior / (prior + 1.0f);
                const float speechProbability = binSpeech * (1.0f - 0.92f * flatNoiseProbability);
                const float noiseBeta = 0.72f + 0.279f * speechProbability;
                noisePower[i] = noiseBeta * noisePower[i] + (1.0f - noiseBeta) * power[i];

                float gain;
                if (toneMode) {
                    // Narrow tones are sparse and stationary: retain prominent
                    // bins and reject the broadband residual aggressively.
                    const float toneSnr = power[i] / (noisePower[i] + 1.0e-20f);
                    gain = (toneSnr - oversubtraction) / (toneSnr + 1.0e-12f);
                }
                else {
                    // Decision-directed Wiener gain with adjustable spectral
                    // over-subtraction for the speech path.
                    gain = prior / (prior + oversubtraction);
                }
                rawGain[i] = std::clamp(gain, gainFloor, 1.0f);
                previousPosterior[i] = posterior;
            }

            for (int i = 0; i < BINS; i++) {
                const int left = i > 0 ? i - 1 : i;
                const int right = i + 1 < BINS ? i + 1 : i;
                const float neighbor = (rawGain[left] + rawGain[i] + rawGain[right]) / 3.0f;
                const float frequencyGain = rawGain[i] + freqSmooth * (neighbor - rawGain[i]);
                const float temporalGain = frequencyGain + timeSmooth * (previousGain[i] - frequencyGain);
                previousGain[i] = temporalGain;
                const float mixedGain = 1.0f - wetMix * (1.0f - temporalGain);
                fftBins[i][0] *= mixedGain;
                fftBins[i][1] *= mixedGain;
            }

            fftwf_execute(backwardPlan);
            constexpr float scale = 1.0f / FFT_SIZE;
            for (int i = 0; i < FFT_SIZE; i++) {
                olaRing[(olaPos + i) % FFT_SIZE] += ifftOut[i] * window[i] * scale;
            }
        }

        int strength = 45;
        int preset = 3;
        bool toneMode = false;
        float thresholdBias = 0.95f;
        float timeSmooth = 0.90f;
        float freqSmooth = 0.62f;
        float baseFloor = 0.13f;
        int inputPos = 0, olaPos = 0, filled = 0, sinceFrame = 0;
        float* fftIn = nullptr;
        fftwf_complex* fftBins = nullptr;
        float* ifftOut = nullptr;
        fftwf_plan forwardPlan = nullptr;
        fftwf_plan backwardPlan = nullptr;
        std::vector<float> window, inputRing, olaRing, power, noisePower;
        std::vector<float> previousPosterior, previousGain, rawGain;
    };
}
