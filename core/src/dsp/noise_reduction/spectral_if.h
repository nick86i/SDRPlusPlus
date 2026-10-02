#pragma once
#include "../processor.h"
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace dsp::noise_reduction {
    class SpectralIF : public Processor<complex_t, complex_t> {
        using base_type = Processor<complex_t, complex_t>;
    public:
        SpectralIF() {}
        ~SpectralIF() {
            if (!base_type::_block_init) return;
            base_type::stop();
            destroyBuffers();
        }

        void init(stream<complex_t>* in, int strength, int preset) {
            this->strength = std::clamp(strength, 0, 200);
            this->preset = std::clamp(preset, 0, 6);
            configurePreset();
            initBuffers();
            base_type::init(in);
        }

        void setStrength(int value) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            strength = std::clamp(value, 0, 200);
        }

        void setPreset(int value) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            preset = std::clamp(value, 0, 6);
            configurePreset();
            clearState();
        }

        void reset() {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            clearState();
        }

        int process(int count, const complex_t* in, complex_t* out) {
            for (int sample = 0; sample < count; sample++) {
                inputRing[inputPos] = in[sample];
                inputPos = (inputPos + 1) % FFT_SIZE;
                if (filled < FFT_SIZE) filled++;

                out[sample] = olaRing[olaPos];
                olaRing[olaPos] = { 0.0f, 0.0f };
                olaPos = (olaPos + 1) % FFT_SIZE;

                if (filled == FFT_SIZE && ++sinceFrame >= HOP_SIZE) {
                    sinceFrame = 0;
                    processFrame();
                }
            }
            return count;
        }

        DEFAULT_PROC_RUN

    private:
        static constexpr int FFT_SIZE = 512;
        static constexpr int HOP_SIZE = FFT_SIZE / 4;

        void configurePreset() {
            // threshold, maximum attenuation, frequency smoothing, time smoothing
            static constexpr float settings[7][4] = {
                { 0.75f, 0.25f, 0.45f, 0.70f }, // Broadcast / music
                { 1.00f, 0.12f, 0.55f, 0.78f }, // Voice
                { 1.20f, 0.07f, 0.68f, 0.82f }, // Narrow band
                { 0.90f, 0.16f, 0.62f, 0.82f }, // Shortwave soft
                { 1.35f, 0.04f, 0.72f, 0.88f }, // Shortwave hard
                { 1.12f, 0.06f, 0.78f, 0.91f }, // London Shortwave
                { 0.85f, 0.20f, 0.35f, 0.65f }  // Hi-Fi
            };
            threshold = settings[preset][0];
            floorGain = settings[preset][1];
            freqSmooth = settings[preset][2];
            timeSmooth = settings[preset][3];
        }

        void initBuffers() {
            fftIn = (complex_t*)fftwf_malloc(FFT_SIZE * sizeof(complex_t));
            fftOut = (complex_t*)fftwf_malloc(FFT_SIZE * sizeof(complex_t));
            ifftOut = (complex_t*)fftwf_malloc(FFT_SIZE * sizeof(complex_t));
            forwardPlan = fftwf_plan_dft_1d(FFT_SIZE, (fftwf_complex*)fftIn, (fftwf_complex*)fftOut, FFTW_FORWARD, FFTW_ESTIMATE);
            backwardPlan = fftwf_plan_dft_1d(FFT_SIZE, (fftwf_complex*)fftOut, (fftwf_complex*)ifftOut, FFTW_BACKWARD, FFTW_ESTIMATE);
            window.resize(FFT_SIZE);
            inputRing.resize(FFT_SIZE);
            olaRing.resize(FFT_SIZE);
            noise.resize(FFT_SIZE);
            previousGain.assign(FFT_SIZE, 1.0f);
            rawGain.resize(FFT_SIZE);
            magnitudeBuffer.resize(FFT_SIZE);
            for (int i = 0; i < FFT_SIZE; i++) {
                const float phase = (2.0f * 3.14159265358979323846f * i) / (FFT_SIZE - 1);
                window[i] = 0.5f - (0.5f * std::cos(phase));
            }
            clearState();
        }

        void destroyBuffers() {
            fftwf_destroy_plan(forwardPlan);
            fftwf_destroy_plan(backwardPlan);
            fftwf_free(fftIn);
            fftwf_free(fftOut);
            fftwf_free(ifftOut);
        }

        void clearState() {
            std::fill(inputRing.begin(), inputRing.end(), complex_t{ 0.0f, 0.0f });
            std::fill(olaRing.begin(), olaRing.end(), complex_t{ 0.0f, 0.0f });
            std::fill(noise.begin(), noise.end(), 0.0f);
            std::fill(previousGain.begin(), previousGain.end(), 1.0f);
            inputPos = olaPos = filled = sinceFrame = 0;
            broadbandGain = 1.0f;
        }

        void processFrame() {
            for (int i = 0; i < FFT_SIZE; i++) {
                const complex_t value = inputRing[(inputPos + i) % FFT_SIZE];
                fftIn[i] = { value.re * window[i], value.im * window[i] };
            }
            fftwf_execute(forwardPlan);

            const float normalizedStrength = strength / 200.0f;
            const float amount = 0.35f + (3.65f * normalizedStrength);
            float magnitudeSum = 0.0f;
            float logMagnitudeSum = 0.0f;
            for (int i = 0; i < FFT_SIZE; i++) {
                magnitudeBuffer[i] = std::sqrt((fftOut[i].re * fftOut[i].re) + (fftOut[i].im * fftOut[i].im));
                magnitudeSum += magnitudeBuffer[i];
                logMagnitudeSum += std::log(magnitudeBuffer[i] + 1.0e-12f);
            }
            const float arithmeticMean = magnitudeSum / FFT_SIZE;
            const float geometricMean = std::exp(logMagnitudeSum / FFT_SIZE);
            const float spectralFlatness = geometricMean / (arithmeticMean + 1.0e-12f);
            // White receiver hiss is spectrally flat.  A carrier, speech, or
            // music produces a less-flat spectrum and releases this broadband
            // attenuation smoothly.
            const float noiseLikelihood = std::clamp((spectralFlatness - 0.58f) / 0.22f, 0.0f, 1.0f);
            const float emptyChannelFloor = 1.0f - (0.975f * normalizedStrength);
            const float wantedBroadbandGain = 1.0f - noiseLikelihood * (1.0f - emptyChannelFloor);
            const float broadbandRate = wantedBroadbandGain < broadbandGain ? 0.12f : 0.035f;
            broadbandGain += broadbandRate * (wantedBroadbandGain - broadbandGain);

            for (int i = 0; i < FFT_SIZE; i++) {
                const float magnitude = magnitudeBuffer[i];
                float adjacentFloor = 0.0f;
                for (int offset = 4; offset <= 11; offset++) {
                    adjacentFloor += magnitudeBuffer[(i + offset) % FFT_SIZE];
                    adjacentFloor += magnitudeBuffer[(i + FFT_SIZE - offset) % FFT_SIZE];
                }
                adjacentFloor /= 16.0f;
                // The neighboring-bin ceiling prevents a stable AM carrier or
                // heterodyne from being mistaken for stationary broadband noise.
                const float adjacentCeiling = adjacentFloor * 1.35f;
                const float candidate = magnitude < adjacentCeiling ? magnitude : adjacentCeiling;
                if (noise[i] == 0.0f) noise[i] = candidate * 0.35f;
                const float risingRate = noiseLikelihood > 0.70f ? 0.045f : 0.0012f;
                const float rate = candidate < noise[i] ? 0.18f : risingRate;
                noise[i] += rate * (candidate - noise[i]);
                const float divisor = magnitude > 1.0e-12f ? magnitude : 1.0e-12f;
                const float ratio = noise[i] / divisor;
                const float effectiveFloor = floorGain * (1.0f - (0.85f * normalizedStrength));
                rawGain[i] = std::clamp(1.0f - (threshold * amount * ratio), effectiveFloor, 1.0f);
            }

            for (int i = 0; i < FFT_SIZE; i++) {
                const int prev = (i + FFT_SIZE - 1) % FFT_SIZE;
                const int next = (i + 1) % FFT_SIZE;
                const float neighborhood = (rawGain[prev] + rawGain[i] + rawGain[next]) / 3.0f;
                const float smoothed = rawGain[i] + freqSmooth * (neighborhood - rawGain[i]);
                const float temporalGain = smoothed + timeSmooth * (previousGain[i] - smoothed);
                previousGain[i] = temporalGain;
                const float gain = temporalGain * broadbandGain;
                fftOut[i].re *= gain;
                fftOut[i].im *= gain;
            }

            fftwf_execute(backwardPlan);
            constexpr float scale = 1.0f / (FFT_SIZE * 1.5f);
            for (int i = 0; i < FFT_SIZE; i++) {
                const int pos = (olaPos + i) % FFT_SIZE;
                olaRing[pos].re += ifftOut[i].re * window[i] * scale;
                olaRing[pos].im += ifftOut[i].im * window[i] * scale;
            }
        }

        int strength = 45;
        int preset = 3;
        float threshold = 0.9f;
        float floorGain = 0.16f;
        float freqSmooth = 0.62f;
        float timeSmooth = 0.82f;
        float broadbandGain = 1.0f;
        int inputPos = 0, olaPos = 0, filled = 0, sinceFrame = 0;
        complex_t* fftIn = nullptr;
        complex_t* fftOut = nullptr;
        complex_t* ifftOut = nullptr;
        fftwf_plan forwardPlan = nullptr;
        fftwf_plan backwardPlan = nullptr;
        std::vector<float> window, noise, previousGain, rawGain, magnitudeBuffer;
        std::vector<complex_t> inputRing, olaRing;
    };
}
