#pragma once

#include <dsp/types.h>
#include <sound/soundinterface.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

// Blocking stereo PCM adapter expected by Dream. Left carries I and right Q.
class DreamIqInput final : public CSoundInInterface {
public:
    bool Init(int sampleRate, int, bool) override {
        std::lock_guard<std::mutex> lock(mutex);
        validRate = sampleRate == 48000;
        closed = false;
        return !validRate;
    }

    bool Read(CVector<short>& output) override {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock, [&] { return closed || samples.size() >= static_cast<size_t>(output.Size()); });
        if (closed) return true;
        for (int i = 0; i < output.Size(); ++i) {
            output[i] = samples.front();
            samples.pop_front();
        }
        return false;
    }

    void push(const dsp::complex_t* input, int count) {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed || !validRate) return;
        constexpr size_t maxQueued = 48000 * 2;
        for (int i = 0; i < count; ++i) {
            if (samples.size() + 2 > maxQueued) {
                samples.pop_front();
                samples.pop_front();
                ++droppedFrames;
            }
            samples.push_back(toPcm(input[i].re));
            samples.push_back(toPcm(input[i].im));
        }
        ready.notify_one();
    }

    void Close() override {
        std::lock_guard<std::mutex> lock(mutex);
        closed = true;
        samples.clear();
        ready.notify_all();
    }

    void Enumerate(std::vector<std::string>& choices, std::vector<std::string>& descriptions,
                   std::string& defaultInput) override {
        choices = {"SDR++ IQ stream"};
        descriptions = {"48 kHz complex IQ"};
        defaultInput = choices.front();
    }
    std::string GetDev() override { return "SDR++ IQ stream"; }
    void SetDev(std::string) override {}
    std::string GetVersion() override { return "SDR++ in-memory IQ adapter"; }
    uint64_t dropped() const { return droppedFrames.load(); }

private:
    static short toPcm(float value) {
        value = std::clamp(value, -1.0f, 0.999969f);
        return static_cast<short>(value * 32768.0f);
    }

    mutable std::mutex mutex;
    std::condition_variable ready;
    std::deque<short> samples;
    bool closed = true;
    bool validRate = false;
    std::atomic<uint64_t> droppedFrames{0};
};
