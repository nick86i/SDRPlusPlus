#pragma once

#include <dsp/stream.h>
#include <dsp/types.h>
#include <sound/soundinterface.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class DreamAudioOutput final : public CSoundOutInterface {
public:
    explicit DreamAudioOutput(dsp::stream<dsp::stereo_t>& output) : output(output) {}

    ~DreamAudioOutput() override { Close(); }

    bool Init(int rate, int, bool) override {
        Close();
        sampleRate.store(rate);
        const std::size_t capacity = static_cast<std::size_t>((std::max)(rate / 2, OUTPUT_BLOCK_FRAMES * 4));
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            queue.assign(capacity, {});
            readPos = 0;
            writePos = 0;
            queuedFrames = 0;
            primed = false;
        }
        output.clearWriteStop();
        open.store(true);
        writerRunning.store(true);
        writerThread = std::thread(&DreamAudioOutput::writerLoop, this);
        return false;
    }

    bool Write(CVector<short>& input) override {
        if (!open.load()) return true;
        const int frames = (std::min)(input.Size() / 2, STREAM_BUFFER_SIZE);
        if (frames <= 0) return false;

        const float scale = gainLinear.load() / 32768.0f;
        std::lock_guard<std::mutex> lock(queueMutex);
        for (int i = 0; i < frames; ++i) {
            if (queuedFrames == queue.size()) {
                if (++readPos == queue.size()) readPos = 0;
                --queuedFrames;
            }
            queue[writePos].l = std::clamp(static_cast<float>(input[2 * i]) * scale, -1.0f, 1.0f);
            queue[writePos].r = std::clamp(static_cast<float>(input[2 * i + 1]) * scale, -1.0f, 1.0f);
            if (++writePos == queue.size()) writePos = 0;
            ++queuedFrames;
        }
        return false;
    }

    void Close() override {
        open.store(false);
        writerRunning.store(false);
        output.stopWriter();
        if (writerThread.joinable()) writerThread.join();
        std::lock_guard<std::mutex> lock(queueMutex);
        queue.clear();
        readPos = 0;
        writePos = 0;
        queuedFrames = 0;
        primed = false;
    }

    void Enumerate(std::vector<std::string>& choices, std::vector<std::string>& descriptions,
                   std::string& defaultOutput) override {
        choices = {"SDR++ Sinks"};
        descriptions = {"Decoded DRM audio"};
        defaultOutput = choices.front();
    }
    std::string GetDev() override { return "SDR++ Sinks"; }
    void SetDev(std::string) override {}
    std::string GetVersion() override { return "SDR++ audio stream adapter"; }
    int rate() const { return sampleRate.load(); }
    void setGainDb(float db) { gainLinear.store(std::pow(10.0f, db / 20.0f)); }
    void setMuted(bool value) {
        muted.store(value);
        if (value) {
            std::lock_guard<std::mutex> lock(queueMutex);
            readPos = writePos;
            queuedFrames = 0;
            primed = false;
        }
    }

private:
    void writerLoop() {
        while (writerRunning.load()) {
            std::fill_n(output.writeBuf, OUTPUT_BLOCK_FRAMES, dsp::stereo_t{});
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                if (!muted.load()) {
                    if (!primed && queuedFrames >= STARTUP_FRAMES) primed = true;
                    if (primed) {
                        const std::size_t frames = (std::min)(queuedFrames,
                            static_cast<std::size_t>(OUTPUT_BLOCK_FRAMES));
                        for (std::size_t i = 0; i < frames; ++i) {
                            output.writeBuf[i] = queue[readPos];
                            if (++readPos == queue.size()) readPos = 0;
                        }
                        queuedFrames -= frames;
                        if (frames < OUTPUT_BLOCK_FRAMES) primed = false;
                    }
                }
            }
            if (!output.swap(OUTPUT_BLOCK_FRAMES)) break;
        }
    }

    static constexpr int OUTPUT_BLOCK_FRAMES = 480; // 10 ms at DRM's 48 kHz output rate.
    static constexpr std::size_t STARTUP_FRAMES = OUTPUT_BLOCK_FRAMES * 2;

    dsp::stream<dsp::stereo_t>& output;
    std::atomic<bool> open{false};
    std::atomic<int> sampleRate{48000};
    std::atomic<float> gainLinear{1.0f};
    std::atomic<bool> muted{true};
    std::atomic<bool> writerRunning{false};
    std::thread writerThread;
    std::mutex queueMutex;
    std::vector<dsp::stereo_t> queue;
    std::size_t readPos = 0;
    std::size_t writePos = 0;
    std::size_t queuedFrames = 0;
    bool primed = false;
};
