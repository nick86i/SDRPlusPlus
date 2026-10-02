#include <core.h>
#include <external_decoder.h>
#include <gui/gui.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

SDRPP_MOD_INFO{
    "morse_decoder",
    "Automatic Morse/CW text decoder",
    "Nick / OpenAI",
    0, 1, 0,
    -1
};

namespace {
    constexpr double SAMPLE_RATE = 8000.0;
    // Narrow enough to reject adjacent CW signals, but wide enough that the
    // channel filter does not smear short key-up gaps used for Morse timing.
    constexpr double CHANNEL_BANDWIDTH = 1000.0;
    constexpr int DETECTION_BINS = 18; // 25 Hz bins across the 1 kHz VFO
    constexpr int BLOCK_SIZE = 80; // 10 ms timing resolution
    // Require 30 ms of stability in both directions. The captures contain
    // frequent 10-20 ms false marks and carrier dropouts; accepting either can
    // create an extra dot or split a real dash into multiple elements.
    constexpr int KEY_DEBOUNCE_BLOCKS = 3;
    constexpr double MIN_MARK_MS = 30.0;
    constexpr double PI = 3.14159265358979323846;

    char decodeMorse(const std::string& symbol) {
        static const std::unordered_map<std::string, char> table = {
            {".-",'A'}, {"-...",'B'}, {"-.-.",'C'}, {"-..",'D'}, {".",'E'},
            {"..-.",'F'}, {"--.",'G'}, {"....",'H'}, {"..",'I'}, {".---",'J'},
            {"-.-",'K'}, {".-..",'L'}, {"--",'M'}, {"-.",'N'}, {"---",'O'},
            {".--.",'P'}, {"--.-",'Q'}, {".-.",'R'}, {"...",'S'}, {"-",'T'},
            {"..-",'U'}, {"...-",'V'}, {".--",'W'}, {"-..-",'X'}, {"-.--",'Y'},
            {"--..",'Z'}, {"-----",'0'}, {".----",'1'}, {"..---",'2'},
            {"...--",'3'}, {"....-",'4'}, {".....",'5'}, {"-....",'6'},
            {"--...",'7'}, {"---..",'8'}, {"----.",'9'}, {".-.-.-",'.'},
            {"--..--",','}, {"..--..",'?'}, {"-..-.",'/'}, {"-....-",'-'},
            {"-.--.",'('}, {"-.--.-",')'}, {".-..-.",'\"'}, {".-.-.",'+'},
            {"-...-",'='}, {"---...",':'}, {"-.-.-.",';'}, {".--.-.",'@'}
        };
        const auto found = table.find(symbol);
        return found == table.end() ? '?' : found->second;
    }
}

class MorseDecoderModule : public ModuleManager::Instance {
public:
    explicit MorseDecoderModule(std::string instanceName) : name(std::move(instanceName)) {
        createVFO();
        inputSink.init(vfo->output, iqHandler, this);
        inputSink.start();
        core::modComManager.registerInterface("external_decoder", "external_decoder.morse", externalInterfaceHandler, this);
    }

    ~MorseDecoderModule() override {
        core::modComManager.unregisterInterface("external_decoder.morse");
        externalAttached = false;
        monitoredAudio.stopWriter();
        inputSink.stop();
        if (vfo) sigpath::vfoManager.deleteVFO(vfo);
    }

    void postInit() override {}

    void enable() override {
        if (enabled) return;
        if (externalAttached.load()) { enabled = true; return; }
        createVFO();
        inputSink.setInput(vfo->output);
        inputSink.start();
        enabled = true;
    }

    void disable() override {
        // Radio owns the IQ and audio streams while Morse mode is selected.
        if (externalAttached.load()) return;
        if (!enabled) return;
        inputSink.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        vfo = nullptr;
        enabled = false;
    }

    bool isEnabled() override { return enabled || externalAttached.load(); }

private:
    static void externalInterfaceHandler(int command, void* in, void* out, void* ctx) {
        auto* self = static_cast<MorseDecoderModule*>(ctx);
        switch (command) {
        case EXTERNAL_DECODER_GET_INFO:
            if (out) {
                auto* info = static_cast<ExternalDecoderInfo*>(out);
                info->id = "morse";
                info->displayName = "Morse";
                info->inputSampleRate = SAMPLE_RATE;
                info->audioSampleRate = SAMPLE_RATE;
                info->defaultBandwidth = CHANNEL_BANDWIDTH;
                info->minBandwidth = CHANNEL_BANDWIDTH;
                info->maxBandwidth = CHANNEL_BANDWIDTH;
                info->snapInterval = 100.0;
                info->audioOutput = &self->monitoredAudio;
            }
            break;
        case EXTERNAL_DECODER_ATTACH:
            if (in) self->attachExternal(static_cast<ExternalDecoderAttachArgs*>(in)->iqInput);
            break;
        case EXTERNAL_DECODER_DETACH:
            self->detachExternal();
            break;
        case EXTERNAL_DECODER_DRAW_MENU:
            ImGui::PushID("embedded_morse_decoder");
            menuHandler(self);
            ImGui::PopID();
            break;
        }
    }

    void attachExternal(dsp::stream<dsp::complex_t>* input) {
        if (!input) return;
        externalAttached = false;
        monitoredAudio.stopWriter();
        inputSink.stop();
        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
        monitoredAudio.clearWriteStop();
        audioFill = 0;
        monitorPhase = 0.0;
        resetTiming();
        inputSink.setInput(input);
        enabled = false;
        externalAttached = true;
        inputSink.start();
    }

    void detachExternal() {
        if (!externalAttached.exchange(false)) return;
        monitoredAudio.stopWriter();
        inputSink.stop();
        audioFill = 0;
        blockFill = 0;
        keyDown = false;
        previousKeyDown = false;
    }

    void createVFO() {
        const double visibleBandwidth = gui::waterfall.getBandwidth();
        const double offset = std::clamp(0.0, -visibleBandwidth / 2.0, visibleBandwidth / 2.0);
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER,
                                            offset, CHANNEL_BANDWIDTH, SAMPLE_RATE,
                                            CHANNEL_BANDWIDTH, CHANNEL_BANDWIDTH, true);
        vfo->setSnapInterval(100);
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<MorseDecoderModule*>(ctx);
        if (!self->enabled && !self->externalAttached.load()) return;

        ImGui::TextUnformatted("Automatic CW decoder");
        ImGui::Separator();
        const bool keyed = self->keyDown.load();
        ImGui::TextUnformatted("Signal:");
        ImGui::SameLine();
        if (keyed) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "KEY DOWN");
        else ImGui::TextUnformatted("idle");
        ImGui::Text("Speed: %.1f WPM", self->wpm.load());
        ImGui::Text("Tone confidence: %.0f%%", self->confidence.load() * 100.0f);

        bool automatic = self->autoThreshold.load();
        float threshold = automatic ? self->effectiveThreshold.load() : self->detectionThreshold.load();
        ImGui::SetNextItemWidth((std::max)(80.0f, ImGui::GetContentRegionAvail().x - 62.0f));
        if (automatic) ImGui::BeginDisabled();
        if (ImGui::SliderFloat("##cw_threshold", &threshold, 0.08f, 0.65f, "Threshold %.2f")) {
            self->detectionThreshold.store(threshold);
        }
        if (automatic) ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Checkbox("Auto##cw_auto_threshold", &automatic)) {
            self->autoThreshold.store(automatic);
            self->resetThresholdEstimator();
        }

        if (ImGui::Button("Clear##cw_clear")) self->clearText();
        ImGui::SameLine();
        if (ImGui::Button("Reset timing##cw_reset")) self->resetTiming();
        ImGui::Separator();

        std::string displayed;
        std::string pending;
        {
            std::lock_guard<std::mutex> lock(self->textMutex);
            displayed = self->decodedText;
            pending = self->currentSymbol;
        }
        ImGui::TextWrapped("%s", displayed.empty() ? "Waiting for Morse..." : displayed.c_str());
        if (!pending.empty()) ImGui::TextDisabled("Current: %s", pending.c_str());
        ImGui::Spacing();
        ImGui::TextWrapped("Center the Radio VFO over the CW signal. The 1 kHz channel automatically tracks a carrier within +/-450 Hz.");
    }

    void clearText() {
        std::lock_guard<std::mutex> lock(textMutex);
        decodedText.clear();
        currentSymbol.clear();
    }

    void resetTiming() {
        dotMs = 60.0;
        wpm.store(20.0f);
        stateBlocks = 0;
        characterCommitted = false;
        wordCommitted = false;
        markDurations.clear();
        fastTimingAcquire = true;
        idleTimingReset = false;
        rawKeyDown = false;
        rawStateBlocks = 0;
        resetThresholdEstimator();
    }

    void resetThresholdEstimator() {
        confidenceWindowMin = 1.0f;
        confidenceWindowBlocks = 0;
        estimatedNoiseFloor = 0.10f;
        effectiveThreshold.store(detectionThreshold.load());
        envelopeInitialized = false;
        envelopeReadyBlocks = 0;
        envelopeNoiseFloor = 0.0f;
        envelopeSignalPeak = 0.0f;
    }

    float updateEnvelopeScore(float amplitude) {
        if (!envelopeInitialized) {
            envelopeNoiseFloor = amplitude;
            envelopeSignalPeak = amplitude;
            envelopeInitialized = true;
            return 0.0f;
        }

        // Noise follows downward changes quickly but rises slowly so that a
        // keyed carrier is not absorbed into the floor. The signal peak has an
        // immediate attack and a roughly ten-second decay at 10 ms per block.
        const float noiseAlpha = amplitude < envelopeNoiseFloor ? 0.20f : 0.002f;
        envelopeNoiseFloor += (amplitude - envelopeNoiseFloor) * noiseAlpha;
        if (amplitude > envelopeSignalPeak) envelopeSignalPeak = amplitude;
        else envelopeSignalPeak += (amplitude - envelopeSignalPeak) * 0.001f;

        const float dynamicRange = envelopeSignalPeak - envelopeNoiseFloor;
        if (envelopeSignalPeak > envelopeNoiseFloor * 2.0f && dynamicRange > 1.0e-12f) {
            ++envelopeReadyBlocks;
        }
        else {
            envelopeReadyBlocks = 0;
        }

        if (envelopeReadyBlocks < 3) return 0.0f;
        return std::clamp((amplitude - envelopeNoiseFloor) / dynamicRange, 0.0f, 1.0f);
    }

    float updateAutomaticThreshold(float score) {
        confidenceWindowMin = (std::min)(confidenceWindowMin, score);
        if (++confidenceWindowBlocks >= 100) { // one-second minimum at 10 ms/block
            estimatedNoiseFloor = estimatedNoiseFloor * 0.70f + confidenceWindowMin * 0.30f;
            confidenceWindowMin = 1.0f;
            confidenceWindowBlocks = 0;
        }
        const float threshold = std::clamp(estimatedNoiseFloor + 0.14f, 0.12f, 0.62f);
        effectiveThreshold.store(threshold);
        return threshold;
    }

    void commitCharacter() {
        std::lock_guard<std::mutex> lock(textMutex);
        if (currentSymbol.empty()) return;
        decodedText.push_back(decodeMorse(currentSymbol));
        currentSymbol.clear();
        if (decodedText.size() > 2048) decodedText.erase(0, decodedText.size() - 1536);
    }

    void processKeyState(bool down) {
        constexpr double blockMs = BLOCK_SIZE * 1000.0 / SAMPLE_RATE;
        if (down == previousKeyDown) {
            ++stateBlocks;
        }
        else {
            const double durationMs = stateBlocks * blockMs;
            if (previousKeyDown && stateBlocks > 0) {
                // A one- or two-block impulse is normally threshold chatter,
                // not a real Morse element. Do not let it corrupt timing.
                if (durationMs < MIN_MARK_MS) {
                    stateBlocks = 1;
                    previousKeyDown = down;
                    keyDown.store(down);
                    return;
                }

                markDurations.push_back(durationMs);
                if (markDurations.size() > 24) markDurations.erase(markDurations.begin());

                // Fit recent marks to Morse's 1:3 dot/dash relationship. The
                // clipped mean tolerates a few remaining short impulses, and
                // preferring the first equal-cost candidate resolves an
                // all-dash sample toward its implied (shorter) dot duration.
                if (markDurations.size() >= 3) {
                    double bestCandidate = dotMs;
                    double bestCost = 1.0e9;
                    for (double candidate = 25.0; candidate <= 240.0; candidate += 1.0) {
                        double cost = 0.0;
                        for (double mark : markDurations) {
                            const double dotError = std::abs(mark - candidate) / candidate;
                            const double dashDuration = candidate * 3.0;
                            const double dashError = std::abs(mark - dashDuration) / dashDuration;
                            cost += (std::min)(1.0, (std::min)(dotError, dashError));
                        }
                        cost /= static_cast<double>(markDurations.size());
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestCandidate = candidate;
                        }
                    }
                    if (fastTimingAcquire) {
                        dotMs = bestCandidate;
                        fastTimingAcquire = false;
                    }
                    else {
                        dotMs = std::clamp(dotMs * 0.65 + bestCandidate * 0.35, 25.0, 240.0);
                    }
                    wpm.store(static_cast<float>(1200.0 / dotMs));
                }

                const bool dash = durationMs > dotMs * 2.0;
                {
                    std::lock_guard<std::mutex> lock(textMutex);
                    if (currentSymbol.size() < 8) currentSymbol.push_back(dash ? '-' : '.');
                }
                characterCommitted = false;
                wordCommitted = false;
            }
            stateBlocks = 1;
            previousKeyDown = down;
        }

        if (!down) {
            const double gapMs = stateBlocks * blockMs;
            if (!characterCommitted && gapMs >= dotMs * 2.2) {
                commitCharacter();
                characterCommitted = true;
            }
            if (!wordCommitted && gapMs >= dotMs * 6.0) {
                std::lock_guard<std::mutex> lock(textMutex);
                if (!decodedText.empty() && decodedText.back() != ' ') decodedText.push_back(' ');
                wordCommitted = true;
            }
            // Treat a multi-second idle period as the boundary between
            // transmissions. Discard the previous sender's speed history and
            // return to a neutral 20 WPM baseline until three new marks provide
            // a reliable 1:3 fit.
            if (!idleTimingReset && gapMs >= 2000.0) {
                markDurations.clear();
                dotMs = 60.0;
                wpm.store(20.0f);
                fastTimingAcquire = true;
                idleTimingReset = true;
            }
        }
        else {
            idleTimingReset = false;
        }
        keyDown.store(down);
    }

    void processBlock(const dsp::complex_t* samples) {
        auto coherentPowerAt = [&](double frequency) {
            const double step = -2.0 * PI * frequency / SAMPLE_RATE;
            double phase = 0.0;
            double sumRe = 0.0;
            double sumIm = 0.0;
            for (int i = 0; i < BLOCK_SIZE; ++i) {
                const double c = std::cos(phase);
                const double s = std::sin(phase);
                sumRe += samples[i].re * c - samples[i].im * s;
                sumIm += samples[i].re * s + samples[i].im * c;
                phase += step;
            }
            return (sumRe * sumRe + sumIm * sumIm) / (BLOCK_SIZE * BLOCK_SIZE);
        };

        double bestPower = 0.0;
        // Re-evaluate the narrow VFO passband for every block. There is no
        // persistent tone lock to reacquire or to drift onto the wrong peak.
        for (int bin = -DETECTION_BINS; bin <= DETECTION_BINS; ++bin) {
            const double frequency = bin * 25.0;
            const double power = coherentPowerAt(frequency);
            if (power > bestPower) {
                bestPower = power;
            }
        }

        const float score = updateEnvelopeScore(static_cast<float>(std::sqrt(bestPower)));
        confidence.store(score);
        // Hysteresis prevents threshold chatter from creating false dots.
        const float onThreshold = autoThreshold.load()
            ? updateAutomaticThreshold(score) : detectionThreshold.load();
        if (!autoThreshold.load()) effectiveThreshold.store(onThreshold);
        const float offThreshold = onThreshold * 0.70f;
        const bool rawDown = keyDown.load() ? score >= offThreshold : score >= onThreshold;

        if (rawDown == rawKeyDown) ++rawStateBlocks;
        else {
            rawKeyDown = rawDown;
            rawStateBlocks = 1;
        }
        if (rawStateBlocks >= KEY_DEBOUNCE_BLOCKS) processKeyState(rawKeyDown);
        else processKeyState(keyDown.load());
    }

    static void iqHandler(dsp::complex_t* samples, int count, void* ctx) {
        auto* self = static_cast<MorseDecoderModule*>(ctx);
        if (count <= 0) return;
        for (int i = 0; i < count; ++i) {
            if (self->externalAttached.load()) {
                const float monitor = static_cast<float>(samples[i].re * std::cos(self->monitorPhase) -
                                                         samples[i].im * std::sin(self->monitorPhase));
                self->monitoredAudio.writeBuf[self->audioFill] = {monitor, monitor};
                self->monitorPhase += 2.0 * PI * 700.0 / SAMPLE_RATE;
                if (self->monitorPhase > 2.0 * PI) self->monitorPhase -= 2.0 * PI;
                if (++self->audioFill == BLOCK_SIZE) {
                    if (!self->monitoredAudio.swap(BLOCK_SIZE)) return;
                    self->audioFill = 0;
                }
            }
            self->blockBuffer[self->blockFill++] = samples[i];
            if (self->blockFill == BLOCK_SIZE) {
                self->processBlock(self->blockBuffer.data());
                self->blockFill = 0;
            }
        }
    }

    std::string name;
    bool enabled = true;
    VFOManager::VFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> inputSink;
    dsp::stream<dsp::stereo_t> monitoredAudio;
    std::array<dsp::complex_t, BLOCK_SIZE> blockBuffer{};
    int blockFill = 0;
    int audioFill = 0;
    double monitorPhase = 0.0;
    std::atomic<bool> externalAttached{false};

    // A narrow channel concentrates noise into fewer bins, so its confidence
    // floor is higher than with the former 2.4 kHz VFO.
    std::atomic<float> detectionThreshold{0.40f};
    std::atomic<float> effectiveThreshold{0.40f};
    std::atomic<bool> autoThreshold{false};
    std::atomic<float> confidence{0.0f};
    std::atomic<float> wpm{15.0f};
    std::atomic<bool> keyDown{false};
    bool rawKeyDown = false;
    int rawStateBlocks = 0;
    bool previousKeyDown = false;
    int stateBlocks = 0;
    double dotMs = 60.0;
    std::vector<double> markDurations;
    bool fastTimingAcquire = true;
    bool idleTimingReset = false;
    bool characterCommitted = false;
    bool wordCommitted = false;
    float confidenceWindowMin = 1.0f;
    int confidenceWindowBlocks = 0;
    float estimatedNoiseFloor = 0.10f;
    bool envelopeInitialized = false;
    int envelopeReadyBlocks = 0;
    float envelopeNoiseFloor = 0.0f;
    float envelopeSignalPeak = 0.0f;

    std::mutex textMutex;
    std::string decodedText;
    std::string currentSymbol;
};

MOD_EXPORT void _INIT_() {}
MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new MorseDecoderModule(std::move(name));
}
MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<MorseDecoderModule*>(instance);
}
MOD_EXPORT void _END_() {}
