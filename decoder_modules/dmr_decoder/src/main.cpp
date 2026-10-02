// mbelib also ships a file named config.h. Include SDR++'s configuration
// declaration first so core.h cannot resolve that unrelated header instead.
#include "../../../core/src/config.h"
#include <core.h>
#include <external_decoder.h>
#include <gui/gui.h>
#include <imgui.h>
#include <module.h>
#include <dsp/sink/handler_sink.h>
#include <dsd_decoder.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>

SDRPP_MOD_INFO{
    "dmr_decoder",
    "DMR Tier II voice decoder",
    "Nick / OpenAI; decoding by DSDcc and mbelib",
    0, 1, 0,
    -1
};

namespace {
constexpr double INPUT_RATE = 48000.0;
constexpr double AUDIO_RATE = 48000.0;
constexpr double CHANNEL_BANDWIDTH = 12500.0;
constexpr double PI = 3.14159265358979323846;
constexpr int AUDIO_BLOCK = 256;
constexpr int LOCK_CONFIRM_SAMPLES = 36000; // 750 ms, longer than one false superframe
constexpr int LOCK_LOSS_SAMPLES = 4800;     // 100 ms after DSDcc drops sync

bool isDMRSync(DSDcc::DSDDecoder::DSDSyncType sync) {
    return sync == DSDcc::DSDDecoder::DSDSyncDMRDataP ||
           sync == DSDcc::DSDDecoder::DSDSyncDMRDataMS ||
           sync == DSDcc::DSDDecoder::DSDSyncDMRVoiceP ||
           sync == DSDcc::DSDDecoder::DSDSyncDMRVoiceMS;
}
}

class DMRDecoderModule : public ModuleManager::Instance {
public:
    explicit DMRDecoderModule(std::string instanceName) : name(std::move(instanceName)) {
        configureDecoder();
        core::modComManager.registerInterface("external_decoder", "external_decoder.dmr",
                                              externalInterfaceHandler, this);
    }

    ~DMRDecoderModule() override {
        core::modComManager.unregisterInterface("external_decoder.dmr");
        detachExternal();
    }

    void postInit() override {}
    void enable() override { enabled = true; }
    void disable() override { if (!externalAttached.load()) enabled = false; }
    bool isEnabled() override { return enabled || externalAttached.load(); }

private:
    static void externalInterfaceHandler(int command, void* in, void* out, void* ctx) {
        auto* self = static_cast<DMRDecoderModule*>(ctx);
        switch (command) {
        case EXTERNAL_DECODER_GET_INFO:
            if (out) {
                auto* info = static_cast<ExternalDecoderInfo*>(out);
                info->id = "dmr";
                info->displayName = "DMR";
                info->inputSampleRate = INPUT_RATE;
                info->audioSampleRate = AUDIO_RATE;
                info->defaultBandwidth = CHANNEL_BANDWIDTH;
                info->minBandwidth = CHANNEL_BANDWIDTH;
                info->maxBandwidth = CHANNEL_BANDWIDTH;
                info->snapInterval = 12500.0;
                info->audioOutput = &self->audioOutput;
            }
            break;
        case EXTERNAL_DECODER_ATTACH:
            if (in) self->attachExternal(static_cast<ExternalDecoderAttachArgs*>(in)->iqInput);
            break;
        case EXTERNAL_DECODER_DETACH:
            self->detachExternal();
            break;
        case EXTERNAL_DECODER_DRAW_MENU:
            ImGui::PushID("embedded_dmr_decoder");
            drawMenu(self);
            ImGui::PopID();
            break;
        }
    }

    void configureDecoder() {
        decoder.setDecodeMode(DSDcc::DSDDecoder::DSDDecodeNone, true);
        decoder.setDecodeMode(DSDcc::DSDDecoder::DSDDecodeDMR, true);
        decoder.enableMbelib(true);
        decoder.enableCosineFiltering(true);
        decoder.setSymbolPLLLock(true);
        decoder.setUpsampling(6);
        decoder.setAudioGain(0.0f); // DSDcc automatic gain
        decoder.setQuiet();
    }

    void attachExternal(dsp::stream<dsp::complex_t>* input) {
        if (!input) return;
        externalAttached = false;
        audioOutput.stopWriter();
        inputSink.stop();
        audioOutput.clearWriteStop();
        previous = {};
        havePrevious = false;
        audioFill = 0;
        decodedAudio.clear();
        lockEvidence = 0;
        lockLoss = 0;
        candidateSync = false;
        synced = false;
        configureDecoder();
        if (inputSinkInitialized) {
            // Replace the stream registered by the previous Radio VFO.  Calling
            // init() again would append another input to dsp::block::inputs;
            // once that old VFO is deleted, a later stop would dereference the
            // stale stream pointer and terminate SDR++.
            inputSink.setInput(input);
        }
        else {
            inputSink.init(input, iqHandler, this);
            inputSinkInitialized = true;
        }
        externalAttached = true;
        enabled = true;
        inputSink.start();
    }

    void detachExternal() {
        if (!externalAttached.exchange(false)) return;
        audioOutput.stopWriter();
        inputSink.stop();
        audioFill = 0;
        decodedAudio.clear();
        lockEvidence = 0;
        lockLoss = 0;
        candidateSync = false;
        havePrevious = false;
        synced = false;
        slot1Voice = false;
        slot2Voice = false;
    }

    static void drawMenu(void* ctx) {
        auto* self = static_cast<DMRDecoderModule*>(ctx);
        const bool lock = self->synced.load();
        ImGui::TextUnformatted("DMR Tier II decoder");
        ImGui::Separator();
        ImGui::TextUnformatted("Sync:");
        ImGui::SameLine();
        if (lock) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "LOCKED");
        else if (self->candidateSync.load()) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "verifying");
        else ImGui::TextUnformatted("searching");
        ImGui::Text("Symbol quality: %d", self->symbolQuality.load());
        ImGui::Text("Color code: %s", self->colorCode.load() < 0 ? "--" : std::to_string(self->colorCode.load()).c_str());

        int selectedSlot = self->selectedSlot.load();
        const char* slots[] = { "Auto", "Slot 1", "Slot 2" };
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::Combo("##dmr_slot", &selectedSlot, slots, 3)) self->selectedSlot = selectedSlot;

        std::string s1;
        std::string s2;
        std::string frame;
        std::string dataHistory[5];
        std::string header1;
        std::string header2;
        std::string packet1;
        std::string packet2;
        {
            std::lock_guard<std::mutex> lockGuard(self->statusMutex);
            s1 = self->slot1Text;
            s2 = self->slot2Text;
            frame = self->frameText;
            for (int i = 0; i < 5; ++i) dataHistory[i] = self->dataHistory[i];
            header1 = self->slot1HeaderText;
            header2 = self->slot2HeaderText;
            packet1 = self->slot1PacketText;
            packet2 = self->slot2PacketText;
        }
        ImGui::Text("Slot 1: %s%s", self->slot1Voice.load() ? "VOICE " : "", s1.empty() ? "--" : s1.c_str());
        drawPrivacy("Privacy 1", self->slot1Privacy.load());
        ImGui::Text("Slot 2: %s%s", self->slot2Voice.load() ? "VOICE " : "", s2.empty() ? "--" : s2.c_str());
        drawPrivacy("Privacy 2", self->slot2Privacy.load());
        if (!frame.empty()) ImGui::TextWrapped("Frame: %s", frame.c_str());
        if (!header1.empty()) ImGui::TextWrapped("Slot 1 %s", header1.c_str());
        if (!packet1.empty()) ImGui::TextWrapped("Slot 1 %s", packet1.c_str());
        if (!header2.empty()) ImGui::TextWrapped("Slot 2 %s", header2.c_str());
        if (!packet2.empty()) ImGui::TextWrapped("Slot 2 %s", packet2.c_str());
        for (int i = 0; i < 5; ++i)
            if (!dataHistory[i].empty()) ImGui::TextWrapped("Recent data %d: %s", i + 1, dataHistory[i].c_str());
        ImGui::Spacing();
        ImGui::TextWrapped("Center the 12.5 kHz Radio VFO on a conventional DMR carrier. Encrypted calls cannot be decoded.");
    }

    static void drawPrivacy(const char* label, int privacy) {
        ImGui::TextUnformatted(label);
        ImGui::SameLine();
        if (privacy > 0) ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.2f, 1.0f), "Encrypted");
        else if (privacy == 0) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Clear");
        else ImGui::TextUnformatted("Unknown");
    }

    void queueAudio(const short* samples, int count) {
        if (!samples || count <= 0) return;
        // Bound latency if downstream audio was temporarily unable to consume.
        if (decodedAudio.size() > static_cast<size_t>(AUDIO_RATE)) decodedAudio.clear();
        for (int i = 0; i < count; ++i) {
            const float value = std::clamp(samples[i] / 32768.0f, -1.0f, 1.0f);
            decodedAudio.push_back(value);
        }
    }

    bool emitAudioSample() {
        // Always clock the output at 48 kHz. If no newly decoded AMBE sample is
        // available, write zero so Radio cannot replay its previous buffer.
        float value = 0.0f;
        if (!decodedAudio.empty()) {
            value = decodedAudio.front();
            decodedAudio.pop_front();
        }
        audioOutput.writeBuf[audioFill++] = {value, value};
        if (audioFill != AUDIO_BLOCK) return true;
        const bool accepted = audioOutput.swap(AUDIO_BLOCK);
        audioFill = 0;
        return accepted;
    }

    void pollDecoder() {
        // A valid DMR sync word is authoritative. DSDcc's timing PLL can stay
        // unlocked on weak direct-mode/mobile bursts even while consecutive
        // voice sync words decode correctly (as seen in recorded IQ tests).
        // The sustained-sync timer below still rejects isolated noise matches.
        const bool rawLock = isDMRSync(decoder.getSyncType());
        candidateSync = rawLock;
        if (rawLock) {
            lockEvidence = (std::min)(LOCK_CONFIRM_SAMPLES, lockEvidence + 1);
            lockLoss = 0;
            if (lockEvidence >= LOCK_CONFIRM_SAMPLES) synced = true;
        }
        else {
            lockEvidence = (std::max)(0, lockEvidence - 4);
            if (synced.load() && ++lockLoss >= LOCK_LOSS_SAMPLES) {
                synced = false;
                decodedAudio.clear();
            }
        }

        // Audio may become ready on any sample, but the human-facing state
        // only needs a 10 Hz refresh. Avoid locking/copying strings 48k times/s.
        if (++statusDivider >= 4800) {
            statusDivider = 0;
            symbolQuality = decoder.getSymbolSyncQuality();
            slot1Voice = synced.load() && decoder.getVoice1On();
            slot2Voice = synced.load() && decoder.getVoice2On();
            colorCode = synced.load() ? static_cast<int>(decoder.getDMRDecoder().getColorCode()) : -1;
            slot1Privacy = synced.load() ? decoder.getDMRDecoder().getSlot0Privacy() : -1;
            slot2Privacy = synced.load() ? decoder.getDMRDecoder().getSlot1Privacy() : -1;

            std::lock_guard<std::mutex> lock(statusMutex);
            if (synced.load()) {
                const char* text1 = decoder.getDMRDecoder().getSlot0Text();
                const char* text2 = decoder.getDMRDecoder().getSlot1Text();
                slot1Text = text1 ? text1 : "";
                slot2Text = text2 ? text2 : "";
                frameText = decoder.getFrameTypeText() ? decoder.getFrameTypeText() : "";
                const char* subtype = decoder.getFrameSubtypeText();
                if (subtype && *subtype) { frameText += " "; frameText += subtype; }
                const char* d1 = decoder.getDMRDecoder().getSlot0DataText();
                const char* d2 = decoder.getDMRDecoder().getSlot1DataText();
                slot1DataText = d1 ? d1 : "";
                slot2DataText = d2 ? d2 : "";
                for (int i = 0; i < 5; ++i) {
                    const char* item = decoder.getDMRDecoder().getDataHistory(i);
                    dataHistory[i] = item ? item : "";
                }
                const char* h1 = decoder.getDMRDecoder().getSlot0HeaderText();
                const char* h2 = decoder.getDMRDecoder().getSlot1HeaderText();
                const char* p1 = decoder.getDMRDecoder().getSlot0PacketText();
                const char* p2 = decoder.getDMRDecoder().getSlot1PacketText();
                slot1HeaderText = h1 ? h1 : "";
                slot2HeaderText = h2 ? h2 : "";
                slot1PacketText = p1 ? p1 : "";
                slot2PacketText = p2 ? p2 : "";
            }
            else {
                slot1Text.clear();
                slot2Text.clear();
                frameText.clear();
                slot1DataText.clear();
                slot2DataText.clear();
                for (auto& item : dataHistory) item.clear();
                slot1HeaderText.clear();
                slot2HeaderText.clear();
                slot1PacketText.clear();
                slot2PacketText.clear();
            }
        }

        int count1 = 0;
        int count2 = 0;
        short* audio1 = decoder.getAudio1(count1);
        short* audio2 = decoder.getAudio2(count2);
        const int selection = selectedSlot.load();
        const bool voice1 = decoder.getVoice1On();
        const bool voice2 = decoder.getVoice2On();
        const bool acceptAudio = synced.load();
        const bool clear1 = decoder.getDMRDecoder().getSlot0Privacy() != 1;
        const bool clear2 = decoder.getDMRDecoder().getSlot1Privacy() != 1;
        if (!clear1 || !clear2) decodedAudio.clear();
        if (acceptAudio && selection == 1 && voice1 && clear1) queueAudio(audio1, count1);
        else if (acceptAudio && selection == 2 && voice2 && clear2) queueAudio(audio2, count2);
        else if (acceptAudio && selection == 0 && count1 > 0 && voice1 && clear1) queueAudio(audio1, count1);
        else if (acceptAudio && selection == 0 && count2 > 0 && voice2 && clear2) queueAudio(audio2, count2);
        if (count1 > 0) decoder.resetAudio1();
        if (count2 > 0) decoder.resetAudio2();
    }

    static void iqHandler(dsp::complex_t* samples, int count, void* ctx) {
        auto* self = static_cast<DMRDecoderModule*>(ctx);
        for (int i = 0; i < count; ++i) {
            const auto current = samples[i];
            if (self->havePrevious) {
                const double cross = static_cast<double>(current.im) * self->previous.re -
                                     static_cast<double>(current.re) * self->previous.im;
                const double dot = static_cast<double>(current.re) * self->previous.re +
                                   static_cast<double>(current.im) * self->previous.im;
                const double phase = std::atan2(cross, dot);
                const short discriminator = static_cast<short>(std::clamp(phase * 24000.0, -32767.0, 32767.0));
                self->decoder.run(discriminator);
                self->pollDecoder();
                if (!self->emitAudioSample()) return;
            }
            self->previous = current;
            self->havePrevious = true;
        }
    }

    std::string name;
    bool enabled = true;
    std::atomic<bool> externalAttached{false};
    bool inputSinkInitialized = false;
    dsp::sink::Handler<dsp::complex_t> inputSink;
    dsp::stream<dsp::stereo_t> audioOutput;
    DSDcc::DSDDecoder decoder;
    dsp::complex_t previous{};
    bool havePrevious = false;
    int audioFill = 0;
    int statusDivider = 0;
    std::deque<float> decodedAudio;
    int lockEvidence = 0;
    int lockLoss = 0;

    std::atomic<bool> synced{false};
    std::atomic<bool> candidateSync{false};
    std::atomic<bool> slot1Voice{false};
    std::atomic<bool> slot2Voice{false};
    std::atomic<int> symbolQuality{0};
    std::atomic<int> colorCode{-1};
    std::atomic<int> slot1Privacy{-1};
    std::atomic<int> slot2Privacy{-1};
    std::atomic<int> selectedSlot{0};
    std::mutex statusMutex;
    std::string slot1Text;
    std::string slot2Text;
    std::string frameText;
    std::string slot1DataText;
    std::string slot2DataText;
    std::string dataHistory[5];
    std::string slot1HeaderText;
    std::string slot2HeaderText;
    std::string slot1PacketText;
    std::string slot2PacketText;
};

MOD_EXPORT void _INIT_() {}
MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new DMRDecoderModule(std::move(name));
}
MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<DMRDecoderModule*>(instance);
}
MOD_EXPORT void _END_() {}
