#include <config.h>
#include <core.h>
#include <external_decoder.h>
#include <gui/gui.h>
#include <gui/widgets/image.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#include <DrmReceiver.h>
#include "dream_iq_input.h"
#include "dream_audio_output.h"
#include "mot_image.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <sstream>
#include <iomanip>
#include <utility>

SDRPP_MOD_INFO{
    /* Name:            */ "drm_decoder",
    /* Description:     */ "Digital Radio Mondiale (DRM30) decoder for SDR++",
    /* Author:          */ "Nick / OpenAI",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ -1
};

namespace {
constexpr double INPUT_SAMPLE_RATE = 48000.0;
constexpr double DEFAULT_BANDWIDTH = 10000.0;
}

ConfigManager config;

class DRMDecoderModule : public ModuleManager::Instance {
public:
    explicit DRMDecoderModule(std::string instanceName)
        : name(std::move(instanceName)), slideImage(SLIDE_WIDTH, SLIDE_HEIGHT) {
        audioOutput = std::make_unique<DreamAudioOutput>(decodedAudioStream);
        config.acquire();
        if (config.conf.contains(name) && config.conf[name].contains("audioGainDb")) {
            audioGainDb = config.conf[name]["audioGainDb"];
        }
        config.release();
        audioGainDb = std::clamp(audioGainDb, 0.0f, 24.0f);
        audioOutput->setGainDb(audioGainDb);
        createVFO();
        iqSink.init(vfo->output, iqHandler, this);
        iqSink.start();
        startDecoder();
        core::modComManager.registerInterface("external_decoder", "external_decoder.drm", externalInterfaceHandler, this);
    }

    ~DRMDecoderModule() override {
        core::modComManager.unregisterInterface("external_decoder.drm");
        stopDecoder();
        iqSink.stop();
        if (vfo) sigpath::vfoManager.deleteVFO(vfo);
    }

    void postInit() override {}

    void enable() override {
        if (enabled) { return; }
        if (externalAttached) { enabled = true; return; }
        createVFO();
        iqSink.setInput(vfo->output);
        iqSink.start();
        startDecoder();
        enabled = true;
    }

    void disable() override {
        if (externalAttached) {
            // Radio owns these streams while DRM mode is selected. Radio will
            // detach them as part of a safe mode switch.
            return;
        }
        if (!enabled) { return; }
        iqSink.stop();
        stopDecoder();
        sigpath::vfoManager.deleteVFO(vfo);
        vfo = nullptr;
        enabled = false;
    }

    bool isEnabled() override { return enabled || externalAttached; }

private:
    static void externalInterfaceHandler(int command, void* in, void* out, void* ctx) {
        auto* self = static_cast<DRMDecoderModule*>(ctx);
        switch (command) {
        case EXTERNAL_DECODER_GET_INFO:
            if (out) {
                auto* info = static_cast<ExternalDecoderInfo*>(out);
                info->id = "drm";
                info->displayName = "DRM";
                info->inputSampleRate = INPUT_SAMPLE_RATE;
                info->audioSampleRate = 48000.0;
                info->defaultBandwidth = DEFAULT_BANDWIDTH;
                info->minBandwidth = DEFAULT_BANDWIDTH;
                info->maxBandwidth = DEFAULT_BANDWIDTH;
                info->snapInterval = 1000.0;
                info->audioOutput = &self->decodedAudioStream;
            }
            break;
        case EXTERNAL_DECODER_ATTACH:
            if (in) self->attachExternal(static_cast<ExternalDecoderAttachArgs*>(in)->iqInput);
            break;
        case EXTERNAL_DECODER_DETACH:
            self->detachExternal();
            break;
        case EXTERNAL_DECODER_DRAW_MENU:
            ImGui::PushID("embedded_drm_decoder");
            self->drawingEmbeddedPanel = true;
            menuHandler(self);
            self->drawingEmbeddedPanel = false;
            ImGui::PopID();
            break;
        }
    }

    void attachExternal(dsp::stream<dsp::complex_t>* input) {
        if (!input) return;
        iqSink.stop();
        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
        audioOutput->setMuted(true);
        audioStatus.store(NOT_PRESENT);
        iqSink.setInput(input);
        startDecoder();
        iqSink.start();
        enabled = false;
        externalAttached = true;
    }

    void detachExternal() {
        if (!externalAttached) return;
        iqSink.stop();
        stopDecoder();
        externalAttached = false;
    }

    void createVFO() {
        const double visibleBandwidth = gui::waterfall.getBandwidth();
        const double offset = std::clamp(0.0, -visibleBandwidth / 2.0, visibleBandwidth / 2.0);
        vfo = sigpath::vfoManager.createVFO(
            name,
            ImGui::WaterfallVFO::REF_CENTER,
            offset,
            DEFAULT_BANDWIDTH,
            INPUT_SAMPLE_RATE,
            20000.0,
            20000.0,
            true);
        vfo->setSnapInterval(1000);
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<DRMDecoderModule*>(ctx);
        if (!self->enabled && !self->externalAttached) { return; }

        if (self->externalAttached && !self->drawingEmbeddedPanel) {
            ImGui::TextWrapped("DRM is controlled by the Radio module. Its status and controls are shown under DRM Decoder details there.");
            return;
        }

        ImGui::TextUnformatted("DRM30 Receiver");
        ImGui::TextUnformatted("Input: 48 kHz complex IQ");
        ImGui::TextUnformatted("Channel bandwidth: 10 kHz");
        ImGui::Spacing();

        std::string service;
        {
            std::lock_guard<std::mutex> lock(self->serviceMutex);
            service = self->audioService;
        }

        if (ImGui::BeginTable("##drm_status", 2,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp)) {
            statusRow("IQ stream", self->sampleCount.load() ? "Active" : "Waiting");
            statusRow("DRM signal", self->signalAcquired.load() ? "Acquired" : "Not acquired");
            statusRow("Time sync", statusText(self->timeSync.load()));
            statusRow("Frequency sync", statusText(self->frequencySync.load()));
            statusRow("FAC", statusText(self->facStatus.load()));
            statusRow("SDC", statusText(self->sdcStatus.load()));
            statusRow("MSC audio", statusText(self->audioStatus.load()));
            statusRow("Audio service", service.empty() ? "--" : service.c_str());
            ImGui::EndTable();
        }

        ImGui::Text("Input level: %.1f dBFS", self->inputLevelDbfs.load());
        ImGui::Text("IQ samples: %llu", static_cast<unsigned long long>(self->sampleCount.load()));
        ImGui::Text("Decoder queue drops: %llu",
            static_cast<unsigned long long>(self->dreamInput.dropped()));
        ImGui::Text("Decoded audio rate: %d Hz", self->audioOutput->rate());
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::SliderFloat("##drm_audio_gain", &self->audioGainDb, 0.0f, 24.0f,
                "Audio gain: +%.1f dB")) {
            self->audioOutput->setGainDb(self->audioGainDb);
            config.acquire();
            config.conf[self->name]["audioGainDb"] = self->audioGainDb;
            config.release(true);
        }
        self->drawServicePanel();
        self->drawSlideshowPanel();
        ImGui::Spacing();
        ImGui::TextWrapped("Dream DRM30 receiver core is running. A valid signal should progress through time sync, frequency sync, FAC and SDC.");
    }

    static const char* statusText(int status) {
        switch (status) {
        case RX_OK: return "OK";
        case CRC_ERROR: return "CRC error";
        case DATA_ERROR: return "Data error";
        default: return "Not locked";
        }
    }

    void drawServicePanel() {
        ServiceSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(serviceMutex);
            snapshot = serviceSnapshot;
        }
        if (snapshot.labels.empty()) return;

        ImGui::Separator();
        ImGui::TextUnformatted("Services");
        std::string choices;
        for (const auto& label : snapshot.labels) {
            choices += label;
            choices.push_back('\0');
        }
        int selected = snapshot.selectedListIndex;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::Combo("##drm_audio_service", &selected, choices.c_str()) &&
            selected >= 0 && selected < static_cast<int>(snapshot.shortIds.size())) {
            requestedAudioService.store(snapshot.shortIds[selected]);
        }
        ImGui::Text("Service ID: %s", snapshot.serviceId.c_str());
        ImGui::Text("Language: %s", snapshot.language.c_str());
        ImGui::Text("Codec: %s", snapshot.codec.c_str());
        ImGui::Text("Mode: %s", snapshot.mode.c_str());
        if (!snapshot.text.empty()) {
            ImGui::TextWrapped("Text: %s", snapshot.text.c_str());
        }
        ImGui::Text("Slideshow/data: %s", snapshot.hasAssociatedData ? "Advertised" : "Not advertised");
    }

    void drawSlideshowPanel() {
        ServiceSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(serviceMutex);
            snapshot = serviceSnapshot;
        }
        if (!snapshot.hasSlideshow) return;

        ImGui::Separator();
        ImGui::TextUnformatted("MOT Slideshow");
        updateSlideImage();
        if (!slideReady) {
            ImGui::TextWrapped("Waiting for a complete slideshow image...");
            return;
        }
        slideImage.draw(ImVec2(ImGui::GetContentRegionAvail().x, 0.0f));
        std::lock_guard<std::mutex> lock(slideMutex);
        if (!displayedSlideName.empty()) ImGui::TextWrapped("%s", displayedSlideName.c_str());
        ImGui::Text("%d x %d%s%s", displayedImageWidth, displayedImageHeight,
            displayedSlideMime.empty() ? "" : "  ", displayedSlideMime.c_str());
    }

    void updateSlideImage() {
        std::vector<std::uint8_t> encoded;
        std::string name;
        std::string mime;
        {
            std::lock_guard<std::mutex> lock(slideMutex);
            if (pendingSlideGeneration == displayedSlideGeneration) return;
            encoded = pendingSlide;
            name = pendingSlideName;
            mime = pendingSlideMime;
        }

        std::vector<std::uint8_t> canvas;
        int width = 0;
        int height = 0;
        if (!decodeMotImage(encoded, canvas, SLIDE_WIDTH, SLIDE_HEIGHT, width, height)) return;
        std::memcpy(slideImage.buffer, canvas.data(), canvas.size());
        slideImage.swap();
        {
            std::lock_guard<std::mutex> lock(slideMutex);
            displayedSlideGeneration = pendingSlideGeneration;
            displayedSlideName = std::move(name);
            displayedSlideMime = std::move(mime);
            displayedImageWidth = width;
            displayedImageHeight = height;
        }
        slideReady = true;
    }

    struct ServiceSnapshot {
        std::vector<std::string> labels;
        std::vector<int> shortIds;
        int selectedListIndex = 0;
        std::string serviceId = "--";
        std::string language = "--";
        std::string codec = "--";
        std::string mode = "--";
        std::string text;
        bool hasAssociatedData = false;
        bool hasSlideshow = false;
    };

    static const char* codecName(CAudioParam::EAudCod codec) {
        switch (codec) {
        case CAudioParam::AC_AAC: return "AAC";
        case CAudioParam::AC_OPUS: return "Opus";
        case CAudioParam::AC_xHE_AAC: return "xHE-AAC";
        default: return "Unknown";
        }
    }

    static const char* modeName(CAudioParam::EAudioMode mode) {
        switch (mode) {
        case CAudioParam::AM_MONO: return "Mono";
        case CAudioParam::AM_P_STEREO: return "Parametric stereo";
        case CAudioParam::AM_STEREO: return "Stereo";
        default: return "Reserved";
        }
    }

    void startDecoder() {
        if (decoderRunning.exchange(true)) return;
        if (decoderThread.joinable()) decoderThread.join();
        // Dream may retain its last concealment block until a new valid MSC
        // frame arrives. Never expose that block during a mode handoff.
        audioOutput->setMuted(true);
        audioStatus.store(NOT_PRESENT);
        // Dream normally initializes its sound interface only after service
        // parameters are available. Start our host bridge immediately so the
        // SDR++ sink receives silence throughout DRM acquisition as well.
        audioOutput->Init(static_cast<int>(INPUT_SAMPLE_RATE), 0, true);
        dreamInput.Init(48000, 0, true);
        receiver = std::make_unique<CDRMReceiver>();
        receiver->SetInputInterface(&dreamInput);
        receiver->SetOutputInterface(audioOutput.get());
        receiver->GetReceiveData()->SetInChanSel(CS_IQ_POS_ZERO);
        receiver->InitReceiverMode();
        decoderThread = std::thread([this] {
            while (decoderRunning.load()) {
                try {
                    receiver->process();
                    updateDecoderStatus();
                }
                catch (...) {
                    decoderRunning.store(false);
                }
            }
        });
    }

    void stopDecoder() {
        decoderRunning.store(false);
        dreamInput.Close();
        audioOutput->Close();
        if (decoderThread.joinable()) decoderThread.join();
        receiver.reset();
    }

    void updateDecoderStatus() {
        auto* parameters = receiver->GetParameters();
        parameters->Lock();
        const int requested = requestedAudioService.exchange(-1);
        if (requested >= 0 && requested < static_cast<int>(parameters->Service.size())) {
            parameters->SetCurSelAudioService(requested);
        }
        const bool acquired = parameters->GetAcquiState() == AS_WITH_SIGNAL;
        signalAcquired.store(acquired);
        timeSync.store(acquired ? parameters->ReceiveStatus.TSync.GetStatus() : NOT_PRESENT);
        frequencySync.store(acquired ? parameters->ReceiveStatus.FSync.GetStatus() : NOT_PRESENT);
        facStatus.store(acquired ? parameters->ReceiveStatus.FAC.GetStatus() : NOT_PRESENT);
        sdcStatus.store(acquired ? parameters->ReceiveStatus.SDC.GetStatus() : NOT_PRESENT);
        const auto shortAudio = parameters->ReceiveStatus.SLAudio.GetStatus();
        const auto longAudio = parameters->ReceiveStatus.LLAudio.GetStatus();
        const int currentAudioStatus = acquired ? (shortAudio == RX_OK ? shortAudio : longAudio) : NOT_PRESENT;
        audioStatus.store(currentAudioStatus);
        audioOutput->setMuted(currentAudioStatus != RX_OK);
        std::string newService;
        ServiceSnapshot newSnapshot;
        if (acquired) {
            const int serviceId = parameters->GetCurSelAudioService();
            int slideshowService = -1;
            for (int i = 0; i < static_cast<int>(parameters->Service.size()); ++i) {
                const auto& candidate = parameters->Service[i];
                if (candidate.IsActive() && candidate.DataParam.iStreamID != STREAM_ID_NOT_USED &&
                    candidate.DataParam.eAppDomain == CDataParam::AD_DAB_SPEC_APP &&
                    candidate.DataParam.iUserAppIdent == DAB_AT_MOTSLIDESHOW) {
                    newSnapshot.hasSlideshow = true;
                    if (slideshowService < 0) slideshowService = i;
                }
                if (!candidate.IsActive() || candidate.AudioParam.iStreamID == STREAM_ID_NOT_USED) continue;
                newSnapshot.shortIds.push_back(i);
                newSnapshot.labels.push_back(candidate.strLabel.empty()
                    ? "Audio service " + std::to_string(i) : candidate.strLabel);
                if (i == serviceId) newSnapshot.selectedListIndex = static_cast<int>(newSnapshot.labels.size()) - 1;
            }
            if (serviceId >= 0 && serviceId < static_cast<int>(parameters->Service.size()) &&
                parameters->Service[serviceId].IsActive()) {
                const auto& service = parameters->Service[serviceId];
                newService = service.strLabel;
                if (newService.empty()) newService = "Audio service " + std::to_string(serviceId);
                std::ostringstream id;
                id << "0x" << std::uppercase << std::hex << std::setw(6) << std::setfill('0') << service.iServiceID;
                newSnapshot.serviceId = id.str();
                newSnapshot.language = service.strLanguageCode.empty()
                    ? (service.iLanguage ? "Code " + std::to_string(service.iLanguage) : "--")
                    : service.strLanguageCode;
                newSnapshot.codec = codecName(service.AudioParam.eAudioCoding);
                newSnapshot.mode = modeName(service.AudioParam.eAudioMode);
                newSnapshot.text = service.AudioParam.strTextMessage;
                newSnapshot.hasAssociatedData = service.DataParam.iStreamID != STREAM_ID_NOT_USED;
            }
            if (slideshowService >= 0 && parameters->GetCurSelDataService() != slideshowService) {
                parameters->SetCurSelDataService(slideshowService);
            }
        }
        parameters->Unlock();
        pollMotSlideshow();
        {
            std::lock_guard<std::mutex> lock(serviceMutex);
            audioService = std::move(newService);
            serviceSnapshot = std::move(newSnapshot);
        }
    }

    void pollMotSlideshow() {
        if (!receiver) return;
        CMOTObject object;
        auto* decoder = receiver->GetDataDecoder();
        if (!decoder || !decoder->GetMOTObject(object, CDataDecoder::AT_MOTSLIDESHOW) ||
            object.Body.vecData.Size() <= 0) return;

        std::vector<std::uint8_t> encoded(static_cast<std::size_t>(object.Body.vecData.Size()));
        for (int i = 0; i < object.Body.vecData.Size(); ++i) {
            encoded[static_cast<std::size_t>(i)] = object.Body.vecData[i];
        }
        std::lock_guard<std::mutex> lock(slideMutex);
        pendingSlide = std::move(encoded);
        pendingSlideName = object.strName;
        pendingSlideMime = object.strMimeType;
        ++pendingSlideGeneration;
    }

    static void statusRow(const char* label, const char* value) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(value);
    }

    static void iqHandler(dsp::complex_t* samples, int count, void* ctx) {
        auto* self = static_cast<DRMDecoderModule*>(ctx);
        if (count <= 0) { return; }

        double power = 0.0;
        for (int i = 0; i < count; ++i) {
            power += static_cast<double>(samples[i].re) * samples[i].re;
            power += static_cast<double>(samples[i].im) * samples[i].im;
        }
        power /= static_cast<double>(count);
        const float level = static_cast<float>(10.0 * std::log10((std::max)(power, 1.0e-20)));
        self->inputLevelDbfs.store(level);
        self->sampleCount.fetch_add(static_cast<std::uint64_t>(count));
        self->dreamInput.push(samples, count);
    }

    std::string name;
    VFOManager::VFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> iqSink;
    std::atomic<float> inputLevelDbfs{-200.0f};
    std::atomic<std::uint64_t> sampleCount{0};
    DreamIqInput dreamInput;
    dsp::stream<dsp::stereo_t> decodedAudioStream;
    std::unique_ptr<DreamAudioOutput> audioOutput;
    std::unique_ptr<CDRMReceiver> receiver;
    std::thread decoderThread;
    std::atomic<bool> decoderRunning{false};
    std::atomic<int> timeSync{NOT_PRESENT};
    std::atomic<int> frequencySync{NOT_PRESENT};
    std::atomic<int> facStatus{NOT_PRESENT};
    std::atomic<int> sdcStatus{NOT_PRESENT};
    std::atomic<int> audioStatus{NOT_PRESENT};
    std::atomic<bool> signalAcquired{false};
    std::mutex serviceMutex;
    std::string audioService;
    ServiceSnapshot serviceSnapshot;
    std::atomic<int> requestedAudioService{-1};
    float audioGainDb = 12.0f;
    static constexpr int SLIDE_WIDTH = 640;
    static constexpr int SLIDE_HEIGHT = 480;
    ImGui::ImageDisplay slideImage;
    std::mutex slideMutex;
    std::vector<std::uint8_t> pendingSlide;
    std::string pendingSlideName;
    std::string pendingSlideMime;
    std::uint64_t pendingSlideGeneration = 0;
    std::uint64_t displayedSlideGeneration = 0;
    std::string displayedSlideName;
    std::string displayedSlideMime;
    int displayedImageWidth = 0;
    int displayedImageHeight = 0;
    bool slideReady = false;
    bool enabled = true;
    bool externalAttached = false;
    bool drawingEmbeddedPanel = false;
};

MOD_EXPORT void _INIT_() {
    json defaults = json({});
    config.setPath(core::args["root"].s() + "/drm_decoder_config.json");
    config.load(defaults);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new DRMDecoderModule(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<DRMDecoderModule*>(instance);
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
