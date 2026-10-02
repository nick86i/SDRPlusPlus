#pragma once

#include "../demod.h"
#include <core.h>
#include <external_decoder.h>
#include <string>

namespace demod {
    class External : public Demodulator {
    public:
        explicit External(std::string interfaceName) : interfaceName(std::move(interfaceName)) {
            available = core::modComManager.callInterface(this->interfaceName,
                EXTERNAL_DECODER_GET_INFO, nullptr, &info);
        }

        void init(std::string, ConfigManager*, dsp::stream<dsp::complex_t>* input,
                  double, double) override { iqInput = input; }
        void start() override {
            if (!available || running) return;
            ExternalDecoderAttachArgs args{iqInput};
            running = core::modComManager.callInterface(interfaceName,
                EXTERNAL_DECODER_ATTACH, &args, nullptr);
        }
        void stop() override {
            if (!running) return;
            core::modComManager.callInterface(interfaceName, EXTERNAL_DECODER_DETACH, nullptr, nullptr);
            running = false;
        }
        void showMenu() override {
            const std::string label = std::string(info.displayName ? info.displayName : "External") + " Decoder details";
            if (ImGui::CollapsingHeader(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
                core::modComManager.callInterface(interfaceName, EXTERNAL_DECODER_DRAW_MENU, nullptr, nullptr);
        }
        void setBandwidth(double) override {}
        void setInput(dsp::stream<dsp::complex_t>* input) override {
            iqInput = input;
            if (running) {
                ExternalDecoderAttachArgs args{iqInput};
                core::modComManager.callInterface(interfaceName, EXTERNAL_DECODER_ATTACH, &args, nullptr);
            }
        }
        void AFSampRateChanged(double) override {}
        const char* getName() override { return info.displayName ? info.displayName : "External"; }
        double getIFSampleRate() override { return info.inputSampleRate; }
        double getAFSampleRate() override { return info.audioSampleRate; }
        double getDefaultBandwidth() override { return info.defaultBandwidth; }
        double getMinBandwidth() override { return info.minBandwidth; }
        double getMaxBandwidth() override { return info.maxBandwidth; }
        bool getBandwidthLocked() override { return info.minBandwidth == info.maxBandwidth; }
        double getDefaultSnapInterval() override { return info.snapInterval; }
        int getVFOReference() override { return ImGui::WaterfallVFO::REF_CENTER; }
        bool getDeempAllowed() override { return false; }
        bool getPostProcEnabled() override { return true; }
        int getDefaultDeemphasisMode() override { return DEEMP_MODE_NONE; }
        bool getFMIFNRAllowed() override { return false; }
        bool getNBAllowed() override { return false; }
        dsp::stream<dsp::stereo_t>* getOutput() override { return info.audioOutput; }
        bool isAvailable() const { return available && info.audioOutput; }

    private:
        std::string interfaceName;
        ExternalDecoderInfo info{};
        dsp::stream<dsp::complex_t>* iqInput = nullptr;
        bool available = false;
        bool running = false;
    };
}
