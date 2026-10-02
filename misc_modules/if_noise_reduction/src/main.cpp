#include <config.h>
#include <core.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <imgui.h>
#include <module.h>
#include <radio_interface.h>

#include <algorithm>
#include <string>

SDRPP_MOD_INFO{
    "if_noise_reduction",
    "Adjustable frequency-domain IF noise reduction for Radio",
    "Nick / OpenAI",
    0, 1, 0,
    1
};

ConfigManager config;

class IFNoiseReductionModule : public ModuleManager::Instance {
public:
    explicit IFNoiseReductionModule(std::string instanceName) : name(std::move(instanceName)) {
        config.acquire();
        strength = std::clamp(config.conf.value("strength", 45), 0, 200);
        profile = std::clamp(config.conf.value("profile", 3), 0, 6);
        config.release();
        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~IFNoiseReductionModule() override { gui::menu.removeEntry(name); }
    void postInit() override { apply(); }
    void enable() override { moduleEnabled = true; apply(); }
    void disable() override { moduleEnabled = false; setRadioEnabled(false); }
    bool isEnabled() override { return moduleEnabled; }

private:
    bool radioAvailable() const { return core::modComManager.interfaceExists("Radio"); }

    void setRadioEnabled(bool value) {
        if (radioAvailable()) {
            core::modComManager.callInterface("Radio", RADIO_IFACE_CMD_SET_IFNR_ENABLED, &value, nullptr);
        }
    }

    void apply() {
        if (!moduleEnabled || !radioAvailable()) return;
        core::modComManager.callInterface("Radio", RADIO_IFACE_CMD_SET_IFNR_STRENGTH, &strength, nullptr);
        core::modComManager.callInterface("Radio", RADIO_IFACE_CMD_SET_IFNR_PROFILE, &profile, nullptr);
        setRadioEnabled(true);
    }

    void save() {
        config.acquire();
        config.conf["strength"] = strength;
        config.conf["profile"] = profile;
        config.release(true);
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<IFNoiseReductionModule*>(ctx);
        const bool available = self->radioAvailable();
        int mode = -1;
        float bandwidth = 0.0f;
        if (available) {
            core::modComManager.callInterface("Radio", RADIO_IFACE_CMD_GET_MODE, nullptr, &mode);
            core::modComManager.callInterface("Radio", RADIO_IFACE_CMD_GET_BANDWIDTH, nullptr, &bandwidth);
        }
        const bool compatible = mode == RADIO_IFACE_MODE_NFM || mode == RADIO_IFACE_MODE_WFM ||
                                mode == RADIO_IFACE_MODE_AM || mode == RADIO_IFACE_MODE_DSB ||
                                mode == RADIO_IFACE_MODE_USB || mode == RADIO_IFACE_MODE_LSB;
        if (mode != self->lastMode) {
            self->lastMode = mode;
            if (compatible) self->apply();
        }

        ImGui::TextUnformatted("Adaptive Noise Reduction");
        ImGui::Separator();
        ImGui::Text("Radio connection: %s", available ? "Active" : "Waiting");
        ImGui::Text("Processing: %s", compatible ? "Active" : "Unavailable in this mode");
        if (compatible) {
            const char* path = (mode == RADIO_IFACE_MODE_NFM || mode == RADIO_IFACE_MODE_WFM)
                ? "FM IF"
                : (bandwidth <= 1000.0f ? "Adaptive tone" : "Adaptive speech");
            ImGui::Text("Algorithm: %s", path);
        }

        if (!available || !compatible) style::beginDisabled();
        static const char* profiles[] = {
            "Broadcast / Music", "Voice", "Narrow Band", "Shortwave Soft",
            "Shortwave Hard", "London Shortwave", "Hi-Fi"
        };
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::Combo("Profile##ifnr", &self->profile, profiles, 7)) {
            self->apply();
            self->save();
        }
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::SliderInt("Aggression##ifnr", &self->strength, 0, 200, "%d%%")) {
            self->apply();
            self->save();
        }
        if (!available || !compatible) style::endDisabled();

        if (!compatible) {
            ImGui::TextWrapped("Noise reduction is available for AM, DSB, LSB, USB, NFM and WFM.");
        }
        else {
            ImGui::TextWrapped("AM and SSB/DSB use speech-presence-aware spectral enhancement, switching automatically to tone processing at 1 kHz bandwidth or below. Values above 100% are intentionally aggressive. NFM/WFM use the FM IF reducer.");
        }
    }

    std::string name;
    bool moduleEnabled = true;
    int strength = 45;
    int profile = 3;
    int lastMode = -1;
};

MOD_EXPORT void _INIT_() {
    config.setPath(core::args["root"].s() + "/if_noise_reduction_config.json");
    config.load(json::object());
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new IFNoiseReductionModule(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<IFNoiseReductionModule*>(instance);
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
