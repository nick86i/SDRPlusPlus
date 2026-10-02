#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <gui/smgui.h>
#include <iio.h>
#include <ad9361.h>
#include <utils/optionlist.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <regex>
#ifdef _WIN32
#include <windows.h>
#endif

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "plutosdr_source",
    /* Description:     */ "PlutoSDR Tezuka source module for SDR++",
    /* Author:          */ "Ryzerth/F5OEO",
    /* Version:         */ 0, 2, 2,
    /* Max instances    */ 1
};

ConfigManager config;

const std::vector<const char*> deviceWhiteList = {
    "PlutoSDR",
    "ANTSDR",
    "LibreSDR",
    "Pluto+",
    "ad9361",
    "FISH"
};

class PlutoSDRSourceModule : public ModuleManager::Instance {
public:
    PlutoSDRSourceModule(std::string name) {
        this->name = name;

        // Define valid samplerates
        for (int sr = 1000000; sr <= 61440000; sr += 500000) {
            samplerates.define(sr, getBandwdithScaled(sr), sr);
        }
        samplerates.define(61440000, getBandwdithScaled(61440000.0), 61440000.0);

        // Define valid bandwidths
        bandwidths.define(0, "Auto", 0);
        for (int bw = 1000000.0; bw <= 52000000; bw += 500000) {
            bandwidths.define(bw, getBandwdithScaled(bw), bw);
        }

        // Define gain modes
        gainModes.define("manual", "Manual", "manual");
        gainModes.define("fast_attack", "Fast Attack", "fast_attack");
        gainModes.define("slow_attack", "Slow Attack", "slow_attack");
        gainModes.define("hybrid", "Hybrid", "hybrid");

        rfInputSelect.define("rx1", "RX1", "rx1");
        rfInputSelect.define("rx2", "RX2", "rx2");

        iqModeSelect.define("cs16", "CS16", "cs16");
        iqModeSelect.define("cs8", "CS8", "cs8");

        // Enumerate devices
        refresh();

        // Select device
        config.acquire();
        devDesc = config.conf["device"];
        config.release();
        select(devDesc);

        // Register source
        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("PlutoSDR", &handler);
        sigpath::sourceManager.registerSourceMenuExtensions("PlutoSDR", menuStatusHandler, menuAdvancedHandler);
    }

    ~PlutoSDRSourceModule() {
        stop(this);
        shuttingDown.store(true);
        finishPreconnect();
        destroyContext();
        sigpath::sourceManager.unregisterSource("PlutoSDR");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = true;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    std::string getBandwdithScaled(double bw) {
        char buf[1024];
        if (bw >= 1000000.0) {
            sprintf(buf, "%.1lfMHz", bw / 1000000.0);
        }
        else if (bw >= 1000.0) {
            sprintf(buf, "%.1lfKHz", bw / 1000.0);
        }
        else {
            sprintf(buf, "%.1lfHz", bw);
        }
        return std::string(buf);
    }

    void refresh() {
        // Clear device list
        devices.clear();

        // Create scan context
        // Restrict discovery to the transports supported by Pluto-family devices.
        // Explicitly including IP is important for Pluto+ units connected by Ethernet.
        iio_scan_context* sctx = iio_create_scan_context("usb:ip", 0);
        if (!sctx) {
            flog::error("Failed get scan context");
            return;
        }

        // Create parsing regexes
        std::regex backendRgx(".+(?=:)", std::regex::ECMAScript);
        std::regex modelRgx("\\(.+(?=\\),)", std::regex::ECMAScript);
        std::regex serialRgx("serial=[0-9A-Za-z]+", std::regex::ECMAScript);

        // Enumerate devices
        iio_context_info** ctxInfoList;
        ssize_t count = iio_scan_context_get_info_list(sctx, &ctxInfoList);
        if (count < 0) {
            flog::error("Failed to enumerate contexts");
            return;
        }
        for (ssize_t i = 0; i < count; i++) {
            iio_context_info* info = ctxInfoList[i];
            std::string desc = iio_context_info_get_description(info);
            std::string duri = iio_context_info_get_uri(info);

            // If the device is not a plutosdr, don't include it
            bool isPluto = false;
            for (const auto type : deviceWhiteList) {
                if (desc.find(type) != std::string::npos) {
                    isPluto = true;
                    break;
                }
            }
            if (!isPluto) {
                flog::warn("Ignored IIO device: [{}] {}", duri, desc);
                continue;
            }

            // Extract the backend
            std::string backend = "unknown";
            std::smatch backendMatch;
            if (std::regex_search(duri, backendMatch, backendRgx)) {
                backend = backendMatch[0];
            }

            // Extract the model
            std::string model = "Unknown";
            std::smatch modelMatch;
            if (std::regex_search(desc, modelMatch, modelRgx)) {
                model = modelMatch[0];
                int parenthPos = model.find('(');
                if (parenthPos != std::string::npos) {
                    model = model.substr(parenthPos+1);
                }
            }

            // Extract the serial
            std::string serial = "unknown";
            std::smatch serialMatch;
            if (std::regex_search(desc, serialMatch, serialRgx)) {
                serial = serialMatch[0].str().substr(7);
            }

            // Construct the device name
            std::string devName = '(' + backend + ") " + model + " [" + serial + ']';

            // Skip duplicate devices
            if (devices.keyExists(desc) || devices.nameExists(devName) || devices.valueExists(duri)) { continue; }

            // Save device
            devices.define(desc, devName, duri);
        }
        iio_context_info_list_free(ctxInfoList);
        
        // Destroy scan context
        iio_scan_context_destroy(sctx);

#ifdef __ANDROID__
        // On Android, a default IP entry must be made (TODO: This is not ideal since the IP cannot be changed)
        const char* androidURI = "ip:192.168.2.1";
        const char* androidName = "Default (192.168.2.1)";
        devices.define(androidName, androidName, androidURI);
#endif
    }

    void select(const std::string& desc) {
        // If no device is available, give up
        if (devices.empty()) {
            devDesc.clear();
            return;
        }

        // If the device is not available, select the first one
        if (!devices.keyExists(desc)) {
            select(devices.key(0));
            return;
        }

        // The background connector owns a copy of the current URI. Finish it
        // before changing that string or replacing its context.
        finishPreconnect();
        destroyContext();

        // Update URI
        devDesc = desc;
        uri = devices.value(devices.keyId(desc));

        // TODO: Enumerate capabilities

        // Load defaults
        samplerate = 4000000;
        bandwidth = 0;
        gmId = 0;
        gain = -1.0f;
        rfId = 0;
        iqModeId = 0;

        // Load device config
        config.acquire();
        if (config.conf["devices"][devDesc].contains("samplerate")) {
            samplerate = config.conf["devices"][devDesc]["samplerate"];
        }
        if (config.conf["devices"][devDesc].contains("bandwidth")) {
            bandwidth = config.conf["devices"][devDesc]["bandwidth"];
        }
        if (config.conf["devices"][devDesc].contains("gainMode")) {
            // Select given gain mode or default if invalid
            std::string gm = config.conf["devices"][devDesc]["gainMode"];
            if (gainModes.keyExists(gm)) {
                gmId = gainModes.keyId(gm);
            }
            else {
                gmId = 0;
            }
        }
        if (config.conf["devices"][devDesc].contains("gain")) {
            gain = config.conf["devices"][devDesc]["gain"];
            gain = std::clamp<int>(gain, -1.0f, 73.0f);
        }
        if (config.conf["devices"][devDesc].contains("rfinput")) {
            const std::string rfInput = config.conf["devices"][devDesc]["rfinput"];
            if (rfInputSelect.keyExists(rfInput)) {
                rfId = rfInputSelect.keyId(rfInput);
            }
        }
        if (config.conf["devices"][devDesc].contains("iqmode")) {
            const std::string iqMode = config.conf["devices"][devDesc]["iqmode"];
            if (iqModeSelect.keyExists(iqMode)) {
                iqModeId = iqModeSelect.keyId(iqMode);
            }
        }
        config.release();

        // Update samplerate ID
        if (samplerates.keyExists(samplerate)) {
            srId = samplerates.keyId(samplerate);
        }
        else {
            srId = 0;
            samplerate = samplerates.value(srId);
        }

        // Update bandwidth ID
        if (bandwidths.keyExists(bandwidth)) {
            bwId = bandwidths.keyId(bandwidth);
        }
        else {
            bwId = 0;
            bandwidth = bandwidths.value(bwId);
        }

        beginPreconnect();
    }

    void finishPreconnect() {
        if (preconnectThread.joinable()) {
            preconnectThread.join();
        }
    }

    void destroyContext() {
        if (ctx) {
            iio_context_destroy(ctx);
            ctx = NULL;
        }
        phy = NULL;
        dev = NULL;
        rxLO = NULL;
        rxChan = NULL;
    }

    void beginPreconnect() {
        finishPreconnect();
        destroyContext();
        if (uri.empty() || shuttingDown.load()) { return; }

        const std::string targetUri = uri;
        preconnectReady.store(false);
        preconnectThread = std::thread([this, targetUri]() {
            const auto began = std::chrono::steady_clock::now();
            iio_context* opened = iio_create_context_from_uri(targetUri.c_str());
            const uint64_t elapsed = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - began).count());
            preconnectUs.store(elapsed);

            if (shuttingDown.load() || targetUri != uri) {
                if (opened) { iio_context_destroy(opened); }
                return;
            }
            ctx = opened;
            preconnectReady.store(opened != NULL);
            if (!opened) {
                flog::warn("PlutoSDRSourceModule '{}': Background connection failed ({})", name, targetUri);
            }
        });
    }

    static void menuSelected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        core::setInputSampleRate(_this->samplerate);
        flog::info("PlutoSDRSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        flog::info("PlutoSDRSourceModule '{0}': Menu Deselect!", _this->name);
    }

    bool applyManualGain() {
        const double requestedGain = gain;
        const ssize_t writeResult = iio_channel_attr_write_double(rxChan, "hardwaregain", requestedGain);
        if (writeResult < 0) {
            flog::error("PlutoSDRSourceModule '{0}': Failed to set RX gain to {1} dB ({2})",
                        name, requestedGain, writeResult);
            return false;
        }

        double appliedGain = requestedGain;
        const ssize_t readResult = iio_channel_attr_read_double(rxChan, "hardwaregain", &appliedGain);
        if (readResult < 0) {
            flog::warn("PlutoSDRSourceModule '{0}': Could not read back RX gain ({1})", name, readResult);
            return false;
        }
        if (std::abs(appliedGain - requestedGain) > 1.0) {
            flog::warn("PlutoSDRSourceModule '{0}': RX gain verification mismatch: requested {1} dB, read {2} dB",
                       name, requestedGain, appliedGain);
            return false;
        }

        gain = static_cast<float>(appliedGain);
        return true;
    }

    bool applyGainSettings() {
        const std::string& gainMode = gainModes.value(gmId);
        const ssize_t modeResult = iio_channel_attr_write(rxChan, "gain_control_mode", gainMode.c_str());
        if (modeResult < 0) {
            flog::error("PlutoSDRSourceModule '{0}': Failed to set gain mode to {1} ({2})",
                        name, gainMode, modeResult);
            return false;
        }

        // The AD9361 accepts a fixed hardware gain only after manual mode is
        // fully active. Networked devices can expose a short transition delay,
        // so verify the mode and the gain rather than trusting the first write.
        if (gainMode == "manual") {
            bool recoveryTransitionAttempted = false;
            for (int attempt = 0; attempt < 6; ++attempt) {
                char appliedMode[32] = {};
                const ssize_t modeReadResult = iio_channel_attr_read(
                    rxChan, "gain_control_mode", appliedMode, sizeof(appliedMode));
                if (modeReadResult >= 0 && std::string(appliedMode) == "manual" && applyManualGain()) {
                    return true;
                }

                // Some Pluto+/Tezuka firmware states report "manual" while
                // rejecting hardwaregain writes with EOPNOTSUPP. A real AGC to
                // Manual transition resets that stale internal state—the same
                // recovery previously achieved by changing the UI mode twice.
                if (!recoveryTransitionAttempted) {
                    recoveryTransitionAttempted = true;
                    const ssize_t agcResult = iio_channel_attr_write(
                        rxChan, "gain_control_mode", "fast_attack");
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    const ssize_t manualResult = iio_channel_attr_write(
                        rxChan, "gain_control_mode", "manual");
                    if (agcResult < 0 || manualResult < 0) {
                        flog::warn("PlutoSDRSourceModule '{0}': Gain-mode recovery transition failed ({1}, {2})",
                                   name, agcResult, manualResult);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            flog::error("PlutoSDRSourceModule '{0}': Manual gain did not become active after retries", name);
            return false;
        }
        return true;
    }

    static void start(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (_this->running) { return; }

        using StartClock = std::chrono::steady_clock;
        const auto startBegan = StartClock::now();
        auto phaseUs = [](StartClock::time_point began, StartClock::time_point ended) {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(ended - began).count());
        };
        _this->startContextUs = 0;
        _this->startDeviceUs = 0;
        _this->startSetupUs = 0;
        _this->startGainUs = 0;
        _this->startTotalUs = 0;
        _this->lastTuneUs = 0;

        // If no device is selected, give up
        if (_this->devDesc.empty() || _this->uri.empty()) { return; }

        // Reuse the context opened in the background after device selection.
        // If Play is pressed before it finishes, wait only for the remainder.
        _this->finishPreconnect();
        if (_this->ctx == NULL) {
            _this->ctx = iio_create_context_from_uri(_this->uri.c_str());
        }
        const auto contextDone = StartClock::now();
        _this->startContextUs.store(phaseUs(startBegan, contextDone));
        if (_this->ctx == NULL) {
            flog::error("Could not open pluto ({})", _this->uri);
            return;
        }

        // Get phy and device handle
        _this->phy = iio_context_find_device(_this->ctx, "ad9361-phy");
        if (_this->phy == NULL) {
            flog::error("Could not connect to pluto phy");
            _this->destroyContext();
            return;
        }
        _this->dev = iio_context_find_device(_this->ctx, "cf-ad9361-lpc");
        if (_this->dev == NULL) {
            flog::error("Could not connect to pluto dev");
            _this->destroyContext();
            return;
        }

        // Select RX1/RX2 using the same AD936x handling as F5OEO's Tezuka module.
        long long twoRxTwoTxMode = 0;
        iio_device_debug_attr_read_longlong(_this->phy, "adi,2rx-2tx-mode-enable", &twoRxTwoTxMode);
        if (twoRxTwoTxMode == 1) {
            _this->rxChan = iio_device_find_channel(_this->phy, _this->rfId == 0 ? "voltage0" : "voltage1", false);
        }
        else {
            uint32_t value = 0;
            iio_device_reg_read(_this->phy, 0x00000003, &value);
            value = (value & 0x3F) | ((_this->rfId + 1) << 6);
            iio_device_reg_write(_this->phy, 0x00000003, value);
            iio_device_debug_attr_write_longlong(_this->phy, "adi,1rx-1tx-mode-use-rx-num", _this->rfId + 1);
            _this->rxChan = iio_device_find_channel(_this->phy, "voltage0", false);
        }
        _this->rxLO = iio_device_find_channel(_this->phy, "altvoltage0", true);
        if (!_this->rxChan || !_this->rxLO) {
            flog::error("Could not select Pluto+ RX{}", _this->rfId + 1);
            _this->destroyContext();
            return;
        }
        const auto deviceDone = StartClock::now();
        _this->startDeviceUs.store(phaseUs(contextDone, deviceDone));

        // Enable RX LO and disable TX
        iio_channel_attr_write_bool(iio_device_find_channel(_this->phy, "altvoltage1", true), "powerdown", true);
        iio_channel_attr_write_bool(_this->rxLO, "powerdown", false);

        // Configure RX channel
        iio_channel_attr_write(_this->rxChan, "rf_port_select", "A_BALANCED");
        iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(_this->freq));                              // Freq
        iio_channel_attr_write_longlong(_this->rxChan, "sampling_frequency", round(_this->samplerate));             // Sample rate
        _this->setBandwidth(_this->bandwidth);
        const auto setupDone = StartClock::now();
        _this->startSetupUs.store(phaseUs(deviceDone, setupDone));
        _this->applyGainSettings();                                                                                  // Apply gain last; verify manual mode
        const auto gainDone = StartClock::now();
        _this->startGainUs.store(phaseUs(setupDone, gainDone));
        
        // Start worker thread
        _this->running = true;
        _this->workerThread = std::thread(worker, _this);
        _this->startTotalUs.store(phaseUs(startBegan, StartClock::now()));
        flog::info("PlutoSDRSourceModule '{0}': Start!", _this->name);
    }

    static void stop(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (!_this->running) { return; }

        // Stop worker thread
        _this->running = false;
        _this->stream.stopWriter();
        _this->workerThread.join();
        _this->stream.clearWriteStop();

        flog::info("PlutoSDRSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        _this->freq = freq;
        if (_this->running) {
            // Tune device
            const auto tuneBegan = std::chrono::steady_clock::now();
            iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(freq));
            _this->lastTuneUs.store(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - tuneBegan).count()));
        }
        flog::info("PlutoSDRSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;

        SmGui::BeginTable(CONCAT("PlutoGainTable##_", _this->name), 2,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        SmGui::TableNextRow();
        SmGui::TableSetColumnIndex(0);
        SmGui::Text("Gain");
        SmGui::SameLine();
        if (_this->gmId) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_pluto_gain__", _this->name), &_this->gain, -1.0f, 73.0f, 1.0f, SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
            if (_this->running) {
                // Reassert manual mode as well as the gain. Some AD9361/IIO
                // stream transitions can leave hardware in AGC while the UI
                // still has Manual selected.
                _this->applyGainSettings();
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gain"] = _this->gain;
                config.release(true);
            }
        }
        if (_this->gmId) { SmGui::EndDisabled(); }
        SmGui::TableSetColumnIndex(1);
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_gainmode_select_", _this->name), &_this->gmId, _this->gainModes.txt)) {
            if (_this->running) {
                _this->applyGainSettings();
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gainMode"] = _this->gainModes.key(_this->gmId);
                config.release(true);
            }
        }
        SmGui::EndTable();

    }

    static void menuStatusHandler(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;

        if (_this->running && _this->bufferProblem.load()) {
            const std::string status = "Buffer: overflow (" +
                std::to_string(_this->bufferOverflowCount.load()) + ")";
            SmGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), status.c_str());
        }
        else if (_this->running) {
            const std::string status = "Buffer: nominal (" +
                std::to_string(_this->bufferOverflowCount.load()) + " overflows)";
            SmGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), status.c_str());
        }
        else {
            SmGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Buffer: stopped");
        }
        SmGui::SameLine();
        if (_this->running && _this->overgain.load()) {
            SmGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Gain: overdrive");
        }
        else if (_this->running) {
            SmGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Gain: OK");
        }
        else {
            SmGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Gain: not monitored");
        }
        if (_this->running) {
            char diagnosticText[256];
            std::snprintf(diagnosticText, sizeof(diagnosticText),
                "I/O latency: refill %.1f / %.1f ms, registers %.1f / %.1f ms",
                _this->lastRefillUs.load() / 1000.0, _this->maxRefillUs.load() / 1000.0,
                _this->lastRegisterUs.load() / 1000.0, _this->maxRegisterUs.load() / 1000.0);
            SmGui::Text(diagnosticText);
            std::snprintf(diagnosticText, sizeof(diagnosticText),
                "Start latency: total %.1f ms, context %.1f, device %.1f, setup %.1f, gain %.1f, tune %.1f ms (preconnect %.1f)",
                _this->startTotalUs.load() / 1000.0, _this->startContextUs.load() / 1000.0,
                _this->startDeviceUs.load() / 1000.0, _this->startSetupUs.load() / 1000.0,
                _this->startGainUs.load() / 1000.0, _this->lastTuneUs.load() / 1000.0,
                _this->preconnectUs.load() / 1000.0);
            SmGui::Text(diagnosticText);
            std::snprintf(diagnosticText, sizeof(diagnosticText),
                "Processing: convert %.1f / %.1f ms, swap %.1f / %.1f ms, cycle max %.1f ms, late %llu",
                _this->lastConvertUs.load() / 1000.0, _this->maxConvertUs.load() / 1000.0,
                _this->lastSwapUs.load() / 1000.0, _this->maxSwapUs.load() / 1000.0,
                _this->maxCycleUs.load() / 1000.0,
                static_cast<unsigned long long>(_this->lateCycleCount.load()));
            SmGui::Text(diagnosticText);
            std::snprintf(diagnosticText, sizeof(diagnosticText),
                "Buffers: requested %u, setup %s (%d), AXI read/clear errors %llu / %llu",
                _this->kernelBuffersRequested.load(),
                _this->kernelBufferSetupResult.load() == 0 ? "OK" : "FAILED",
                _this->kernelBufferSetupResult.load(),
                static_cast<unsigned long long>(_this->axiStatusReadErrors.load()),
                static_cast<unsigned long long>(_this->axiStatusClearErrors.load()));
            SmGui::Text(diagnosticText);
            if (_this->bufferOverflowCount.load() > 0) {
                std::snprintf(diagnosticText, sizeof(diagnosticText),
                    "Last overflow: AXI 0x%08X, refill %.1f ms, registers %.1f ms, previous swap %.1f ms, cycle %.1f ms",
                    _this->overflowRawAxiStatus.load(),
                    _this->overflowRefillUs.load() / 1000.0, _this->overflowRegisterUs.load() / 1000.0,
                    _this->overflowPreviousSwapUs.load() / 1000.0,
                    _this->overflowPreviousCycleUs.load() / 1000.0);
                SmGui::Text(diagnosticText);
            }
        }
    }

    static void menuAdvancedHandler(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::SetNextItemWidth(-31.0f);
        SmGui::ForceSync();
        if (SmGui::Combo("##plutosdr_dev_sel", &_this->devId, _this->devices.txt)) {
            _this->select(_this->devices.key(_this->devId));
            core::setInputSampleRate(_this->samplerate);
            config.acquire();
            config.conf["device"] = _this->devices.key(_this->devId);
            config.release(true);
        }
        SmGui::SameLine();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("R##_pluto_refr_", _this->name), ImVec2(25.0f, 25.0f))) {
            _this->refresh();
            _this->select(_this->devDesc);
            core::setInputSampleRate(_this->samplerate);
        }
        if (_this->running) { SmGui::EndDisabled(); }

        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::BeginTable(CONCAT("PlutoRfIqTable##_", _this->name), 3,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 68.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        SmGui::TableNextRow();
        SmGui::TableSetColumnIndex(0);
        SmGui::Text("RF in / IQ");
        SmGui::TableSetColumnIndex(1);
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_rfinput_select_", _this->name), &_this->rfId, _this->rfInputSelect.txt)) {
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["rfinput"] = _this->rfInputSelect.key(_this->rfId);
                config.release(true);
            }
        }
        SmGui::TableSetColumnIndex(2);
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_iqmode_select_", _this->name), &_this->iqModeId, _this->iqModeSelect.txt)) {
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["iqmode"] = _this->iqModeSelect.key(_this->iqModeId);
                config.release(true);
            }
        }
        SmGui::EndTable();
        if (_this->running) { SmGui::EndDisabled(); }

        SmGui::BeginTable(CONCAT("PlutoBwFilterTable##_", _this->name), 3,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 68.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        SmGui::TableNextRow();
        SmGui::TableSetColumnIndex(0);
        SmGui::Text("BW / Filter");
        SmGui::TableSetColumnIndex(1);
        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_sr_", _this->name), &_this->srId, _this->samplerates.txt)) {
            _this->samplerate = _this->samplerates.value(_this->srId);
            core::setInputSampleRate(_this->samplerate);
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["samplerate"] = _this->samplerate;
                config.release(true);
            }
        }
        if (_this->running) { SmGui::EndDisabled(); }
        SmGui::TableSetColumnIndex(2);
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_bw_", _this->name), &_this->bwId, _this->bandwidths.txt)) {
            _this->bandwidth = _this->bandwidths.value(_this->bwId);
            if (_this->running) {
                _this->setBandwidth(_this->bandwidth);
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["bandwidth"] = _this->bandwidth;
                config.release(true);
            }
        }
        SmGui::EndTable();

    }

    void setBandwidth(int bw) {
        if (bw > 0) {
            iio_channel_attr_write_longlong(rxChan, "rf_bandwidth", bw);
        }
        else {
            iio_channel_attr_write_longlong(rxChan, "rf_bandwidth", std::min<int>(samplerate, 52000000));
        }
    }

    static void worker(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
#ifdef _WIN32
        // Give continuous IIO/DMA draining the highest safe priority within
        // SDR++'s normal process class. Avoid TIME_CRITICAL: starving network
        // and application threads can make reception less reliable.
        if (!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST)) {
            flog::warn("PlutoSDRSourceModule '{0}': Failed to raise receive-thread priority ({1})",
                _this->name, static_cast<int>(GetLastError()));
        }
#endif
        constexpr size_t MAX_BUFFER_PLUTO = 64000000 / 2;
        const size_t requestedBufferSize = static_cast<size_t>(_this->samplerate / 20.0f);
        const size_t blockSize = (std::min)(static_cast<size_t>(STREAM_BUFFER_SIZE), requestedBufferSize);
        // Leave enough DMA buffering to ride through occasional host/libiio
        // transport stalls without losing samples.  At the usual 50 ms block
        // size this provides about 800 ms of buffering.
        const size_t kernelBufferCount = (std::min)(static_cast<size_t>(16), MAX_BUFFER_PLUTO / blockSize);

        // Acquire channels
        iio_channel* rx0_i = iio_device_find_channel(_this->dev, "voltage0", 0);
        iio_channel* rx0_q = iio_device_find_channel(_this->dev, "voltage1", 0);

        long long twoRxTwoTxMode = 0;
        iio_device_debug_attr_read_longlong(_this->phy, "adi,2rx-2tx-mode-enable", &twoRxTwoTxMode);
        if (_this->rfId == 1 && twoRxTwoTxMode == 1) {
            rx0_i = iio_device_find_channel(_this->dev, "voltage2", 0);
            rx0_q = iio_device_find_channel(_this->dev, "voltage3", 0);
        }
        if (!rx0_i || !rx0_q) {
            flog::error("Failed to acquire RX channels");
            _this->bufferProblem.store(true);
            return;
        }

        // Start streaming
        iio_channel_enable(rx0_i);
        if (_this->iqModeId == 0) {
            iio_channel_enable(rx0_q);
        }
        else {
            iio_channel_disable(rx0_q);
        }

        // Allocate buffer
        _this->kernelBuffersRequested.store(static_cast<uint32_t>(kernelBufferCount));
        const int kernelBufferResult = iio_device_set_kernel_buffers_count(
            _this->dev, static_cast<unsigned int>(kernelBufferCount));
        _this->kernelBufferSetupResult.store(kernelBufferResult);
        if (kernelBufferResult < 0) {
            flog::warn("PlutoSDRSourceModule '{0}': Failed to configure {1} kernel buffers ({2})",
                _this->name, kernelBufferCount, kernelBufferResult);
        }
        flog::info("PlutoSDRSourceModule '{0}': Allocate {1} kernel buffers", _this->name, kernelBufferCount);
        flog::info("PlutoSDRSourceModule '{0}': Allocate buffer size {1}", _this->name, blockSize);
        iio_buffer* rxbuf = iio_device_create_buffer(_this->dev, blockSize, false);
        if (!rxbuf) {
            flog::error("Could not create RX buffer");
            _this->bufferProblem.store(true);
            return;
        }

        // Channel enablement and buffer creation complete the IIO streaming
        // transition. Reapply gain afterward so Manual cannot be silently
        // replaced by an AGC state during that transition.
        _this->applyGainSettings();

        _this->bufferProblem.store(false);
        _this->bufferOverflowCount.store(0);
        _this->resetLatencyDiagnostics();
        uint32_t value = 0;
        if (iio_device_reg_read(_this->dev, 0x80000088, &value) == 0) {
            if (iio_device_reg_write(_this->dev, 0x80000088, value) < 0) {
                _this->axiStatusClearErrors.fetch_add(1);
            }
        }
        else {
            _this->axiStatusReadErrors.fetch_add(1);
        }

        using LatencyClock = std::chrono::steady_clock;
        auto elapsedUs = [](LatencyClock::time_point started) {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                LatencyClock::now() - started).count());
        };
        uint64_t previousSwapUs = 0;
        uint64_t previousCycleUs = 0;
        constexpr auto REGISTER_POLL_INTERVAL = std::chrono::milliseconds(500);
        auto lastRegisterPoll = LatencyClock::now();
        bool hardwareOverflow = false;
        const uint32_t overflowStatusRegister =
            (_this->rfId == 1 && twoRxTwoTxMode == 1) ? 0x0000005F : 0x0000005E;

        // Receive loop
        while (true) {
            const auto cycleStarted = LatencyClock::now();
            // Read samples
            const auto refillStarted = LatencyClock::now();
            const ssize_t bytesRead = iio_buffer_refill(rxbuf);
            const uint64_t refillUs = elapsedUs(refillStarted);
            _this->lastRefillUs.store(refillUs);
            updateMaximum(_this->maxRefillUs, refillUs);
            if (bytesRead < 0) {
                _this->bufferProblem.store(true);
                flog::error("PlutoSDRSourceModule '{0}': buffer refill failed ({1})", _this->name, bytesRead);
                break;
            }

            // These are synchronous libiio control transactions. Polling after
            // every sample block can delay the next refill, especially over IP.
            // The AXI overflow bit is sticky, so a 500 ms interval preserves the
            // overflow indication; it only delays its presentation in the UI.
            const auto afterRefill = LatencyClock::now();
            if (afterRefill - lastRegisterPoll >= REGISTER_POLL_INTERVAL) {
                lastRegisterPoll = afterRefill;
                const auto registerStarted = LatencyClock::now();
                uint32_t axiStatus = 0;
                const int axiReadResult = iio_device_reg_read(_this->dev, 0x80000088, &axiStatus);
                if (axiReadResult < 0) {
                    _this->axiStatusReadErrors.fetch_add(1);
                }
                else {
                    const bool hasBufferProblem = (axiStatus & 4U) != 0;
                    _this->bufferProblem.store(hasBufferProblem);
                    if (hasBufferProblem) {
                        _this->bufferOverflowCount.fetch_add(1);
                        _this->overflowRawAxiStatus.store(axiStatus);
                        if (iio_device_reg_write(_this->dev, 0x80000088, axiStatus) < 0) {
                            _this->axiStatusClearErrors.fetch_add(1);
                        }
                    }

                    if (hasBufferProblem) {
                        _this->overflowRefillUs.store(refillUs);
                        _this->overflowPreviousSwapUs.store(previousSwapUs);
                        _this->overflowPreviousCycleUs.store(previousCycleUs);
                    }
                }

                if (iio_device_reg_read(_this->phy, overflowStatusRegister, &value) == 0) {
                    // Bits 0..6 cover RFIR, TFIR, HB1/HB2/HB3, QEC and INT3 overflow.
                    // Bit 7 of channel 1 is BBPLL lock and must not be treated as clipping.
                    hardwareOverflow = (value & 0x7FU) != 0;
                }
                const uint64_t registerUs = elapsedUs(registerStarted);
                _this->lastRegisterUs.store(registerUs);
                updateMaximum(_this->maxRegisterUs, registerUs);
                if (_this->bufferProblem.load()) {
                    _this->overflowRegisterUs.store(registerUs);
                }
            }

            const auto convertStarted = LatencyClock::now();
            bool sampleClipping = false;
            if (_this->iqModeId == 0) {
                auto* buffer = static_cast<int16_t*>(iio_buffer_start(rxbuf));
                if (!buffer) { break; }
                for (size_t i = 0; i < blockSize * 2; ++i) {
                    if (buffer[i] <= -2048 || buffer[i] >= 2047) {
                        sampleClipping = true;
                        break;
                    }
                }
                volk_16i_s32f_convert_32f(reinterpret_cast<float*>(_this->stream.writeBuf), buffer, 2048.0f, blockSize * 2);
            }
            else {
                auto* buffer = static_cast<int8_t*>(iio_buffer_start(rxbuf));
                if (!buffer) { break; }
                for (size_t i = 0; i < blockSize * 2; ++i) {
                    if (buffer[i] <= -128 || buffer[i] >= 127) {
                        sampleClipping = true;
                        break;
                    }
                }
                volk_8i_s32f_convert_32f(reinterpret_cast<float*>(_this->stream.writeBuf), buffer, 128.0f, blockSize * 2);
            }

            _this->overgain.store(hardwareOverflow || sampleClipping);
            const uint64_t convertUs = elapsedUs(convertStarted);
            _this->lastConvertUs.store(convertUs);
            updateMaximum(_this->maxConvertUs, convertUs);

            const auto swapStarted = LatencyClock::now();
            if (!_this->stream.swap(blockSize)) { break; }
            const uint64_t swapUs = elapsedUs(swapStarted);
            _this->lastSwapUs.store(swapUs);
            updateMaximum(_this->maxSwapUs, swapUs);
            const uint64_t cycleUs = elapsedUs(cycleStarted);
            updateMaximum(_this->maxCycleUs, cycleUs);
            if (cycleUs > 75000) _this->lateCycleCount.fetch_add(1);
            previousSwapUs = swapUs;
            previousCycleUs = cycleUs;
        }

        // Stop streaming
        iio_channel_disable(rx0_i);
        iio_channel_disable(rx0_q);

        // Free buffer
        iio_buffer_destroy(rxbuf);
    }

    std::string name;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    std::thread workerThread;
    std::thread preconnectThread;
    iio_context* ctx = NULL;
    iio_device* phy = NULL;
    iio_device* dev = NULL;
    iio_channel* rxLO = NULL;
    iio_channel* rxChan = NULL;
    bool running = false;
    std::atomic<bool> shuttingDown{false};
    std::atomic<bool> preconnectReady{false};
    std::atomic<uint64_t> preconnectUs{0};

    std::string devDesc = "";
    std::string uri = "";

    double freq;
    int samplerate = 4000000;
    int bandwidth = 0;
    float gain = -1;

    int devId = 0;
    int srId = 0;
    int bwId = 0;
    int gmId = 0;
    int rfId = 0;
    int iqModeId = 0;
    std::atomic<bool> bufferProblem{false};
    std::atomic<uint64_t> bufferOverflowCount{0};
    std::atomic<bool> overgain{false};
    std::atomic<uint64_t> lastRefillUs{0}, maxRefillUs{0};
    std::atomic<uint64_t> startTotalUs{0}, startContextUs{0}, startDeviceUs{0};
    std::atomic<uint64_t> startSetupUs{0}, startGainUs{0}, lastTuneUs{0};
    std::atomic<uint64_t> lastRegisterUs{0}, maxRegisterUs{0};
    std::atomic<uint64_t> lastConvertUs{0}, maxConvertUs{0};
    std::atomic<uint64_t> lastSwapUs{0}, maxSwapUs{0};
    std::atomic<uint64_t> maxCycleUs{0}, lateCycleCount{0};
    std::atomic<uint64_t> overflowRefillUs{0}, overflowRegisterUs{0};
    std::atomic<uint64_t> overflowPreviousSwapUs{0}, overflowPreviousCycleUs{0};
    std::atomic<uint32_t> kernelBuffersRequested{0};
    std::atomic<int> kernelBufferSetupResult{0};
    std::atomic<uint64_t> axiStatusReadErrors{0}, axiStatusClearErrors{0};
    std::atomic<uint32_t> overflowRawAxiStatus{0};

    static void updateMaximum(std::atomic<uint64_t>& maximum, uint64_t value) {
        uint64_t observed = maximum.load(std::memory_order_relaxed);
        while (observed < value &&
               !maximum.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {}
    }

    void resetLatencyDiagnostics() {
        lastRefillUs = maxRefillUs = 0;
        lastRegisterUs = maxRegisterUs = 0;
        lastConvertUs = maxConvertUs = 0;
        lastSwapUs = maxSwapUs = 0;
        maxCycleUs = lateCycleCount = 0;
        overflowRefillUs = overflowRegisterUs = 0;
        overflowPreviousSwapUs = overflowPreviousCycleUs = 0;
        axiStatusReadErrors = axiStatusClearErrors = 0;
        overflowRawAxiStatus = 0;
    }

    OptionList<std::string, std::string> devices;
    OptionList<int, double> samplerates;
    OptionList<int, double> bandwidths;
    OptionList<std::string, std::string> gainModes;
    OptionList<std::string, std::string> rfInputSelect;
    OptionList<std::string, std::string> iqModeSelect;
};

MOD_EXPORT void _INIT_() {
    json defConf = {};
    defConf["device"] = "";
    defConf["devices"] = {};
    config.setPath(core::args["root"].s() + "/plutosdr_source_config.json");
    config.load(defConf);
    config.enableAutoSave();

    // Reset the configuration if the old format is still used
    config.acquire();
    if (!config.conf.contains("device") || !config.conf.contains("devices")) {
        config.conf = defConf;
        config.release(true);
    }
    else {
        config.release();
    }
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new PlutoSDRSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (PlutoSDRSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
