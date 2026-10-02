#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <shellapi.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <core.h>
#include <config.h>
#include <gui/gui.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>
#include <fftw3.h>
#include "p1_tables.h"
#include "DVB_T2/dvbt2_definition.h"
#include "DVB_T2/pilot_generator.h"
#include "DVB_T2/address_freq_deinterleaver.h"
#include "DVB_T2/LDPC/dvb_t2_tables.hh"
#include "DVB_T2/LDPC/algorithms.hh"
#include "DVB_T2/LDPC/layered_decoder.hh"
#include <bch/bch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <chrono>
#include <ctime>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
using Dvbt2UdpSocket = SOCKET;
constexpr Dvbt2UdpSocket INVALID_DVBT2_UDP_SOCKET = INVALID_SOCKET;
#else
using Dvbt2UdpSocket = int;
constexpr Dvbt2UdpSocket INVALID_DVBT2_UDP_SOCKET = -1;
#endif

constexpr int DVB_T2_TABLE_NORMAL_C2_3::DEG[];
constexpr int DVB_T2_TABLE_NORMAL_C2_3::LEN[];
constexpr int DVB_T2_TABLE_NORMAL_C2_3::POS[];

SDRPP_MOD_INFO{
    "dvbt2_decoder",
    "DVB-T2 wideband signal and P1 preamble analyzer",
    "Nick / OpenAI; P1 structure based on ETSI DVB-T2 and Oleg Malyutin's receiver",
    0, 1, 0,
    -1
};

ConfigManager config;

namespace {
constexpr double DVB_SAMPLE_RATE = 64000000.0 / 7.0;
constexpr double INPUT_SAMPLE_RATE = 10000000.0; // rational-friendly SDR++ VFO boundary
constexpr double CHANNEL_BANDWIDTH = 8000000.0;
constexpr int P1_B = 527; // round(482 * SAMPLE_RATE / DVB_SAMPLE_RATE)
constexpr int P1_LEN = 2240;
constexpr int P2_CAPTURE_SAMPLES = 100000;
constexpr int MAX_FRAME_CAPTURE_SAMPLES = 2200000;
constexpr size_t UDP_DATAGRAM_QUEUE_CAPACITY = 256;
constexpr double PI = 3.14159265358979323846;
constexpr int RESAMPLE_HALF_TAPS = 12;
constexpr int RESAMPLE_TAP_COUNT = 2 * RESAMPLE_HALF_TAPS;
constexpr int RESAMPLE_PHASES = 1024;
constexpr double RESAMPLE_CUTOFF = 0.45;
constexpr int FRAME_QUEUE_CAPACITY = 2;
constexpr int MAX_COMPUTE_WORKERS = 8;
constexpr int TS_RESYNC_BLOCK_BUCKETS = 256;
constexpr size_t QAM_CONSTELLATION_POINTS = 384;
constexpr size_t QAM_CONSTELLATION_HISTORY = 3;
constexpr const char* FFTW_WISDOM_PATH = "dvbt2_fftw_wisdom.dat";

int chooseComputeWorkerCount(unsigned int logicalThreads) {
    // Leave roughly half of the logical processors available to the SDR source,
    // UI, audio, UDP output and operating system.  Eight workers are enough to
    // saturate the frame-level DVB-T2 stages without creating an unbounded
    // number of latency-sensitive threads on high-core-count machines.
    if (logicalThreads == 0) return 4;
    return (std::max)(2, (std::min)(MAX_COMPUTE_WORKERS,
        static_cast<int>(logicalThreads / 2)));
}

void lowerDecoderThreadPriority() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
}

inline float magnitudeSquared(const dsp::complex_t& value) {
    return value.re * value.re + value.im * value.im;
}

inline dsp::complex_t multiplyConjugate(const dsp::complex_t& a, const dsp::complex_t& b) {
    return {a.re * b.re + a.im * b.im, a.im * b.re - a.re * b.im};
}

inline dsp::complex_t interpolateChannelPolar(const dsp::complex_t& a, const dsp::complex_t& b, float t) {
    // The sampled OFDM channel response is complex-valued, so interpolate its
    // real and imaginary components directly between adjacent pilots.  Besides
    // being the conventional frequency-domain estimate, this avoids doing five
    // transcendental operations for every data carrier in the frame.
    return {a.re + (b.re - a.re) * t, a.im + (b.im - a.im) * t};
}

inline dsp::complex_t resampleDvbAt(const dsp::complex_t* samples, int sampleCount, double position) {
    // Windowed-sinc reconstruction plus anti-alias filtering for the fixed
    // 10 MHz -> 64/7 MHz DVB-T2 conversion.  Linear interpolation imposes a
    // large sinc-squared error across an 8 MHz OFDM channel.
    static const auto coefficients = [] {
        std::array<std::array<float, RESAMPLE_TAP_COUNT>, RESAMPLE_PHASES> table{};
        for (int phase = 0; phase < RESAMPLE_PHASES; ++phase) {
            const double fraction = static_cast<double>(phase) / RESAMPLE_PHASES;
            double sum = 0.0;
            for (int tapIndex = 0; tapIndex < RESAMPLE_TAP_COUNT; ++tapIndex) {
                const int tap = tapIndex - RESAMPLE_HALF_TAPS + 1;
                const double distance = tap - fraction;
                const double argument = 2.0 * RESAMPLE_CUTOFF * distance;
                const double sinc = std::abs(argument) < 1.0e-12
                    ? 1.0 : std::sin(PI * argument) / (PI * argument);
                const double normalized = distance / RESAMPLE_HALF_TAPS;
                const double window = 0.42 + 0.5 * std::cos(PI * normalized) +
                                      0.08 * std::cos(2.0 * PI * normalized);
                table[phase][tapIndex] = static_cast<float>(2.0 * RESAMPLE_CUTOFF * sinc * window);
                sum += table[phase][tapIndex];
            }
            for (float& coefficient : table[phase]) coefficient = static_cast<float>(coefficient / sum);
        }
        return table;
    }();
    const int center = static_cast<int>(std::floor(position));
    const double fraction = position - center;
    const int phase = (std::min)(static_cast<int>(fraction * RESAMPLE_PHASES), RESAMPLE_PHASES - 1);
    float sumRe = 0.0f, sumIm = 0.0f;
    const int firstIndex = center - RESAMPLE_HALF_TAPS + 1;
    if (firstIndex >= 0 && firstIndex + RESAMPLE_TAP_COUNT <= sampleCount) {
        // Full-frame FFT windows always take this interior path.  Coefficients
        // are already normalized, so the fixed-size loop has no branches or
        // per-sample division and can be vectorized by the AVX2 compiler.
        for (int tapIndex = 0; tapIndex < RESAMPLE_TAP_COUNT; ++tapIndex) {
            const float weight = coefficients[phase][tapIndex];
            sumRe += samples[firstIndex + tapIndex].re * weight;
            sumIm += samples[firstIndex + tapIndex].im * weight;
        }
        return {sumRe, sumIm};
    }
    float sumWeight = 0.0f;
    for (int tapIndex = 0; tapIndex < RESAMPLE_TAP_COUNT; ++tapIndex) {
        const int tap = tapIndex - RESAMPLE_HALF_TAPS + 1;
        const int index = center + tap;
        if (index < 0 || index >= sampleCount) continue;
        const float weight = coefficients[phase][tapIndex];
        sumRe += samples[index].re * weight;
        sumIm += samples[index].im * weight;
        sumWeight += weight;
    }
    if (std::abs(sumWeight) < 1.0e-12f) return {0.0f, 0.0f};
    return {sumRe / sumWeight, sumIm / sumWeight};
}

inline float normalizeQamCells(std::vector<dsp::complex_t>& cells, float maximumRadius = 2.2f) {
    if (cells.empty()) return 1.0f;
    double energy = 0.0;
    size_t validCells = 0;
    for (const auto& cell : cells) {
        const float power = magnitudeSquared(cell);
        if (std::isfinite(power)) {
            energy += power;
            ++validCells;
        }
    }
    // Every normalized DVB-T2 constellation has unit mean symbol energy.  The
    // median radius is modulation-dependent (and is not one for 256-QAM), so
    // using it biases all decision thresholds and LLR magnitudes.
    const double meanEnergy = validCells > 0 ? energy / static_cast<double>(validCells) : 0.0;
    const float scale = meanEnergy > 1.0e-18 ? static_cast<float>(1.0 / std::sqrt(meanEnergy)) : 1.0f;
    for (auto& cell : cells) {
        cell.re *= scale;
        cell.im *= scale;
        const float radiusSquared = magnitudeSquared(cell);
        if (!std::isfinite(radiusSquared)) {
            cell = {0.0f, 0.0f};
        } else if (radiusSquared > maximumRadius * maximumRadius) {
            const float limiter = maximumRadius / std::sqrt(radiusSquared);
            cell.re *= limiter;
            cell.im *= limiter;
        }
    }
    return scale;
}

template <size_t Size>
void copyStringToBuffer(char (&destination)[Size], const std::string& source) {
    const size_t length = (std::min)(source.size(), Size - 1);
    std::memcpy(destination, source.data(), length);
    destination[length] = '\0';
}

}

class Dvbt2DecoderModule : public ModuleManager::Instance {
public:
    explicit Dvbt2DecoderModule(std::string instanceName) : name(std::move(instanceName)) {
#ifdef _WIN32
        WSADATA winsockData{};
        udpWinsockReady = WSAStartup(MAKEWORD(2, 2), &winsockData) == 0;
#endif
        config.acquire();
        auto& moduleConfig = config.conf[name];
        if (!moduleConfig.contains("displayMode")) moduleConfig["displayMode"] = 0;
        if (!moduleConfig.contains("udpHost")) moduleConfig["udpHost"] = "127.0.0.1";
        if (!moduleConfig.contains("udpPort")) moduleConfig["udpPort"] = 1234;
        if (!moduleConfig.contains("selectedRecordingPath"))
            moduleConfig["selectedRecordingPath"] = "recordings/dvbt2_selected.ts";
        if (!moduleConfig.contains("fullRecordingPath"))
            moduleConfig["fullRecordingPath"] = "recordings/dvbt2_output.ts";
        if (!moduleConfig.contains("appendRecordingTimestamp"))
            moduleConfig["appendRecordingTimestamp"] = false;
        if (!moduleConfig.contains("playerCommand")) {
            const std::string legacyPath = moduleConfig.value("playerPath", std::string("vlc.exe"));
            const std::string legacyParameters = moduleConfig.value("playerParameters",
                std::string("--video-on-top --network-caching=750 --live-caching=750"));
            moduleConfig["playerCommand"] = legacyPath + " " + legacyParameters + " udp://@:$port";
        }
        const std::string previousDefaultCommand =
            "vlc.exe --video-on-top --network-caching=750 --live-caching=750 udp://@:$port";
        if (moduleConfig["playerCommand"].get<std::string>() == previousDefaultCommand) {
            moduleConfig["playerCommand"] =
                "vlc.exe --video-on-top --network-caching=750\n--live-caching=750 udp://@:$port";
        }
        displayMode = (std::max)(0, (std::min)(3, moduleConfig["displayMode"].get<int>()));
        const std::string configuredUdpHost = moduleConfig["udpHost"].get<std::string>();
        copyStringToBuffer(udpHost, configuredUdpHost);
        udpPort = (std::max)(1, (std::min)(65535, moduleConfig["udpPort"].get<int>()));
        const std::string configuredSelectedPath = moduleConfig["selectedRecordingPath"].get<std::string>();
        const std::string configuredFullPath = moduleConfig["fullRecordingPath"].get<std::string>();
        copyStringToBuffer(selectedRecordingPath, configuredSelectedPath);
        copyStringToBuffer(fullRecordingPath, configuredFullPath);
        appendRecordingTimestamp.store(moduleConfig["appendRecordingTimestamp"].get<bool>());
        const std::string configuredPlayerCommand = moduleConfig["playerCommand"].get<std::string>();
        copyStringToBuffer(playerCommand, configuredPlayerCommand);
        config.release(true);
        fftInput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(1024));
        fftOutput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(1024));
        fftPlan = fftwf_plan_dft_1d(1024, reinterpret_cast<fftwf_complex*>(fftInput),
                                    reinterpret_cast<fftwf_complex*>(fftOutput), FFTW_FORWARD, FFTW_ESTIMATE);
        // FFTW_MEASURE benchmarks several 32K FFT strategies for this CPU.
        // Persisting wisdom makes that one-time search reusable on later runs.
        fftwf_import_wisdom_from_filename(FFTW_WISDOM_PATH);
        p2FftInput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(32768));
        p2FftOutput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(32768));
        p2FftPlan = fftwf_plan_dft_1d(32768, reinterpret_cast<fftwf_complex*>(p2FftInput),
                                     reinterpret_cast<fftwf_complex*>(p2FftOutput), FFTW_FORWARD, FFTW_MEASURE);
        fullFrameCapture.resize(MAX_FRAME_CAPTURE_SAMPLES);
        frameWorkBuffer.resize(MAX_FRAME_CAPTURE_SAMPLES);
        for (auto& queued : queuedFrames) queued.samples.resize(MAX_FRAME_CAPTURE_SAMPLES);
        workerFftInput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(32768));
        workerFftOutput = reinterpret_cast<dsp::complex_t*>(fftwf_alloc_complex(32768));
        workerFftPlan = fftwf_plan_dft_1d(32768, reinterpret_cast<fftwf_complex*>(workerFftInput),
                                         reinterpret_cast<fftwf_complex*>(workerFftOutput), FFTW_FORWARD, FFTW_MEASURE);
        fftwf_export_wisdom_to_filename(FFTW_WISDOM_PATH);
        dvbt2Params = {};
        dvbt2Params.preamble = T2_SISO;
        dvbt2Params.fft_mode = FFTSIZE_32K;
        dvbt2Params.carrier_mode = CARRIERS_EXTENDED;
        dvbt2_p2_parameters_init(dvbt2Params);
        p2Pilots.p2_generator(dvbt2Params);
        p2Addresses.init(dvbt2Params);
        p2Addresses.p2_address_freq_deinterleaver(dvbt2Params);
        logicalProcessorCount = (std::max)(1u, std::thread::hardware_concurrency());
        computeWorkerLimit = chooseComputeWorkerCount(logicalProcessorCount);
        for (int worker = 0; worker < computeWorkerLimit; ++worker)
            computeWorkers.emplace_back(&Dvbt2DecoderModule::computeWorkerLoop, this, worker);
        frameWorker = std::thread(&Dvbt2DecoderModule::frameWorkerLoop, this);
        udpSenderThread = std::thread(&Dvbt2DecoderModule::udpSenderLoop, this);
        initP1Randomizer();
        createVFO();
        sink.init(vfo->output, iqHandler, this);
        sink.start();
        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~Dvbt2DecoderModule() override {
        gui::menu.removeEntry(name);
        decodeGeneration.fetch_add(1);
        if (enabled) {
            sink.stop();
            sigpath::vfoManager.deleteVFO(vfo);
        }
        {
            std::lock_guard<std::mutex> lock(frameWorkerMutex);
            frameWorkerStop = true;
        }
        frameWorkerCv.notify_one();
        if (frameWorker.joinable()) frameWorker.join();
        {
            std::lock_guard<std::mutex> lock(computeMutex);
            computeStop = true;
            ++computeGeneration;
        }
        computeCv.notify_all();
        for (auto& worker : computeWorkers) if (worker.joinable()) worker.join();
        {
            std::lock_guard<std::mutex> lock(udpQueueMutex);
            udpSenderStop = true;
        }
        udpQueueCv.notify_one();
        if (udpSenderThread.joinable()) udpSenderThread.join();
        closeUdpSocket();
#ifdef _WIN32
        if (udpWinsockReady) WSACleanup();
#endif
        if (fftPlan) fftwf_destroy_plan(fftPlan);
        if (fftInput) fftwf_free(fftInput);
        if (fftOutput) fftwf_free(fftOutput);
        if (p2FftPlan) fftwf_destroy_plan(p2FftPlan);
        if (p2FftInput) fftwf_free(p2FftInput);
        if (p2FftOutput) fftwf_free(p2FftOutput);
        if (workerFftPlan) fftwf_destroy_plan(workerFftPlan);
        if (workerFftInput) fftwf_free(workerFftInput);
        if (workerFftOutput) fftwf_free(workerFftOutput);
    }

    void postInit() override {}
    void enable() override {
        if (enabled) return;
        resetDetector();
        createVFO();
        sink.setInput(vfo->output);
        sink.start();
        enabled = true;
    }
    void disable() override {
        if (!enabled) return;
        // Invalidate the active frame before disconnecting the live sample path.
        // Long LDPC recovery passes must not continue competing with acquisition
        // after the user has disabled this module.
        decodeGeneration.fetch_add(1);
        sink.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        vfo = nullptr;
        {
            std::lock_guard<std::mutex> lock(frameWorkerMutex);
            queuedFrameCount = 0;
            frameWorkPending = false;
            workerQueueDepth.store(0);
            for (auto& queued : queuedFrames) queued.p2Payload.clear();
        }
        enabled = false;
    }
    bool isEnabled() override { return enabled; }

private:
    struct TsElementaryStream {
        int pid = -1;
        int type = -1;
    };

    struct TsServiceInfo {
        int serviceId = -1;
        int pmtPid = -1;
        int pcrPid = -1;
        std::string name;
        std::string provider;
        std::vector<TsElementaryStream> streams;
    };

    struct PsiAssemblyState {
        std::vector<uint8_t> bytes;
        size_t expected = 0;
        int continuity = 0;
        bool continuityValid = false;
    };

    void createVFO() {
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0.0,
                                            CHANNEL_BANDWIDTH, INPUT_SAMPLE_RATE,
                                            CHANNEL_BANDWIDTH, CHANNEL_BANDWIDTH, true);
        vfo->setSnapInterval(1000.0);
    }

    void resetDetector() {
        delayed.fill({0.0f, 0.0f});
        captureRing.fill({0.0f, 0.0f});
        products.fill({0.0f, 0.0f});
        currentEnergy.fill(0.0f);
        delayedEnergy.fill(0.0f);
        delayPos = productPos = 0;
        capturePos = 0;
        sumProduct = {0.0f, 0.0f};
        sumCurrentEnergy = sumDelayedEnergy = 0.0;
        oscillatorRe = 1.0f;
        oscillatorIm = 0.0f;
        oscillatorSamples = 0;
        samplesSeen = lastP1Sample = lastCandidateSample = nextPredictedFrameSample = 0;
        candidateSamples = 0;
        repeatConfirmations = 0;
        expectedFrameSamples = 0.0;
        peakMetric = 0.0f;
        displayedMetric = 0.0f;
        p1Locked = false;
        p1Count = 0;
        p1PredictedCaptures = 0;
        decodedPreamble = -1;
        decodedFft = -1;
        decodedMixed = false;
        p2Collecting = false;
        p2Fill = 0;
        p2Synchronized = false;
        p2Correlation = 0.0f;
        guardInterval = -1;
        p2Count = 0;
        l1PreValid = false;
        l1LiveValid = false;
        l1CrcAttempts = 0;
        l1CrcValidFrames = 0;
        l1SoftAccumulator.fill(0.0f);
        l1SoftCount = 0;
        l1AlignmentLocked = false;
        l1PilotPattern = -1;
        l1CellId = -1;
        l1NetworkId = -1;
        l1SystemId = -1;
        l1Frames = -1;
        l1DataSymbols = -1;
        l1CarrierShift = 0;
        l1DecisionQuality = 0.0f;
        l1SpectrumMirrored = false;
        l1PostValid = false;
        l1PostAttempts = 0;
        l1PostValidFrames = 0;
        l1NumPlps = -1;
        dataPilotsReady = false;
        dataSymbolSynchronized = false;
        dataPilotCorrelation = 0.0f;
        dataSymbolCount = 0;
        fullFrameCollecting = false;
        fullFrameFill = 0;
        fullFrameTarget = 0;
        fullFrameSnapshotReady = false;
        fullFrameCount = 0;
        frameSymbolsVerified = 0;
        frameMeanCpCorrelation = 0.0f;
        frameTimingCorrection = 0;
        workerFramesProcessed = 0;
        workerFramesSkipped = 0;
        workerFramesQueued = 0;
        queuedFrameHead = 0;
        queuedFrameCount = 0;
        workerQueueDepth = 0;
        workerQueuePeak = 0;
        workerFrameMilliseconds = 0.0f;
        workerLdpcMilliseconds = 0.0f;
        workerOfdmMilliseconds = 0.0f;
        workerDemapMilliseconds = 0.0f;
        workerResampleMilliseconds = 0.0f;
        workerFftMilliseconds = 0.0f;
        workerEqualizeMilliseconds = 0.0f;
        workerFrameAverageMilliseconds = 0.0f;
        workerFrameMaximumMilliseconds = 0.0f;
        workerSymbolsEqualized = 0;
        workerMeanPilotCorrelation = 0.0f;
        workerDataCells = 0;
        frameClosingAcquired = false;
        frameClosingPilotCorrelation = 0.0f;
        frameClosingCells = 0;
        l1DynamicValid = false;
        currentP2PayloadReady = false;
        plpFramesAssembled = 0;
        plpCellsAvailable = 0;
        plpCellsRequired = 0;
        plpCellsExtracted = 0;
        l1SubSlicesPerFrame = -1;
        plpNumBlocksMax = -1;
        plpFrameInterval = -1;
        plpTimeIlLength = -1;
        plpTimeIlType = -1;
        tiBlocksCompleted = 0;
        fecBlocksReordered = 0;
        softQamValues = 0;
        qamSnrDb = -120.0f;
        bitDeinterleavedBlocks = 0;
        ldpcBlocksTested = 0;
        ldpcBlocksConverged = 0;
        ldpcAlternateAttempts = 0;
        ldpcAlternateRecovered = 0;
        ldpcHalfScaleAttempts = 0;
        ldpcHalfScaleRecovered = 0;
        ldpcQuarterScaleAttempts = 0;
        ldpcQuarterScaleRecovered = 0;
        bchRecoveryAttempts = 0;
        bchRecoveredBlocks = 0;
        bchCorrectedBits = 0;
        residualFecCapturesAttempted = 0;
        residualFecCapturesSaved = 0;
        ldpcFirstBlockTested = 0;
        ldpcMeanRawUnsatisfied = 0.0f;
        ldpcMeanUnsatisfied = 0.0f;
        bbHeadersTested = 0;
        bbHeadersValid = 0;
        bbInputMode = -1;
        bbTsGs = -1;
        bbUpl = -1;
        bbDfl = -1;
        bbSync = -1;
        bbSyncd = -1;
        bbNpd = false;
        tsPacketsWritten = 0;
        tsBytesWritten = 0;
        tsResyncs = 0;
        tsNullPacketsReinserted = 0;
        tsResyncFirstBlock = 0;
        tsResyncTiBoundary = 0;
        tsResyncOtherBlock = 0;
        tsLastExpectedSyncBytes = -1;
        tsLastActualSyncBytes = -1;
        tsLastResyncBlock = -1;
        tsResyncForwardOne = 0;
        tsResyncBackwardOne = 0;
        tsResyncIrregular = 0;
        selectedTsService = -1;
        selectedTsPacketsWritten = 0;
        selectedPatVersion = 0;
        selectedRecordedPackets = 0;
        fullRecordedPackets = 0;
        udpDatagramsSent = 0;
        udpErrors = 0;
        udpDatagramsDropped = 0;
        selectedRecordingRestartRequested = true;
        fullRecordingRestartRequested = true;
        udpRestartRequested = true;
        tsTransportStreamId = 1;
        tsPatVersion = 0;
        for (auto& count : tsResyncByBlock) count.store(0);
        {
            std::lock_guard<std::mutex> lock(tsOutputMutex);
            tsPacketPosition = 0;
            tsSynchronized = false;
            tsAwaitingDnp = false;
            if (tsOutput.is_open()) tsOutput.close();
            if (selectedTsOutput.is_open()) selectedTsOutput.close();
            selectedTsOutputService = -1;
            std::lock_guard<std::mutex> serviceLock(tsServiceMutex);
            tsServices.clear();
            tsPmtPids.clear();
            tsPsiStates.clear();
        }
        ldpcSelfTestRun = false;
        ldpcSelfTestStatus = -1;
        debugFrameWritten = false;
        debugSymbolWritten = false;
        debugSymbolSaved = false;
        debugAllSymbolsWritten = false;
        debugAllSymbolsSaved = false;
        debugTimeDeinterleaverWritten = false;
        debugTimeDeinterleaverSaved = false;
        debugFecInputWritten = false;
        debugFecInputSaved = false;
        debugRawFrameWritten = false;
        debugRawFrameSaved = false;
        correlation.store(0.0f);
        levelDb.store(-120.0f);
        coarseOffset.store(0.0f);
        {
            std::lock_guard<std::mutex> lock(qamConstellationMutex);
            qamConstellationCounts.fill(0);
            qamConstellationHead = 0;
        }
    }

    void drawQamConstellation() {
        std::array<std::array<dsp::complex_t, QAM_CONSTELLATION_POINTS>, QAM_CONSTELLATION_HISTORY> history{};
        std::array<size_t, QAM_CONSTELLATION_HISTORY> historyCounts{};
        size_t historyHead = 0;
        {
            std::lock_guard<std::mutex> lock(qamConstellationMutex);
            historyCounts = qamConstellationCounts;
            historyHead = qamConstellationHead;
            for (size_t frame = 0; frame < QAM_CONSTELLATION_HISTORY; ++frame) {
                std::copy_n(qamConstellationHistory[frame].begin(), historyCounts[frame], history[frame].begin());
            }
        }
        const size_t pointCount = historyCounts[historyHead];
        if (pointCount == 0) return;

        ImGui::Text("QAM constellation: %d sampled cells", static_cast<int>(pointCount));
        const float plotSize = (std::min)(350.0f, (std::max)(140.0f, ImGui::GetContentRegionAvail().x));
        const ImVec2 plotMin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##QAMConstellation", ImVec2(plotSize, plotSize));
        const ImVec2 plotMax(plotMin.x + plotSize, plotMin.y + plotSize);
        auto* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(plotMin, plotMax, IM_COL32(0, 0, 0, 255));
        const ImU32 gridColor = IM_COL32(70, 70, 70, 255);
        const float centerX = plotMin.x + plotSize * 0.5f;
        const float centerY = plotMin.y + plotSize * 0.5f;
        drawList->AddLine(ImVec2(centerX, plotMin.y), ImVec2(centerX, plotMax.y), gridColor);
        drawList->AddLine(ImVec2(plotMin.x, centerY), ImVec2(plotMax.x, centerY), gridColor);
        drawList->AddRect(plotMin, plotMax, gridColor);

        // Normalized 256-QAM outer points are near +/-1.63.  A slightly wider
        // view keeps noisy edge cells visible without wasting plot area.
        constexpr float extent = 1.75f;
        const float scale = plotSize / (2.0f * extent);
        drawList->PushClipRect(plotMin, plotMax, true);
        for (size_t age = 0; age < QAM_CONSTELLATION_HISTORY; ++age) {
            const size_t frame = (historyHead + age + 1) % QAM_CONSTELLATION_HISTORY;
            const int alpha = 45 + static_cast<int>(age) * 210 /
                static_cast<int>(QAM_CONSTELLATION_HISTORY - 1);
            const ImU32 pointColor = IM_COL32(80, 225, 255, alpha);
            for (size_t i = 0; i < historyCounts[frame]; ++i) {
                const float x = centerX + history[frame][i].re * scale;
                const float y = centerY - history[frame][i].im * scale;
                drawList->AddRectFilled(ImVec2(x, y), ImVec2(x + 1.0f, y + 1.0f), pointColor);
            }
        }
        drawList->PopClipRect();
    }

    void saveUiConfiguration() {
        std::string configuredUdpHost;
        int configuredUdpPort = 1234;
        std::string configuredPlayerCommand;
        {
            std::lock_guard<std::mutex> lock(udpConfigMutex);
            configuredUdpHost = udpHost;
            configuredUdpPort = udpPort;
            configuredPlayerCommand = playerCommand;
        }
        std::string configuredSelectedPath;
        std::string configuredFullPath;
        {
            std::lock_guard<std::mutex> lock(recordingConfigMutex);
            configuredSelectedPath = selectedRecordingPath;
            configuredFullPath = fullRecordingPath;
        }
        config.acquire();
        config.conf[name]["displayMode"] = displayMode;
        config.conf[name]["udpHost"] = configuredUdpHost;
        config.conf[name]["udpPort"] = configuredUdpPort;
        config.conf[name]["playerCommand"] = configuredPlayerCommand;
        config.conf[name]["selectedRecordingPath"] = configuredSelectedPath;
        config.conf[name]["fullRecordingPath"] = configuredFullPath;
        config.conf[name]["appendRecordingTimestamp"] = appendRecordingTimestamp.load();
        config.release(true);
    }

    static std::string timestampedRecordingPath(const std::string& configuredPath) {
        const std::filesystem::path path(configuredPath);
        const std::time_t now = std::time(nullptr);
        std::tm localTime{};
#ifdef _WIN32
        localtime_s(&localTime, &now);
#else
        localtime_r(&now, &localTime);
#endif
        char timestamp[32]{};
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &localTime);
        return (path.parent_path() /
            (path.stem().string() + "_" + timestamp + path.extension().string())).string();
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<Dvbt2DecoderModule*>(ctx);
        if (!self->enabled) return;

        const auto drawSelectedOutputStatus = [self] {
            ImGui::Text("Selected filtered output: %llu packets",
                        static_cast<unsigned long long>(self->selectedTsPacketsWritten.load()));
            ImGui::Text("UDP: %llu datagrams, %llu errors, queue %d, dropped %llu",
                        static_cast<unsigned long long>(self->udpDatagramsSent.load()),
                        static_cast<unsigned long long>(self->udpErrors.load()),
                        self->udpQueueDepth.load(),
                        static_cast<unsigned long long>(self->udpDatagramsDropped.load()));
            ImGui::Separator();
        };

        float threshold = self->threshold.load();
        ImGui::TextUnformatted("P1 Threshold");
        ImGui::SameLine();
        const ImGuiStyle& style = ImGui::GetStyle();
        const float resetButtonWidth = ImGui::CalcTextSize("Reset Acquisition").x +
                                       style.FramePadding.x * 2.0f;
        ImGui::SetNextItemWidth((std::max)(1.0f,
            ImGui::GetContentRegionAvail().x - resetButtonWidth - style.ItemSpacing.x));
        if (ImGui::SliderFloat("##P1Threshold", &threshold, 0.35f, 0.90f, "%.2f")) {
            self->threshold.store(threshold);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Acquisition")) self->resetRequested.store(true);
        ImGui::Separator();

        if (gui::waterfall.getBandwidth() < CHANNEL_BANDWIDTH * 1.02) {
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Source bandwidth may be too narrow; select at least 10 MHz on Pluto+.");
        }
        bool displayModeChanged = ImGui::RadioButton("Info", &self->displayMode, 0);
        ImGui::SameLine();
        displayModeChanged |= ImGui::RadioButton("Info + Graph", &self->displayMode, 1);
        ImGui::SameLine();
        displayModeChanged |= ImGui::RadioButton("Advanced", &self->displayMode, 2);
        ImGui::SameLine();
        displayModeChanged |= ImGui::RadioButton("Settings", &self->displayMode, 3);
        if (displayModeChanged) self->saveUiConfiguration();
        ImGui::Separator();

        const bool showAdvancedInfo = self->displayMode == 2;
        const bool showGraph = self->displayMode == 1 || self->displayMode == 2;
        if (self->displayMode == 3) {
            bool settingsChanged = false;
            ImGui::TextUnformatted("UDP Address / Port:");
            ImGui::SameLine();
            {
                std::lock_guard<std::mutex> lock(self->udpConfigMutex);
                constexpr float udpPortControlWidth = 120.0f;
                ImGui::SetNextItemWidth((std::max)(1.0f,
                    ImGui::GetContentRegionAvail().x - udpPortControlWidth - style.ItemSpacing.x));
                if (ImGui::InputText("##UDPAddress", self->udpHost, sizeof(self->udpHost))) {
                    self->udpRestartRequested.store(true);
                    settingsChanged = true;
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(udpPortControlWidth);
                if (ImGui::InputInt("##UDPPort", &self->udpPort)) {
                    self->udpPort = (std::max)(1, (std::min)(65535, self->udpPort));
                    self->udpRestartRequested.store(true);
                    settingsChanged = true;
                }
            }
            ImGui::Separator();
            ImGui::TextUnformatted("Player command:");
            {
                std::lock_guard<std::mutex> lock(self->udpConfigMutex);
                ImGui::SetNextItemWidth(-1.0f);
                const float commandHeight = ImGui::GetTextLineHeightWithSpacing() * 3.0f +
                                            style.FramePadding.y * 2.0f;
                if (ImGui::InputTextMultiline("##PlayerCommand", self->playerCommand,
                                              sizeof(self->playerCommand), ImVec2(-1.0f, commandHeight)))
                    settingsChanged = true;
            }
            ImGui::TextDisabled("$ip and $port variables can be used");
            ImGui::Separator();
            ImGui::TextUnformatted("Record TS path:");
            ImGui::SameLine();
            {
                std::lock_guard<std::mutex> lock(self->recordingConfigMutex);
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputText("##SelectedRecordingPath", self->selectedRecordingPath,
                                     sizeof(self->selectedRecordingPath))) {
                    self->selectedRecordingRestartRequested.store(true);
                    settingsChanged = true;
                }
            }
            ImGui::TextUnformatted("Record MUX path:");
            ImGui::SameLine();
            {
                std::lock_guard<std::mutex> lock(self->recordingConfigMutex);
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputText("##FullRecordingPath", self->fullRecordingPath,
                                     sizeof(self->fullRecordingPath))) {
                    self->fullRecordingRestartRequested.store(true);
                    settingsChanged = true;
                }
            }
            bool appendTimestamp = self->appendRecordingTimestamp.load();
            if (ImGui::Checkbox("Append date and time to recording filenames", &appendTimestamp)) {
                self->appendRecordingTimestamp.store(appendTimestamp);
                self->selectedRecordingRestartRequested.store(true);
                self->fullRecordingRestartRequested.store(true);
                settingsChanged = true;
            }
            if (self->udpRestartRequested.load()) self->udpQueueCv.notify_one();
            if (settingsChanged) self->saveUiConfiguration();
            ImGui::Separator();
        }

        const bool locked = self->p1Locked.load();
        bool qamGraphDrawn = false;
        bool infoSeparatorDrawn = false;
        if (!showAdvancedInfo && self->displayMode != 3) {
            ImGui::Text("Sync: P1 %s, P2 %s, data pilots %s",
                        locked ? "ACQUIRED" : "searching",
                        self->p2Synchronized.load() ? "ACQUIRED" : "searching",
                        self->dataSymbolSynchronized.load() ? "ACQUIRED" : "searching");
            if (self->softQamValues.load() > 0) {
                ImGui::Text("Signal: %.1f dBFS, SNR %.1f dB, pilot correlation %.1f%%",
                            self->levelDb.load(), self->qamSnrDb.load(),
                            self->workerMeanPilotCorrelation.load() * 100.0f);
            }
            else {
                ImGui::Text("Signal: %.1f dBFS, SNR pending, P1 correlation %.1f%%",
                            self->levelDb.load(), self->correlation.load() * 100.0f);
            }
            ImGui::Text("Worker: average/max %.0f / %.0f ms, queue %d/%d, %llu skipped",
                        self->workerFrameAverageMilliseconds.load(), self->workerFrameMaximumMilliseconds.load(),
                        self->workerQueueDepth.load(), FRAME_QUEUE_CAPACITY,
                        static_cast<unsigned long long>(self->workerFramesSkipped.load()));
            ImGui::Text("FEC: LDPC %llu / %llu converged, %.0f unsatisfied checks",
                        static_cast<unsigned long long>(self->ldpcBlocksConverged.load()),
                        static_cast<unsigned long long>(self->ldpcBlocksTested.load()),
                        self->ldpcMeanUnsatisfied.load());
            ImGui::Text("TS: %llu packets, %llu resyncs; UDP %llu errors, %llu dropped",
                        static_cast<unsigned long long>(self->tsPacketsWritten.load()),
                        static_cast<unsigned long long>(self->tsResyncs.load()),
                        static_cast<unsigned long long>(self->udpErrors.load()),
                        static_cast<unsigned long long>(self->udpDatagramsDropped.load()));
            ImGui::Separator();
            infoSeparatorDrawn = true;
            if (showGraph) {
                self->drawQamConstellation();
                ImGui::Separator();
                qamGraphDrawn = true;
            }
            drawSelectedOutputStatus();
        }

        float infoStartY = ImGui::GetCursorPosY();
        bool infoClipActive = !showAdvancedInfo;
        if (infoClipActive) {
            const ImVec2 clipPoint = ImGui::GetCursorScreenPos();
            ImGui::PushClipRect(clipPoint, clipPoint, false);
        }
        ImGui::Text("Channel: 8 MHz   VFO rate: %.1f MS/s", INPUT_SAMPLE_RATE / 1.0e6);
        ImGui::Text("Input level: %.1f dBFS", self->levelDb.load());

        ImGui::TextUnformatted("P1 synchronization:");
        ImGui::SameLine();
        if (locked) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "ACQUIRED");
        else ImGui::TextUnformatted("searching");
        ImGui::Text("P1 correlation: %.1f%%", self->correlation.load() * 100.0f);
        ImGui::Text("P1 frames: %llu verified, %llu timing-recovered",
                    static_cast<unsigned long long>(self->p1Count.load()),
                    static_cast<unsigned long long>(self->p1PredictedCaptures.load()));
        if (locked) ImGui::Text("Coarse frequency error: %+.0f Hz", self->coarseOffset.load());
        if (self->decodedPreamble.load() >= 0) {
            static const char* preambles[] = {"T2 SISO", "T2 MISO", "Non-T2", "T2-Lite SISO", "T2-Lite MISO"};
            static const char* fftModes[] = {"2K", "8K", "4K", "1K", "16K", "32K", "8K T2-GI", "32K T2-GI"};
            ImGui::Text("Preamble: %s", preambles[self->decodedPreamble.load()]);
            ImGui::Text("FFT mode: %s", fftModes[self->decodedFft.load()]);
            ImGui::Text("Preamble sequence: %s", self->decodedMixed.load() ? "mixed" : "not mixed");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("P2 synchronization:");
        ImGui::SameLine();
        if (self->p2Synchronized.load()) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "ACQUIRED");
        else ImGui::TextUnformatted("searching");
        ImGui::Text("P2 cyclic correlation: %.1f%%", self->p2Correlation.load() * 100.0f);
        ImGui::Text("Verified P2 boundaries: %llu", static_cast<unsigned long long>(self->p2Count.load()));
        static const char* guardNames[] = {"1/128", "1/32", "1/16", "19/256", "1/8", "19/128", "1/4"};
        if (self->guardInterval.load() >= 0) ImGui::Text("Guard interval: %s", guardNames[self->guardInterval.load()]);
        ImGui::TextUnformatted("L1-pre CRC:");
        ImGui::SameLine();
        if (self->l1LiveValid.load()) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "OK");
        else if (self->l1PreValid.load()) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "OK (last valid held)");
        else ImGui::TextUnformatted("pending");
        ImGui::Text("L1 valid decodes: %llu / %llu",
                    static_cast<unsigned long long>(self->l1CrcValidFrames.load()),
                    static_cast<unsigned long long>(self->l1CrcAttempts.load()));
        ImGui::Text("L1 BPSK quality: %.1f%%", self->l1DecisionQuality.load() * 100.0f);
        if (self->l1PreValid.load()) {
            static const char* postModNames[] = {"BPSK", "QPSK", "16-QAM", "64-QAM"};
            ImGui::Text("Pilot pattern: PP%d", self->l1PilotPattern.load() + 1);
            ImGui::Text("L1 carrier alignment: %+d bins", self->l1CarrierShift.load());
            ImGui::Text("Spectrum orientation: %s", self->l1SpectrumMirrored.load() ? "mirrored" : "normal");
            ImGui::Text("Cell / network / system ID: %04X / %04X / %04X",
                        self->l1CellId.load(), self->l1NetworkId.load(), self->l1SystemId.load());
            ImGui::Text("T2 frames: %d   Data symbols: %d", self->l1Frames.load(), self->l1DataSymbols.load());
            const int postMod = self->l1PostMod.load();
            if (postMod >= 0 && postMod < 4) {
                ImGui::Text("L1-post: %s, %d cells, %d info bits", postModNames[postMod],
                            self->l1PostSize.load(), self->l1PostInfoSize.load());
            }
            ImGui::Text("L1-post CRC: %s  (%llu / %llu)", self->l1PostValid.load() ? "OK" : "pending",
                        static_cast<unsigned long long>(self->l1PostValidFrames.load()),
                        static_cast<unsigned long long>(self->l1PostAttempts.load()));
            if (self->l1PostValid.load()) {
                ImGui::Text("Physical-layer pipes: %d", self->l1NumPlps.load());
                const int count = (std::min)(self->l1NumPlps.load(), 8);
                for (int i = 0; i < count; ++i) {
                    ImGui::Text("PLP %d: ID %d  type %d  mod %d  code %d  FEC %d", i + 1,
                                self->plpIds[i].load(), self->plpTypes[i].load(), self->plpMods[i].load(),
                                self->plpCodes[i].load(), self->plpFecTypes[i].load());
                }
                if (self->l1DynamicValid.load()) {
                    ImGui::Text("L1-dynamic frame: %d", self->l1DynamicFrame.load());
                    ImGui::Text("PLP 0 allocation: start %d, blocks %d", self->dynamicPlpStart.load(),
                                self->dynamicPlpBlocks.load());
                    ImGui::Text("Time interleaver: type %d, length %d, max blocks %d",
                                self->plpTimeIlType.load(), self->plpTimeIlLength.load(),
                                self->plpNumBlocksMax.load());
                    ImGui::Text("Sub-slices per frame: %d", self->l1SubSlicesPerFrame.load());
                    ImGui::Text("PLP rotation: %s", self->plpRotations[0].load() ? "on" : "off");
                }
            }
            ImGui::TextUnformatted("Data-symbol pilots:");
            ImGui::SameLine();
            if (self->dataSymbolSynchronized.load()) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "ACQUIRED");
            else ImGui::TextUnformatted("searching");
            ImGui::Text("Data-pilot correlation: %.1f%%", self->dataPilotCorrelation.load() * 100.0f);
            ImGui::Text("Verified data symbols: %llu", static_cast<unsigned long long>(self->dataSymbolCount.load()));
            ImGui::Text("Full frames captured: %llu", static_cast<unsigned long long>(self->fullFrameCount.load()));
            if (self->fullFrameCount.load() > 0) {
                ImGui::Text("Frame boundaries: %d / %d", self->frameSymbolsVerified.load(), self->l1DataSymbols.load());
                ImGui::Text("Frame timing correction: %d samples", self->frameTimingCorrection.load());
                ImGui::Text("Mean data CP correlation: %.1f%%", self->frameMeanCpCorrelation.load() * 100.0f);
            }
            ImGui::Text("Frames FFT-processed: %llu", static_cast<unsigned long long>(self->workerFramesProcessed.load()));
            if (self->workerFramesProcessed.load() > 0) {
                ImGui::Text("Last full worker: %.0f ms total (OFDM %.0f, demap %.0f, LDPC %.0f)",
                            self->workerFrameMilliseconds.load(), self->workerOfdmMilliseconds.load(),
                            self->workerDemapMilliseconds.load(), self->workerLdpcMilliseconds.load());
                ImGui::Text("Worker average/max: %.0f / %.0f ms, %llu queued, %llu skipped",
                            self->workerFrameAverageMilliseconds.load(), self->workerFrameMaximumMilliseconds.load(),
                            static_cast<unsigned long long>(self->workerFramesQueued.load()),
                            static_cast<unsigned long long>(self->workerFramesSkipped.load()));
                ImGui::Text("Worker queue: %d / %d frames (peak %d)",
                            self->workerQueueDepth.load(), FRAME_QUEUE_CAPACITY,
                            self->workerQueuePeak.load());
                ImGui::Text("Compute workers: %d (%u logical CPU threads)",
                            self->computeWorkerLimit, self->logicalProcessorCount);
                ImGui::Text("OFDM detail: resample %.0f, FFT %.0f, equalize %.0f ms",
                            self->workerResampleMilliseconds.load(), self->workerFftMilliseconds.load(),
                            self->workerEqualizeMilliseconds.load());
                ImGui::Text("Pilot-equalized data symbols: %d / %d", self->workerSymbolsEqualized.load(), self->workerExpectedDataSymbols.load());
                ImGui::Text("Mean symbol-pilot correlation: %.1f%%", self->workerMeanPilotCorrelation.load() * 100.0f);
                ImGui::Text("Equalized data cells: %llu", static_cast<unsigned long long>(self->workerDataCells.load()));
                    ImGui::Text("Frame-cell sections: P2 %llu  data %llu  closing %llu",
                            static_cast<unsigned long long>(self->frameP2SectionCells.load()),
                            static_cast<unsigned long long>(self->frameDataSectionCells.load()),
                            static_cast<unsigned long long>(self->frameClosingSectionCells.load()));
                ImGui::Text("Worker L1-dynamic frame: %d", self->workerDynamicFrame.load());
                ImGui::Text("Frame-closing pilots: %s (%.1f%%)", self->frameClosingAcquired.load() ? "ACQUIRED" : "searching",
                            self->frameClosingPilotCorrelation.load() * 100.0f);
                if (self->frameClosingAcquired.load()) ImGui::Text("Frame-closing data cells: %d", self->frameClosingCells.load());
                ImGui::Text("PLP frames assembled: %llu", static_cast<unsigned long long>(self->plpFramesAssembled.load()));
                if (self->plpCellsRequired.load() > 0) {
                    ImGui::Text("PLP 0 cells: %llu / %llu required",
                                static_cast<unsigned long long>(self->plpCellsExtracted.load()),
                                static_cast<unsigned long long>(self->plpCellsRequired.load()));
                    ImGui::Text("Frame cells available: %llu", static_cast<unsigned long long>(self->plpCellsAvailable.load()));
                    if (self->fecBlocksReordered.load() > 0) {
                        ImGui::Text("Time-deinterleaved: %llu TI blocks, %llu FEC blocks",
                                    static_cast<unsigned long long>(self->tiBlocksCompleted.load()),
                                    static_cast<unsigned long long>(self->fecBlocksReordered.load()));
                        ImGui::Text("Raw soft QAM values: %llu   SNR: %.1f dB",
                                    static_cast<unsigned long long>(self->softQamValues.load()),
                                    self->qamSnrDb.load());
                        ImGui::Text("Bit-deinterleaved FEC blocks: %llu",
                                    static_cast<unsigned long long>(self->bitDeinterleavedBlocks.load()));
                        if (self->ldpcBlocksTested.load() > 0) {
                        ImGui::Text("LDPC convergence: %llu / %llu blocks",
                                    static_cast<unsigned long long>(self->ldpcBlocksConverged.load()),
                                    static_cast<unsigned long long>(self->ldpcBlocksTested.load()));
                        ImGui::Text("LDPC alternate: %llu / %llu residual blocks recovered",
                                    static_cast<unsigned long long>(self->ldpcAlternateRecovered.load()),
                                    static_cast<unsigned long long>(self->ldpcAlternateAttempts.load()));
                        ImGui::Text("LDPC half-scale: %llu / %llu residual blocks recovered",
                                    static_cast<unsigned long long>(self->ldpcHalfScaleRecovered.load()),
                                    static_cast<unsigned long long>(self->ldpcHalfScaleAttempts.load()));
                        ImGui::Text("LDPC quarter-scale: %llu / %llu residual blocks recovered",
                                    static_cast<unsigned long long>(self->ldpcQuarterScaleRecovered.load()),
                                    static_cast<unsigned long long>(self->ldpcQuarterScaleAttempts.load()));
                        ImGui::Text("BCH recovery: %llu / %llu blocks, %llu bits corrected",
                                    static_cast<unsigned long long>(self->bchRecoveredBlocks.load()),
                                    static_cast<unsigned long long>(self->bchRecoveryAttempts.load()),
                                    static_cast<unsigned long long>(self->bchCorrectedBits.load()));
                        if (self->residualFecCapturesAttempted.load() > 0) {
                            ImGui::Text("Residual FEC captures: %d / %d saved",
                                        self->residualFecCapturesSaved.load(),
                                        self->residualFecCapturesAttempted.load());
                        }
                            ImGui::Text("LDPC block range: %d..%d", self->ldpcFirstBlockTested.load(),
                                        self->ldpcFirstBlockTested.load() +
                                        static_cast<int>(self->ldpcBlocksTested.load()) - 1);
                            ImGui::Text("LDPC checks after decode: %.0f / 21600",
                                         self->ldpcMeanUnsatisfied.load());
                            ImGui::Text("BBHEADER CRC: %llu / %llu valid",
                                        static_cast<unsigned long long>(self->bbHeadersValid.load()),
                                        static_cast<unsigned long long>(self->bbHeadersTested.load()));
                            if (self->bbHeadersValid.load() > 0) {
                                ImGui::Text("BBFRAME: mode %s  TS/GS %d  NPD %s  UPL %d  DFL %d  SYNC 0x%02X  SYNCD %d",
                                            self->bbInputMode.load() == 0 ? "normal" : "high-efficiency",
                                            self->bbTsGs.load(), self->bbNpd.load() ? "on" : "off",
                                            self->bbUpl.load(), self->bbDfl.load(),
                                            self->bbSync.load(), self->bbSyncd.load());
                                ImGui::Text("TS output: %llu packets, %llu bytes, %llu resyncs, %llu nulls restored",
                                            static_cast<unsigned long long>(self->tsPacketsWritten.load()),
                                            static_cast<unsigned long long>(self->tsBytesWritten.load()),
                                            static_cast<unsigned long long>(self->tsResyncs.load()),
                                            static_cast<unsigned long long>(self->tsNullPacketsReinserted.load()));
                                ImGui::Separator();
                                infoSeparatorDrawn = true;
                                if (showGraph) {
                                    self->drawQamConstellation();
                                    ImGui::Separator();
                                    qamGraphDrawn = true;
                                }
                                if (showAdvancedInfo || self->displayMode == 3) {
                                    drawSelectedOutputStatus();
                                }
                                if (infoClipActive) {
                                    ImGui::PopClipRect();
                                    ImGui::SetCursorPosY(infoStartY);
                                    infoClipActive = false;
                                }
                                std::vector<TsServiceInfo> serviceSnapshot;
                                {
                                    std::lock_guard<std::mutex> lock(self->tsServiceMutex);
                                    for (const auto& entry : self->tsServices) serviceSnapshot.push_back(entry.second);
                                }
                                if (!serviceSnapshot.empty()) {
                                    const int selectedId = self->selectedTsService.load();
                                    std::string selectedLabel = "Select a service";
                                    for (const auto& service : serviceSnapshot) {
                                        if (service.serviceId == selectedId) {
                                            selectedLabel = service.name.empty() ?
                                                ("Service " + std::to_string(service.serviceId)) : service.name;
                                            break;
                                        }
                                    }
                                    const char* udpStreamButtonText = self->udpStreamingEnabled.load() ?
                                        "Stop Stream" : "Start Stream";
                                    const char* playerLinkText = "Play";
                                    const float udpStreamButtonWidth = ImGui::CalcTextSize(udpStreamButtonText).x +
                                                                       style.FramePadding.x * 2.0f;
                                    const float playerLinkWidth = ImGui::CalcTextSize(playerLinkText).x;
                                    const float udpStateWidth = ImGui::CalcTextSize("●").x;
                                    ImGui::AlignTextToFramePadding();
                                    ImGui::TextUnformatted("TS:");
                                    ImGui::SameLine();
                                    ImGui::SetNextItemWidth((std::max)(1.0f,
                                        ImGui::GetContentRegionAvail().x - udpStreamButtonWidth -
                                        playerLinkWidth - udpStateWidth - style.ItemSpacing.x * 3.0f));
                                    if (ImGui::BeginCombo("##SelectedTSService", selectedLabel.c_str())) {
                                        for (const auto& service : serviceSnapshot) {
                                            const std::string label = service.name.empty() ?
                                                ("Service " + std::to_string(service.serviceId)) : service.name;
                                            const bool selected = service.serviceId == selectedId;
                                            if (ImGui::Selectable(label.c_str(), selected)) {
                                                self->selectedTsService.store(service.serviceId);
                                                self->selectedPatVersion.fetch_add(1);
                                                self->selectedRecordingRestartRequested.store(true);
                                                self->udpPacketResetRequested.store(true);
                                            }
                                            if (selected) ImGui::SetItemDefaultFocus();
                                        }
                                        ImGui::EndCombo();
                                    }
                                    ImGui::SameLine();
                                    const bool udpActive = self->udpStreamingEnabled.load();
                                    const bool udpFailed = self->udpErrors.load() > 0;
                                    const ImVec4 udpStateColor = udpFailed ? ImVec4(1.0f, 0.25f, 0.2f, 1.0f) :
                                        (udpActive ? ImVec4(0.2f, 1.0f, 0.3f, 1.0f) :
                                                     ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
                                    ImGui::TextColored(udpStateColor, "●");
                                    if (ImGui::IsItemHovered())
                                        ImGui::SetTooltip(udpFailed ? "UDP stream has reported errors" :
                                            (udpActive ? "UDP stream active" : "UDP stream stopped"));
                                    ImGui::SameLine();
                                    if (ImGui::Button(udpStreamButtonText)) {
                                        self->udpStreamingEnabled.store(!self->udpStreamingEnabled.load());
                                        self->udpRestartRequested.store(true);
                                        self->udpQueueCv.notify_one();
                                    }
                                    ImGui::SameLine();
                                    const ImVec2 playerLinkSize = ImGui::CalcTextSize(playerLinkText);
                                    const ImVec2 playerLinkItemPos = ImGui::GetCursorScreenPos();
                                    ImGui::InvisibleButton("##OpenPlayer", ImVec2(playerLinkSize.x, ImGui::GetFrameHeight()));
                                    const bool playerLinkHovered = ImGui::IsItemHovered();
                                    const ImVec2 playerLinkTextPos(playerLinkItemPos.x,
                                        playerLinkItemPos.y + (ImGui::GetFrameHeight() - playerLinkSize.y) * 0.5f);
                                    const ImU32 playerLinkColor = ImGui::GetColorU32(ImGuiCol_Text);
                                    ImGui::GetWindowDrawList()->AddText(playerLinkTextPos, playerLinkColor, playerLinkText);
                                    ImGui::GetWindowDrawList()->AddLine(
                                        ImVec2(playerLinkTextPos.x, playerLinkTextPos.y + playerLinkSize.y),
                                        ImVec2(playerLinkTextPos.x + playerLinkSize.x, playerLinkTextPos.y + playerLinkSize.y),
                                        playerLinkColor);
                                    if (playerLinkHovered) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                    if (ImGui::IsItemClicked()) self->openPlayer();
                                    if (self->playerLaunchStatus.load() < 0)
                                        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.25f, 1.0f),
                                                           "Player could not be opened; check its path in Settings.");
                                    ImGui::Separator();
                                    if (ImGui::Button(self->selectedRecordingEnabled.load() ?
                                                      "Stop TS" : "Record TS")) {
                                        self->selectedRecordingEnabled.store(!self->selectedRecordingEnabled.load());
                                        self->selectedRecordingRestartRequested.store(true);
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::Button(self->fullRecordingEnabled.load() ?
                                                      "Stop MUX" : "Record MUX")) {
                                        self->fullRecordingEnabled.store(!self->fullRecordingEnabled.load());
                                        self->fullRecordingRestartRequested.store(true);
                                    }
                                    ImGui::SameLine();
                                    ImGui::Text("TS: %llu packets / MUX: %llu packets",
                                                static_cast<unsigned long long>(self->selectedRecordedPackets.load()),
                                                static_cast<unsigned long long>(self->fullRecordedPackets.load()));
                                    ImGui::Separator();
                                    auto streamTypeName = [](int type) {
                                        switch (type) {
                                            case 0x02: return "MPEG-2 video";
                                            case 0x03: return "MPEG-1 audio";
                                            case 0x04: return "MPEG-2 audio";
                                            case 0x0F: return "AAC audio";
                                            case 0x11: return "AAC LATM";
                                            case 0x1B: return "H.264 video";
                                            case 0x24: return "HEVC video";
                                            case 0x06: return "private data";
                                            case 0x81: return "AC-3 audio";
                                            default: return "other";
                                        }
                                    };
                                    if (ImGui::TreeNodeEx("##TransportServices",
                                            0,
                                            "Transport services: %d",
                                            static_cast<int>(serviceSnapshot.size()))) {
                                        for (const auto& service : serviceSnapshot) {
                                            const std::string label = service.name.empty() ?
                                                ("Service " + std::to_string(service.serviceId)) : service.name;
                                            const void* treeId = reinterpret_cast<const void*>(
                                                static_cast<intptr_t>(service.serviceId));
                                            if (ImGui::TreeNode(treeId, "%s  (SID %d)", label.c_str(), service.serviceId)) {
                                                ImGui::Text("PMT PID %d   PCR PID %d", service.pmtPid, service.pcrPid);
                                                if (!service.provider.empty())
                                                    ImGui::Text("Provider: %s", service.provider.c_str());
                                                for (const auto& stream : service.streams)
                                                    ImGui::Text("PID %d: %s (0x%02X)", stream.pid,
                                                                streamTypeName(stream.type), stream.type);
                                                ImGui::TreePop();
                                            }
                                        }
                                        ImGui::TreePop();
                                    }
                                }
                                if (!showAdvancedInfo) {
                                    infoStartY = ImGui::GetCursorPosY();
                                    const ImVec2 clipPoint = ImGui::GetCursorScreenPos();
                                    ImGui::PushClipRect(clipPoint, clipPoint, false);
                                    infoClipActive = true;
                                }
                                ImGui::Text("TS resync location: first %llu, TI boundary %llu, other %llu",
                                            static_cast<unsigned long long>(self->tsResyncFirstBlock.load()),
                                            static_cast<unsigned long long>(self->tsResyncTiBoundary.load()),
                                            static_cast<unsigned long long>(self->tsResyncOtherBlock.load()));
                                if (self->tsLastResyncBlock.load() >= 0) {
                                    ImGui::Text("Last TS mismatch: block %d, SYNCD %d bytes, expected %d",
                                                self->tsLastResyncBlock.load(), self->tsLastActualSyncBytes.load(),
                                                self->tsLastExpectedSyncBytes.load());
                                }
                                std::array<int, 3> topBlocks{-1, -1, -1};
                                std::array<uint64_t, 3> topCounts{};
                                for (int block = 0; block < TS_RESYNC_BLOCK_BUCKETS; ++block) {
                                    const uint64_t count = self->tsResyncByBlock[block].load();
                                    for (int rank = 0; rank < 3; ++rank) {
                                        if (count > topCounts[rank]) {
                                            for (int move = 2; move > rank; --move) {
                                                topCounts[move] = topCounts[move - 1];
                                                topBlocks[move] = topBlocks[move - 1];
                                            }
                                            topCounts[rank] = count;
                                            topBlocks[rank] = block;
                                            break;
                                        }
                                    }
                                }
                                if (topBlocks[0] >= 0) {
                                    ImGui::Text("TS resync hot blocks: %d:%llu  %d:%llu  %d:%llu",
                                                topBlocks[0], static_cast<unsigned long long>(topCounts[0]),
                                                topBlocks[1], static_cast<unsigned long long>(topCounts[1]),
                                                topBlocks[2], static_cast<unsigned long long>(topCounts[2]));
                                }
                                ImGui::Text("TS mismatch phase: +1 BBFRAME %llu, -1 BBFRAME %llu, irregular %llu",
                                            static_cast<unsigned long long>(self->tsResyncForwardOne.load()),
                                            static_cast<unsigned long long>(self->tsResyncBackwardOne.load()),
                                            static_cast<unsigned long long>(self->tsResyncIrregular.load()));
                            }
                            const int selfTest = self->ldpcSelfTestStatus.load();
                            ImGui::Text("LDPC self-test: %s", selfTest > 0 ? "PASS" : (selfTest == 0 ? "FAIL" : "pending"));
                            ImGui::TextUnformatted("Chain: ETSI 256-QAM 2/3, direct frequency equalizer");
                        }
                    }
                }
            }
        }

        if (infoClipActive) {
            ImGui::PopClipRect();
            ImGui::SetCursorPosY(infoStartY);
        }
        if (showAdvancedInfo && !infoSeparatorDrawn) ImGui::Separator();
        if (showGraph && !qamGraphDrawn) {
            self->drawQamConstellation();
            ImGui::Separator();
        }
    }

    void processSample(const dsp::complex_t& sample) {
        if (resetRequested.exchange(false)) resetDetector();

        if (p2Collecting) {
            p2Capture[p2Fill++] = sample;
            if (p2Fill >= P2_CAPTURE_SAMPLES) {
                p2Collecting = false;
                analyzeP2Capture();
            }
        }
        if (fullFrameCollecting && fullFrameFill < MAX_FRAME_CAPTURE_SAMPLES) {
            fullFrameCapture[fullFrameFill++] = sample;
            if (fullFrameTarget > 0 && fullFrameFill >= fullFrameTarget) {
                fullFrameCollecting = false;
                analyzeFullFrameCapture();
            }
            else if (fullFrameFill >= MAX_FRAME_CAPTURE_SAMPLES && fullFrameTarget == 0) {
                fullFrameCollecting = false;
            }
        }

        captureRing[capturePos] = sample;
        capturePos = (capturePos + 1) % captureRing.size();

        constexpr float oscillatorStepRe = 0.99999901699f;
        constexpr float oscillatorStepIm = 0.00140249626f;
        const dsp::complex_t shifted{sample.re * oscillatorRe - sample.im * oscillatorIm,
                                     sample.re * oscillatorIm + sample.im * oscillatorRe};
        const float nextRe = oscillatorRe * oscillatorStepRe - oscillatorIm * oscillatorStepIm;
        oscillatorIm = oscillatorRe * oscillatorStepIm + oscillatorIm * oscillatorStepRe;
        oscillatorRe = nextRe;
        if ((++oscillatorSamples & 0xFFFu) == 0) {
            const float inverseLength = 1.0f / std::sqrt(oscillatorRe * oscillatorRe + oscillatorIm * oscillatorIm);
            oscillatorRe *= inverseLength;
            oscillatorIm *= inverseLength;
        }

        const dsp::complex_t old = delayed[delayPos];
        delayed[delayPos] = sample;
        delayPos = (delayPos + 1) % P1_B;

        const dsp::complex_t product = multiplyConjugate(shifted, old);
        const dsp::complex_t expired = products[productPos];
        products[productPos] = product;
        sumProduct.re += product.re - expired.re;
        sumProduct.im += product.im - expired.im;

        const float samplePower = magnitudeSquared(sample);
        const float oldPower = magnitudeSquared(old);
        sumCurrentEnergy += samplePower - currentEnergy[productPos];
        sumDelayedEnergy += oldPower - delayedEnergy[productPos];
        currentEnergy[productPos] = samplePower;
        delayedEnergy[productPos] = oldPower;
        productPos = (productPos + 1) % P1_B;

        ++samplesSeen;
        if ((samplesSeen & 0x7FFu) == 0) {
            const float rms = static_cast<float>(std::sqrt((std::max)(sumCurrentEnergy / P1_B, 1.0e-12)));
            const float db = 20.0f * std::log10((std::max)(rms, 1.0e-6f));
            levelDb.store(0.92f * levelDb.load() + 0.08f * db);
        }

        if (samplesSeen < static_cast<uint64_t>(P1_B * 2)) return;
        const double denom = std::sqrt((std::max)(sumCurrentEnergy * sumDelayedEnergy, 1.0e-20));
        const float metric = static_cast<float>(std::sqrt(magnitudeSquared(sumProduct)) / denom);
        displayedMetric = (std::max)(metric, displayedMetric * 0.99995f);
        if ((samplesSeen & 0xFFFu) == 0) correlation.store(displayedMetric);

        if (metric >= threshold.load() && levelDb.load() > -75.0f) {
            ++candidateSamples;
            if (metric > peakMetric) {
                peakMetric = metric;
                peakArgument = sumProduct;
            }
        }
        else if (candidateSamples > 80) {
            registerCandidate();
            candidateSamples = 0;
            peakMetric = 0.0f;
        }
        else {
            candidateSamples = 0;
            peakMetric = 0.0f;
        }

        // Once the repeating P1 period is locked, start the next capture from
        // that timing even when the strict P1 payload validator misses a faded
        // preamble.  P2 correlation and its CRC-protected L1 signalling still
        // independently validate every recovered frame before it is decoded.
        if (p1Locked.load() && nextPredictedFrameSample != 0 &&
            samplesSeen >= nextPredictedFrameSample && expectedFrameSamples > 0.0) {
            bool started = false;
            if (!p2Collecting) {
                p2Fill = 0;
                p2Collecting = true;
                started = true;
            }
            if (!fullFrameCollecting || fullFrameTarget == 0) {
                fullFrameFill = 0;
                fullFrameTarget = 0;
                fullFrameCollecting = true;
                started = true;
            }
            if (started) p1PredictedCaptures.fetch_add(1);
            const uint64_t period = static_cast<uint64_t>(std::llround(expectedFrameSamples));
            do { nextPredictedFrameSample += (std::max)(period, uint64_t{1}); }
            while (nextPredictedFrameSample <= samplesSeen);
        }

        // P1 payload validation is intentionally strict, so an otherwise good
        // multiplex may lose several individual frames to fading.  Acquisition
        // has already been proven by exact S1/S2 payload and P2-boundary checks;
        // use a five-second floor for the UI lock hold to prevent brief gaps
        // from presenting as a complete loss of synchronization.
        const uint64_t lockTimeout = expectedFrameSamples > 0.0
            ? static_cast<uint64_t>((std::max)(expectedFrameSamples * 12.0, INPUT_SAMPLE_RATE * 5.0))
            : static_cast<uint64_t>(INPUT_SAMPLE_RATE * 5.0);
        if (p1Locked.load() && samplesSeen - lastP1Sample > lockTimeout) {
            p1Locked.store(false);
        }
    }

    void registerCandidate() {
        // A DVB-T2 P1 occurs once per T2 frame. Reject isolated and implausibly
        // frequent correlation spikes, then require a stable repeating period.
        constexpr double minFrameSeconds = 0.020;
        constexpr double maxFrameSeconds = 1.000;
        const uint64_t minimum = static_cast<uint64_t>(INPUT_SAMPLE_RATE * minFrameSeconds);
        const uint64_t maximum = static_cast<uint64_t>(INPUT_SAMPLE_RATE * maxFrameSeconds);

        if (lastCandidateSample == 0) {
            lastCandidateSample = samplesSeen;
            return;
        }

        const uint64_t interval = samplesSeen - lastCandidateSample;
        if (interval < minimum) return;

        if (expectedFrameSamples == 0.0) {
            if (interval > maximum) {
                lastCandidateSample = samplesSeen;
                return;
            }
            lastCandidateSample = samplesSeen;
            expectedFrameSamples = static_cast<double>(interval);
            repeatConfirmations = 1;
            return;
        }

        // A weak secondary peak must not replace the frame timing anchor or
        // clear an established lock.  Also accept an integer number of frame
        // periods so that one or more faded P1 symbols do not break tracking.
        const int elapsedFrames = static_cast<int>(std::llround(static_cast<double>(interval) / expectedFrameSamples));
        const bool plausibleMultiple = elapsedFrames >= 1 && elapsedFrames <= 8;
        const double expectedInterval = expectedFrameSamples * (std::max)(elapsedFrames, 1);
        const double relativeError = std::abs(static_cast<double>(interval) - expectedInterval) / expectedInterval;
        if (relativeError <= 0.12) {
            if (!plausibleMultiple) return;
            lastCandidateSample = samplesSeen;
            const double measuredFrameSamples = static_cast<double>(interval) / elapsedFrames;
            expectedFrameSamples = expectedFrameSamples * 0.8 + measuredFrameSamples * 0.2;
            ++repeatConfirmations;
        }
        else {
            if (!p1Locked.load() && interval > maximum) {
                lastCandidateSample = samplesSeen;
                expectedFrameSamples = 0.0;
                repeatConfirmations = 0;
            }
            return;
        }

        const double angle = std::atan2(peakArgument.im, peakArgument.re);
        coarseOffset.store(static_cast<float>(angle * INPUT_SAMPLE_RATE / (2.0 * PI * P1_B)));
        if (repeatConfirmations >= 3 && decodeP1()) {
            lastP1Sample = samplesSeen;
            p1Count.fetch_add(1);
            p1Locked.store(true);
            nextPredictedFrameSample = samplesSeen +
                static_cast<uint64_t>(std::llround(expectedFrameSamples));
            if (!p2Collecting) {
                p2Fill = 0;
                p2Collecting = true;
            }
            if (!fullFrameCollecting || fullFrameTarget == 0) {
                fullFrameFill = 0;
                fullFrameTarget = 0;
                fullFrameCollecting = true;
            }
        }
    }

    void analyzeP2Capture() {
        // 32K useful-symbol length at the 10 MHz VFO boundary.
        constexpr int fftLag = 35840;
        constexpr int searchSamples = 2600;
        static constexpr int guardSamples[] = {280, 1120, 2240, 2660, 4480, 5320, 8960};

        float bestAccepted = 0.0f;
        double bestGuardScore = 0.0;
        int bestGuard = -1;
        int bestBoundaryStart = -1;
        dsp::complex_t bestPrefixCorrelation{0.0f, 0.0f};
        for (int guardId = 0; guardId < 7; ++guardId) {
            const int window = guardSamples[guardId];
            dsp::complex_t correlationSum{0.0f, 0.0f};
            double energyA = 0.0;
            double energyB = 0.0;
            for (int i = 0; i < window; ++i) {
                const auto& a = p2Capture[i];
                const auto& b = p2Capture[i + fftLag];
                const auto product = multiplyConjugate(a, b);
                correlationSum.re += product.re;
                correlationSum.im += product.im;
                energyA += magnitudeSquared(a);
                energyB += magnitudeSquared(b);
            }

            float bestForGuard = 0.0f;
            int bestStartForGuard = 0;
            dsp::complex_t bestCorrelationForGuard{0.0f, 0.0f};
            for (int start = 0; start < searchSamples; ++start) {
                const double denominator = std::sqrt((std::max)(energyA * energyB, 1.0e-20));
                const float metric = static_cast<float>(std::sqrt(magnitudeSquared(correlationSum)) / denominator);
                if (metric > bestForGuard) {
                    bestForGuard = metric;
                    bestStartForGuard = start;
                    bestCorrelationForGuard = correlationSum;
                }

                const auto& oldA = p2Capture[start];
                const auto& oldB = p2Capture[start + fftLag];
                const auto& newA = p2Capture[start + window];
                const auto& newB = p2Capture[start + window + fftLag];
                const auto oldProduct = multiplyConjugate(oldA, oldB);
                const auto newProduct = multiplyConjugate(newA, newB);
                correlationSum.re += newProduct.re - oldProduct.re;
                correlationSum.im += newProduct.im - oldProduct.im;
                energyA += magnitudeSquared(newA) - magnitudeSquared(oldA);
                energyB += magnitudeSquared(newB) - magnitudeSquared(oldB);
            }

            // For windows shorter than the true prefix the normalized metric
            // remains high; beyond it the metric falls approximately as G/W.
            // metric*sqrt(W) therefore peaks close to the actual prefix length.
            const double guardScore = static_cast<double>(bestForGuard) * std::sqrt(static_cast<double>(window));
            if (bestForGuard >= 0.25f && guardScore > bestGuardScore) {
                bestGuard = guardId;
                bestAccepted = bestForGuard;
                bestGuardScore = guardScore;
                bestBoundaryStart = bestStartForGuard;
                bestPrefixCorrelation = bestCorrelationForGuard;
            }
        }

        p2Correlation.store(bestAccepted);
        if (bestGuard >= 0) {
            currentP2PayloadReady = false;
            guardInterval.store(bestGuard);
            p2Synchronized.store(true);
            p2Count.fetch_add(1);
            const double subcarrierSpacing = DVB_SAMPLE_RATE / 32768.0;
            const double integerOffset = std::round(coarseOffset.load() / subcarrierSpacing) * subcarrierSpacing;
            const double prefixPhase = std::atan2(bestPrefixCorrelation.im, bestPrefixCorrelation.re);
            const double fractionalOffset = -prefixPhase * INPUT_SAMPLE_RATE / (2.0 * PI * fftLag);
            // bestBoundaryStart is the prefix start of P2 symbol 0.  Data begins
            // only after all N_P2 P2 symbols, not unconditionally after one.
            currentFirstDataUsefulStart = bestBoundaryStart +
                dvbt2Params.n_p2 * (fftLag + guardSamples[bestGuard]) + guardSamples[bestGuard];
            currentFrameFrequency = integerOffset + fractionalOffset;
            decodeL1Pre(bestBoundaryStart + guardSamples[bestGuard], integerOffset + fractionalOffset);
            const int dataSymbols = l1DataSymbols.load();
            if (fullFrameCollecting && dataSymbols > 0) {
                const int symbolLength = fftLag + guardSamples[bestGuard];
                fullFrameP2Boundary = bestBoundaryStart;
                fullFrameGuardSamples = guardSamples[bestGuard];
                fullFrameTarget = (std::min)(bestBoundaryStart +
                                             (dvbt2Params.n_p2 + dataSymbols) * symbolLength + 1,
                                             MAX_FRAME_CAPTURE_SAMPLES);
                if (currentP2PayloadReady && l1DynamicValid.load()) {
                    fullFrameP2Payload = currentP2Payload;
                    fullFrameSnapshotSymbols = dataSymbols;
                    fullFrameSnapshotFrequency = currentFrameFrequency;
                    fullFrameSnapshotCarrierShift = l1CarrierShift.load();
                    fullFrameSnapshotDynamicFrame = l1DynamicFrame.load();
                    fullFrameSnapshotPlpStart = dynamicPlpStart.load();
                    fullFrameSnapshotPlpBlocks = dynamicPlpBlocks.load();
                    fullFrameSnapshotPlpModulation = plpMods[0].load();
                    fullFrameSnapshotPlpCode = plpCodes[0].load();
                    fullFrameSnapshotPlpFecType = plpFecTypes[0].load();
                    fullFrameSnapshotPlpRotation = plpRotations[0].load();
                    fullFrameSnapshotMaxBlocks = plpNumBlocksMax.load();
                    fullFrameSnapshotTimeLength = plpTimeIlLength.load();
                    fullFrameSnapshotTimeType = plpTimeIlType.load();
                    fullFrameSnapshotReady = true;
                }
            }
        }
        else {
            p2Synchronized.store(false);
            l1LiveValid.store(false);
        }
    }

    static uint32_t readBits(const std::array<uint8_t, 200>& bits, int first, int count) {
        uint32_t value = 0;
        for (int i = 0; i < count; ++i) value = (value << 1) | bits[first + i];
        return value;
    }

    static uint32_t readVectorBits(const std::vector<uint8_t>& bits, int first, int count) {
        uint32_t value = 0;
        for (int i = 0; i < count; ++i) value = (value << 1) | bits[first + i];
        return value;
    }

    void decodeL1Post(const std::vector<dsp::complex_t>& cells) {
        constexpr int firstPostCell = 1840;
        const int modulation = l1PostMod.load();
        const int postCells = l1PostSize.load();
        const int infoBits = l1PostInfoSize.load();
        if (modulation < 0 || modulation > 3 || postCells <= 0 || infoBits <= 0) return;
        const int bitsPerCell = 1 << modulation;
        const int bitCount = postCells * bitsPerCell;
        if (firstPostCell + postCells > static_cast<int>(cells.size()) || infoBits + 32 > bitCount) return;
        l1PostAttempts.fetch_add(1);

        double powerSum = 0.0;
        for (int i = 0; i < postCells; ++i) powerSum += magnitudeSquared(cells[firstPostCell + i]);
        const float scale = static_cast<float>(1.0 / std::sqrt((std::max)(powerSum / postCells, 1.0e-20)));
        std::vector<uint8_t> multiplexed(bitCount, 0);
        static constexpr int mux16Local[8] = {7, 1, 3, 5, 2, 4, 6, 0};
        static constexpr int mux64Local[12] = {11, 8, 5, 2, 10, 7, 4, 1, 9, 6, 3, 0};
        int rows = 0;
        int columns = 0;
        int substreams = 1;
        const int* mux = nullptr;
        int bpskMux[1] = {0};
        if (modulation <= 1) mux = bpskMux;
        else if (modulation == 2) { columns = 8; rows = bitCount / columns; substreams = 8; mux = mux16Local; }
        else { columns = 12; rows = bitCount / columns; substreams = 12; mux = mux64Local; }

        int outputOffset = 0;
        int muxIndex = 0;
        for (int bitIndex = 0; bitIndex < bitCount; ++bitIndex) {
            const auto& raw = cells[firstPostCell + bitIndex / bitsPerCell];
            const float re = raw.re * scale;
            const float im = raw.im * scale;
            const int plane = bitIndex % bitsPerCell;
            uint8_t bit = 0;
            if (plane == 0) bit = re > 0.0f ? 0 : 1;
            else if (plane == 1) bit = im > 0.0f ? 0 : 1;
            else if (plane == 2) bit = std::abs(re) > (modulation == 2 ? NORM_FACTOR_QAM16 * 2.0f : NORM_FACTOR_QAM64 * 4.0f) ? 0 : 1;
            else if (plane == 3) bit = std::abs(im) > (modulation == 2 ? NORM_FACTOR_QAM16 * 2.0f : NORM_FACTOR_QAM64 * 4.0f) ? 0 : 1;
            else if (plane == 4) bit = std::abs(std::abs(re) - NORM_FACTOR_QAM64 * 4.0f) > NORM_FACTOR_QAM64 * 2.0f ? 0 : 1;
            else bit = std::abs(std::abs(im) - NORM_FACTOR_QAM64 * 4.0f) > NORM_FACTOR_QAM64 * 2.0f ? 0 : 1;
            multiplexed[mux[muxIndex] + outputOffset] = bit;
            if (++muxIndex == substreams) { muxIndex = 0; outputOffset += substreams; }
        }

        std::vector<uint8_t> decoded(bitCount, 0);
        std::vector<uint8_t> randomBits(bitCount, 0);
        int randomizer = 0x4A80;
        for (int i = 0; i < bitCount; ++i) {
            const int randomBit = (randomizer ^ (randomizer >> 1)) & 1;
            randomBits[i] = static_cast<uint8_t>(randomBit);
            randomizer >>= 1;
            if (randomBit) randomizer |= 0x4000;
        }
        int column = 0;
        int step = 0;
        const int blockSize = rows * columns;
        for (int i = 0; i < bitCount; ++i) {
            const int destination = column + step;
            decoded[destination] = multiplexed[i];
            if (l1PostScrambled.load()) decoded[destination] ^= randomBits[destination];
            step += rows;
            if (step == blockSize) { step = 0; ++column; }
        }

        uint32_t crc = 0xFFFFFFFFu;
        for (int i = 0; i < infoBits; ++i) {
            const uint32_t feedback = decoded[i] ^ ((crc >> 31) & 1u);
            crc <<= 1;
            if (feedback) crc ^= 0x04C11DB7u;
        }
        if (crc != readVectorBits(decoded, infoBits, 32)) return;

        const int numPlps = static_cast<int>(readVectorBits(decoded, 15, 8));
        const int numAux = static_cast<int>(readVectorBits(decoded, 23, 4));
        l1SubSlicesPerFrame.store(static_cast<int>(readVectorBits(decoded, 0, 15)));
        l1NumPlps.store(numPlps);
        const int rfShift = (l1NumRf.load() - 1) * 35;
        const int fefShift = l1S2Field2.load() ? 34 : 0;
        int index = rfShift + fefShift + 70;
        for (int i = 0; i < (std::min)(numPlps, 8); ++i) {
            if (index + 89 > infoBits) break;
            plpIds[i].store(static_cast<int>(readVectorBits(decoded, index, 8))); index += 8;
            plpTypes[i].store(static_cast<int>(readVectorBits(decoded, index, 3))); index += 3;
            index += 5 + 1 + 3 + 8 + 8;
            plpCodes[i].store(static_cast<int>(readVectorBits(decoded, index, 3))); index += 3;
            plpMods[i].store(static_cast<int>(readVectorBits(decoded, index, 3))); index += 3;
            plpRotations[i].store(static_cast<int>(readVectorBits(decoded, index, 1))); index += 1;
            plpFecTypes[i].store(static_cast<int>(readVectorBits(decoded, index, 2))); index += 2;
            const int maxBlocks = static_cast<int>(readVectorBits(decoded, index, 10)); index += 10;
            const int frameInterval = static_cast<int>(readVectorBits(decoded, index, 8)); index += 8;
            const int timeLength = static_cast<int>(readVectorBits(decoded, index, 8)); index += 8;
            const int timeType = static_cast<int>(readVectorBits(decoded, index, 1)); index += 1;
            if (i == 0) {
                plpNumBlocksMax.store(maxBlocks);
                plpFrameInterval.store(frameInterval);
                plpTimeIlLength.store(timeLength);
                plpTimeIlType.store(timeType);
            }
            index += 1 + 1 + 11 + 2 + 1 + 1;
        }
        const int plpShift = (numPlps - 1) * 89;
        const int auxShift = (numAux - 1) * 32;
        const int configurableShift = rfShift + fefShift + plpShift + auxShift + 223;
        const int dynamicPlpIndex = configurableShift + 71;
        if (configurableShift >= 0 && dynamicPlpIndex + 48 <= infoBits) {
            l1DynamicFrame.store(static_cast<int>(readVectorBits(decoded, configurableShift, 8)));
            const int dynamicId = static_cast<int>(readVectorBits(decoded, dynamicPlpIndex, 8));
            dynamicPlpStart.store(static_cast<int>(readVectorBits(decoded, dynamicPlpIndex + 8, 22)));
            dynamicPlpBlocks.store(static_cast<int>(readVectorBits(decoded, dynamicPlpIndex + 30, 10)));
            l1DynamicValid.store(dynamicId == plpIds[0].load());
        }
        l1PostValidFrames.fetch_add(1);
        l1PostValid.store(true);
    }

    void prepareDataPilots() {
        if (dataPilotsReady) return;
        std::lock_guard<std::mutex> lock(frameWorkerMutex);
        if (frameWorkerBusy || frameWorkPending) return;
        dataParams = dvbt2Params;
        dataParams.carrier_mode = l1BandwidthExtended.load();
        dvbt2_bwt_ext_parameters_init(dataParams);
        dataParams.guard_interval_mode = l1GuardMode.load();
        dataParams.papr_mode = l1PaprMode.load();
        dataParams.pilot_pattern = l1PilotPattern.load();
        dataParams.n_data = l1DataSymbols.load();
        dvbt2_data_parameters_init(dataParams);
        p2Pilots.data_generator(dataParams);
        p2Addresses.data_address_freq_deinterleaver(dataParams);
        dataPilotsReady = true;
    }

    void analyzeFirstDataSymbol() {
        constexpr int fftSize = 32768;
        constexpr double inputPerDvbSample = INPUT_SAMPLE_RATE / DVB_SAMPLE_RATE;
        const int usefulStart = currentFirstDataUsefulStart;
        if (!dataPilotsReady || usefulStart < 0 ||
            usefulStart + static_cast<int>(std::ceil((fftSize - 1) * inputPerDvbSample)) + 1 >= P2_CAPTURE_SAMPLES) return;

        for (int i = 0; i < fftSize; ++i) {
            const double position = usefulStart + i * inputPerDvbSample;
            const auto sample = resampleDvbAt(p2Capture.data(), P2_CAPTURE_SAMPLES, position);
            const double phase = -2.0 * PI * currentFrameFrequency * position / INPUT_SAMPLE_RATE;
            const float cs = static_cast<float>(std::cos(phase));
            const float sn = static_cast<float>(std::sin(phase));
            p2FftInput[i] = {sample.re * cs - sample.im * sn, sample.re * sn + sample.im * cs};
        }
        fftwf_execute(p2FftPlan);

        const int* map = p2Pilots.data_carrier_map[0];
        const float* references = p2Pilots.data_pilot_refer[0];
        dsp::complex_t previous{0.0f, 0.0f};
        dsp::complex_t sum{0.0f, 0.0f};
        double previousEnergy = 0.0;
        double currentEnergy = 0.0;
        bool havePrevious = false;
        int pairs = 0;
        for (int carrier = 0; carrier < dataParams.k_total; ++carrier) {
            if (map[carrier] == DATA_CARRIER || map[carrier] == TRPAPR_CARRIER) continue;
            if (references[carrier] == 0.0f) continue;
            const int shiftedBin = dataParams.l_nulls + carrier;
            const int rawBin = (shiftedBin + fftSize / 2 + l1CarrierShift.load()) & (fftSize - 1);
            const auto& cell = p2FftOutput[rawBin];
            const float inverseReference = 1.0f / references[carrier];
            const dsp::complex_t channel{cell.re * inverseReference, cell.im * inverseReference};
            if (havePrevious) {
                const auto product = multiplyConjugate(channel, previous);
                sum.re += product.re;
                sum.im += product.im;
                currentEnergy += magnitudeSquared(channel);
                previousEnergy += magnitudeSquared(previous);
                ++pairs;
            }
            previous = channel;
            havePrevious = true;
        }
        const float metric = pairs > 0 ? static_cast<float>(
            std::sqrt(magnitudeSquared(sum)) / std::sqrt((std::max)(currentEnergy * previousEnergy, 1.0e-20))) : 0.0f;
        dataPilotCorrelation.store(metric);
        const bool acquired = metric >= 0.35f;
        dataSymbolSynchronized.store(acquired);
        if (acquired) dataSymbolCount.fetch_add(1);
    }

    void analyzeFullFrameCapture() {
        constexpr int fftLag = 35840;
        const int symbolLength = fftLag + fullFrameGuardSamples;
        const int symbolCount = fullFrameSnapshotReady ? fullFrameSnapshotSymbols : l1DataSymbols.load();
        if (fullFrameP2Boundary < 0 || fullFrameGuardSamples <= 0 || symbolCount <= 0) return;
        auto cpMetricAt = [&](int start) {
            if (start < 0 || start + fftLag + fullFrameGuardSamples > fullFrameFill) return -1.0f;
            dsp::complex_t correlationSum{0.0f, 0.0f};
            double prefixEnergy = 0.0, tailEnergy = 0.0;
            for (int i = 0; i < fullFrameGuardSamples; ++i) {
                const auto& prefix = fullFrameCapture[start + i];
                const auto& tail = fullFrameCapture[start + fftLag + i];
                const auto product = multiplyConjugate(prefix, tail);
                correlationSum.re += product.re;
                correlationSum.im += product.im;
                prefixEnergy += magnitudeSquared(prefix);
                tailEnergy += magnitudeSquared(tail);
            }
            return static_cast<float>(std::sqrt(magnitudeSquared(correlationSum)) /
                std::sqrt((std::max)(prefixEnergy * tailEnergy, 1.0e-20)));
        };

        // The initial P2 capture can begin part-way through its cyclic prefix,
        // making its boundary appear as sample zero.  Refine that boundary on
        // the first complete data symbol, whose entire prefix is in the frame
        // buffer.  A coarse/fine search keeps the per-frame cost bounded.
        const int nominalDataStart = fullFrameP2Boundary + dataParams.n_p2 * symbolLength;
        const int searchRadius = (std::min)(fullFrameGuardSamples / 4, 512);
        int bestCorrection = 0;
        float bestTimingMetric = cpMetricAt(nominalDataStart);
        for (int correction = -searchRadius; correction <= searchRadius; correction += 4) {
            const float metric = cpMetricAt(nominalDataStart + correction);
            if (metric > bestTimingMetric) { bestTimingMetric = metric; bestCorrection = correction; }
        }
        const int coarseCorrection = bestCorrection;
        for (int correction = coarseCorrection - 4; correction <= coarseCorrection + 4; ++correction) {
            const float metric = cpMetricAt(nominalDataStart + correction);
            if (metric > bestTimingMetric) { bestTimingMetric = metric; bestCorrection = correction; }
        }
        const int refinedBoundary = fullFrameP2Boundary + bestCorrection;
        frameTimingCorrection.store(bestCorrection);
        int verified = 0;
        double metricSum = 0.0;
        for (int symbol = 0; symbol < symbolCount; ++symbol) {
            const int start = refinedBoundary + (dataParams.n_p2 + symbol) * symbolLength;
            if (start + fftLag + fullFrameGuardSamples > fullFrameFill) break;
            dsp::complex_t correlationSum{0.0f, 0.0f};
            double prefixEnergy = 0.0;
            double tailEnergy = 0.0;
            for (int i = 0; i < fullFrameGuardSamples; ++i) {
                const auto& prefix = fullFrameCapture[start + i];
                const auto& tail = fullFrameCapture[start + fftLag + i];
                const auto product = multiplyConjugate(prefix, tail);
                correlationSum.re += product.re;
                correlationSum.im += product.im;
                prefixEnergy += magnitudeSquared(prefix);
                tailEnergy += magnitudeSquared(tail);
            }
            const float metric = static_cast<float>(std::sqrt(magnitudeSquared(correlationSum)) /
                std::sqrt((std::max)(prefixEnergy * tailEnergy, 1.0e-20)));
            metricSum += metric;
            if (metric >= 0.30f) ++verified;
        }
        frameSymbolsVerified.store(verified);
        frameMeanCpCorrelation.store(symbolCount > 0 ? static_cast<float>(metricSum / symbolCount) : 0.0f);
        fullFrameCount.fetch_add(1);

        if (dataPilotsReady && fullFrameSnapshotReady) {
            std::lock_guard<std::mutex> lock(frameWorkerMutex);
            if (!frameWorkerBusy && !frameWorkPending) {
                if (!debugRawFrameWritten.exchange(true)) {
                    struct RawFrameHeader {
                        uint32_t magic, version;
                        int32_t sampleCount, boundary, guard, symbolCount, carrierShift;
                        double frequency, inputSampleRate, dvbSampleRate;
                    } header{0x32574152u, 1u, fullFrameFill, refinedBoundary,
                             fullFrameGuardSamples, symbolCount, fullFrameSnapshotCarrierShift,
                             fullFrameSnapshotFrequency, INPUT_SAMPLE_RATE, DVB_SAMPLE_RATE};
                    std::error_code error;
                    std::filesystem::create_directories("recordings", error);
                    std::ofstream output("recordings/dvbt2_debug_raw_frame.bin",
                                         std::ios::binary | std::ios::trunc);
                    if (output) {
                        output.write(reinterpret_cast<const char*>(&header), sizeof(header));
                        output.write(reinterpret_cast<const char*>(fullFrameCapture.data()),
                                     static_cast<std::streamsize>(fullFrameFill * sizeof(dsp::complex_t)));
                        debugRawFrameSaved.store(output.good());
                    }
                }
                fullFrameCapture.swap(frameWorkBuffer);
                workerFrameSamples = fullFrameFill;
                workerFrameGeneration = decodeGeneration.load();
                workerBoundary = refinedBoundary;
                workerGuard = fullFrameGuardSamples;
                workerSymbolCount = symbolCount;
                workerFrequency = fullFrameSnapshotFrequency;
                workerCarrierShift = fullFrameSnapshotCarrierShift;
                workerP2Payload = fullFrameP2Payload;
                previousFrameP2Payload = fullFrameP2Payload;
                workerDynamicFrame.store(fullFrameSnapshotDynamicFrame);
                workerPlpStart = (std::max)(fullFrameSnapshotPlpStart, 0);
                workerPlpBlocks = (std::max)(fullFrameSnapshotPlpBlocks, 0);
                workerPlpModulation = fullFrameSnapshotPlpModulation;
                workerPlpCode = fullFrameSnapshotPlpCode;
                workerPlpFecType = fullFrameSnapshotPlpFecType;
                workerPlpRotation = fullFrameSnapshotPlpRotation;
                workerPlpMaxBlocks = fullFrameSnapshotMaxBlocks;
                workerTimeIlLength = fullFrameSnapshotTimeLength;
                workerTimeIlType = fullFrameSnapshotTimeType;
                // Re-test the two standards-relevant symbol-parity choices now
                // that full-frame CP timing is corrected.  Earlier comparisons
                // were dominated by the 240--290 sample FFT-window error.
                workerFrequencyMapping = 0;
                frameWorkPending = true;
                fullFrameSnapshotReady = false;
                frameWorkerCv.notify_one();
            } else if (queuedFrameCount < FRAME_QUEUE_CAPACITY) {
                // Preserve capture order in a small bounded FIFO.  Buffer swaps
                // keep the real-time capture path constant-time while allowing
                // two transiently slow worker intervals before a frame is lost.
                const int tail = (queuedFrameHead + queuedFrameCount) % FRAME_QUEUE_CAPACITY;
                auto& queued = queuedFrames[tail];
                fullFrameCapture.swap(queued.samples);
                queued.sampleCount = fullFrameFill;
                queued.generation = decodeGeneration.load();
                queued.boundary = refinedBoundary;
                queued.guard = fullFrameGuardSamples;
                queued.symbolCount = symbolCount;
                queued.frequency = fullFrameSnapshotFrequency;
                queued.carrierShift = fullFrameSnapshotCarrierShift;
                queued.p2Payload = fullFrameP2Payload;
                queued.dynamicFrame = fullFrameSnapshotDynamicFrame;
                queued.plpStart = (std::max)(fullFrameSnapshotPlpStart, 0);
                queued.plpBlocks = (std::max)(fullFrameSnapshotPlpBlocks, 0);
                queued.plpModulation = fullFrameSnapshotPlpModulation;
                queued.plpCode = fullFrameSnapshotPlpCode;
                queued.plpFecType = fullFrameSnapshotPlpFecType;
                queued.plpRotation = fullFrameSnapshotPlpRotation;
                queued.plpMaxBlocks = fullFrameSnapshotMaxBlocks;
                queued.timeIlLength = fullFrameSnapshotTimeLength;
                queued.timeIlType = fullFrameSnapshotTimeType;
                ++queuedFrameCount;
                frameWorkPending = true;
                fullFrameSnapshotReady = false;
                workerFramesQueued.fetch_add(1);
                workerQueueDepth.store(queuedFrameCount);
                workerQueuePeak.store((std::max)(workerQueuePeak.load(), queuedFrameCount));
            } else {
                workerFramesSkipped.fetch_add(1);
            }
        }
    }

    static bool generateCellPermutation(int blockMax, int cellsSize, std::vector<int>& permutation) {
        if (blockMax <= 0 || cellsSize <= 0) return false;
        const int pnDegree = static_cast<int>(std::ceil(std::log2(static_cast<double>(cellsSize))));
        if (pnDegree < 11 || pnDegree > 15) return false;
        const int maxStates = 1 << pnDegree;
        static constexpr int logic11[] = {0, 3};
        static constexpr int logic12[] = {0, 2};
        static constexpr int logic13[] = {0, 1, 4, 6};
        static constexpr int logic14[] = {0, 1, 4, 5, 9, 11};
        static constexpr int logic15[] = {0, 1, 2, 12};
        const int* logic = logic14;
        int xorSize = 6;
        if (pnDegree == 11) { logic = logic11; xorSize = 2; }
        else if (pnDegree == 12) { logic = logic12; xorSize = 2; }
        else if (pnDegree == 13) { logic = logic13; xorSize = 4; }
        else if (pnDegree == 15) { logic = logic15; xorSize = 4; }
        const int pnMask = (1 << (pnDegree - 1)) - 1;
        std::vector<int> firstPermutation;
        firstPermutation.reserve(cellsSize);
        int lfsr = 0;
        for (int i = 0; i < maxStates && static_cast<int>(firstPermutation.size()) < cellsSize; ++i) {
            if (i < 2) lfsr = 0;
            else if (i == 2) lfsr = 1;
            else {
                int result = 0;
                for (int k = 0; k < xorSize; ++k) result ^= (lfsr >> logic[k]) & 1;
                lfsr = (lfsr & pnMask) >> 1;
                lfsr |= result << (pnDegree - 2);
            }
            lfsr |= (i & 1) << (pnDegree - 1);
            if (lfsr < cellsSize) firstPermutation.push_back(lfsr);
        }
        if (static_cast<int>(firstPermutation.size()) != cellsSize) return false;
        permutation.resize(static_cast<size_t>(blockMax) * cellsSize);
        int n = 0;
        int address = 0;
        for (int block = 0; block < blockMax; ++block) {
            int shift = cellsSize;
            while (shift >= cellsSize) {
                int temp = n++;
                shift = 0;
                for (int p = 0; p < pnDegree; ++p) {
                    shift |= temp & 1;
                    shift <<= 1;
                    temp >>= 1;
                }
            }
            const int offset = block * cellsSize;
            for (int w = 0; w < cellsSize; ++w) {
                permutation[((firstPermutation[w] + shift) % cellsSize) + offset] = address++;
            }
        }
        return true;
    }

    static uint32_t mpegSectionCrc(const uint8_t* data, size_t size) {
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < size; ++i) {
            crc ^= static_cast<uint32_t>(data[i]) << 24;
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
        }
        return crc;
    }

    static std::string psiText(const uint8_t* data, size_t size) {
        std::string text;
        text.reserve(size);
        for (size_t i = 0; i < size; ++i) {
            const uint8_t value = data[i];
            if (value >= 0x20 && value <= 0x7E) text.push_back(static_cast<char>(value));
            else if (value == 0x0A || value == 0x0D) text.push_back(' ');
            else if (value >= 0x80) text.push_back('?');
        }
        return text;
    }

    void parsePsiSection(int pid, const std::vector<uint8_t>& section) {
        if (section.size() < 8 || mpegSectionCrc(section.data(), section.size()) != 0) return;
        const uint8_t tableId = section[0];
        std::lock_guard<std::mutex> lock(tsServiceMutex);
        if (pid == 0x0000 && tableId == 0x00) {
            tsTransportStreamId.store((section[3] << 8) | section[4]);
            tsPatVersion.store((section[5] >> 1) & 0x1F);
            for (size_t offset = 8; offset + 4 <= section.size() - 4; offset += 4) {
                const int program = (section[offset] << 8) | section[offset + 1];
                const int programPid = ((section[offset + 2] & 0x1F) << 8) | section[offset + 3];
                if (program == 0) continue;
                auto& service = tsServices[program];
                service.serviceId = program;
                service.pmtPid = programPid;
                tsPmtPids.insert(programPid);
                int noSelection = -1;
                selectedTsService.compare_exchange_strong(noSelection, program);
            }
            return;
        }
        if (tsPmtPids.count(pid) && tableId == 0x02 && section.size() >= 16) {
            const int program = (section[3] << 8) | section[4];
            auto& service = tsServices[program];
            service.serviceId = program;
            service.pmtPid = pid;
            service.pcrPid = ((section[8] & 0x1F) << 8) | section[9];
            const int programInfoLength = ((section[10] & 0x0F) << 8) | section[11];
            size_t offset = 12 + programInfoLength;
            std::vector<TsElementaryStream> streams;
            while (offset + 5 <= section.size() - 4) {
                const int streamType = section[offset];
                const int streamPid = ((section[offset + 1] & 0x1F) << 8) | section[offset + 2];
                const int infoLength = ((section[offset + 3] & 0x0F) << 8) | section[offset + 4];
                if (offset + 5 + infoLength > section.size() - 4) break;
                streams.push_back({streamPid, streamType});
                offset += 5 + infoLength;
            }
            service.streams = std::move(streams);
            return;
        }
        if (pid == 0x0011 && (tableId == 0x42 || tableId == 0x46) && section.size() >= 15) {
            size_t offset = 11;
            while (offset + 5 <= section.size() - 4) {
                const int serviceId = (section[offset] << 8) | section[offset + 1];
                const int descriptorLength = ((section[offset + 3] & 0x0F) << 8) | section[offset + 4];
                const size_t descriptorEnd = offset + 5 + descriptorLength;
                if (descriptorEnd > section.size() - 4) break;
                auto& service = tsServices[serviceId];
                service.serviceId = serviceId;
                size_t descriptor = offset + 5;
                while (descriptor + 2 <= descriptorEnd) {
                    const uint8_t tag = section[descriptor];
                    const size_t length = section[descriptor + 1];
                    const size_t end = descriptor + 2 + length;
                    if (end > descriptorEnd) break;
                    if (tag == 0x48 && length >= 3) {
                        const size_t providerLength = section[descriptor + 3];
                        const size_t providerStart = descriptor + 4;
                        if (providerStart + providerLength < end) {
                            const size_t nameLengthOffset = providerStart + providerLength;
                            const size_t nameLength = section[nameLengthOffset];
                            const size_t nameStart = nameLengthOffset + 1;
                            service.provider = psiText(section.data() + providerStart, providerLength);
                            if (nameStart + nameLength <= end)
                                service.name = psiText(section.data() + nameStart, nameLength);
                        }
                    }
                    descriptor = end;
                }
                offset = descriptorEnd;
            }
        }
    }

    void appendPsiPayload(int pid, const uint8_t* data, size_t size, bool payloadStart) {
        auto& state = tsPsiStates[pid];
        size_t offset = 0;
        if (payloadStart) {
            if (size == 0) return;
            const size_t pointer = data[0];
            offset = 1;
            if (!state.bytes.empty()) {
                const size_t continuation = (std::min)(pointer, size - offset);
                state.bytes.insert(state.bytes.end(), data + offset, data + offset + continuation);
                offset += continuation;
                if (state.expected > 0 && state.bytes.size() == state.expected)
                    parsePsiSection(pid, state.bytes);
                state.bytes.clear();
                state.expected = 0;
            } else {
                offset += (std::min)(pointer, size - offset);
            }
        } else if (state.bytes.empty()) {
            return;
        }
        while (offset < size) {
            if (state.bytes.empty() && data[offset] == 0xFF) break;
            state.bytes.push_back(data[offset++]);
            if (state.bytes.size() == 3) {
                state.expected = 3 + (((state.bytes[1] & 0x0F) << 8) | state.bytes[2]);
                if (state.expected < 8 || state.expected > 4096) {
                    state.bytes.clear();
                    state.expected = 0;
                    break;
                }
            }
            if (state.expected > 0 && state.bytes.size() == state.expected) {
                parsePsiSection(pid, state.bytes);
                state.bytes.clear();
                state.expected = 0;
            }
        }
    }

    void inspectTsPacket(const std::array<uint8_t, 188>& packet) {
        if (packet[0] != 0x47 || (packet[1] & 0x80)) return;
        const int pid = ((packet[1] & 0x1F) << 8) | packet[2];
        {
            std::lock_guard<std::mutex> lock(tsServiceMutex);
            if (pid != 0x0000 && pid != 0x0011 && !tsPmtPids.count(pid)) return;
        }
        const int adaptationControl = (packet[3] >> 4) & 3;
        if (adaptationControl == 0 || adaptationControl == 2) return;
        size_t offset = 4;
        if (adaptationControl == 3) {
            offset += 1 + packet[4];
            if (offset >= packet.size()) return;
        }
        const int continuity = packet[3] & 0x0F;
        auto& state = tsPsiStates[pid];
        if (state.continuityValid && continuity != ((state.continuity + 1) & 0x0F)) {
            state.bytes.clear();
            state.expected = 0;
        }
        state.continuity = continuity;
        state.continuityValid = true;
        appendPsiPayload(pid, packet.data() + offset, packet.size() - offset, (packet[1] & 0x40) != 0);
    }

    void closeUdpSocket() {
        if (udpSocket != INVALID_DVBT2_UDP_SOCKET) {
#ifdef _WIN32
            closesocket(udpSocket);
#else
            close(udpSocket);
#endif
            udpSocket = INVALID_DVBT2_UDP_SOCKET;
        }
    }

    void openPlayer() {
        if (!udpStreamingEnabled.exchange(true)) {
            udpRestartRequested.store(true);
            udpQueueCv.notify_one();
        }
        int port = 1234;
        std::string ip;
        std::string command;
        {
            std::lock_guard<std::mutex> lock(udpConfigMutex);
            port = udpPort;
            ip = udpHost;
            command = playerCommand;
        }
        const auto replaceAll = [](std::string& value, const std::string& token,
                                   const std::string& replacement) {
            size_t position = 0;
            while ((position = value.find(token, position)) != std::string::npos) {
                value.replace(position, token.size(), replacement);
                position += replacement.size();
            }
        };
        replaceAll(command, "\r\n", " ");
        replaceAll(command, "\n", " ");
        replaceAll(command, "\r", " ");
        replaceAll(command, "$ip", ip);
        replaceAll(command, "$port", std::to_string(port));

        std::string executable;
        std::string arguments;
        const size_t commandStart = command.find_first_not_of(" \t");
        if (commandStart != std::string::npos && command[commandStart] == '"') {
            const size_t closingQuote = command.find('"', commandStart + 1);
            if (closingQuote != std::string::npos) {
                executable = command.substr(commandStart + 1, closingQuote - commandStart - 1);
                const size_t argumentsStart = command.find_first_not_of(" \t", closingQuote + 1);
                if (argumentsStart != std::string::npos) arguments = command.substr(argumentsStart);
            }
        } else if (commandStart != std::string::npos) {
            const size_t executableEnd = command.find_first_of(" \t", commandStart);
            executable = command.substr(commandStart, executableEnd - commandStart);
            if (executableEnd != std::string::npos) {
                const size_t argumentsStart = command.find_first_not_of(" \t", executableEnd);
                if (argumentsStart != std::string::npos) arguments = command.substr(argumentsStart);
            }
        }
#ifdef _WIN32
        const bool launched = !executable.empty() &&
            reinterpret_cast<intptr_t>(ShellExecuteA(nullptr, "open", executable.c_str(),
                arguments.c_str(), nullptr, SW_SHOWNORMAL)) > 32;
        playerLaunchStatus.store(launched ? 1 : -1);
#else
        playerLaunchStatus.store(-1);
#endif
    }

    bool ensureUdpSocket() {
        if (!udpStreamingEnabled.load()) {
            closeUdpSocket();
            return false;
        }
        if (udpSocket != INVALID_DVBT2_UDP_SOCKET) return true;
#ifdef _WIN32
        if (!udpWinsockReady) return false;
#endif
        std::string host;
        int port = 0;
        {
            std::lock_guard<std::mutex> lock(udpConfigMutex);
            host = udpHost;
            port = udpPort;
        }
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* results = nullptr;
        const std::string portText = std::to_string(port);
        if (getaddrinfo(host.c_str(), portText.c_str(), &hints, &results) != 0) {
            udpErrors.fetch_add(1);
            return false;
        }
        for (addrinfo* result = results; result; result = result->ai_next) {
            const auto candidate = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
            if (candidate == INVALID_DVBT2_UDP_SOCKET) continue;
            udpSocket = candidate;
            std::memcpy(&udpDestination, result->ai_addr, result->ai_addrlen);
            udpDestinationLength = static_cast<int>(result->ai_addrlen);
            break;
        }
        freeaddrinfo(results);
        if (udpSocket == INVALID_DVBT2_UDP_SOCKET) {
            udpErrors.fetch_add(1);
            return false;
        }
        return true;
    }

    void udpSenderLoop() {
        while (true) {
            std::array<uint8_t, 188 * 7> datagram{};
            {
                std::unique_lock<std::mutex> lock(udpQueueMutex);
                udpQueueCv.wait(lock, [&] {
                    return udpSenderStop || udpRestartRequested.load() || !udpDatagramQueue.empty();
                });
                if (udpSenderStop) break;
                if (udpRestartRequested.exchange(false)) {
                    udpDatagramQueue.clear();
                    udpQueueDepth.store(0);
                    lock.unlock();
                    closeUdpSocket();
                    continue;
                }
                if (!udpStreamingEnabled.load()) {
                    udpDatagramQueue.clear();
                    udpQueueDepth.store(0);
                    lock.unlock();
                    closeUdpSocket();
                    continue;
                }
                datagram = std::move(udpDatagramQueue.front());
                udpDatagramQueue.pop_front();
                udpQueueDepth.store(static_cast<int>(udpDatagramQueue.size()));
            }
            if (!ensureUdpSocket()) continue;
            const int sent = sendto(udpSocket, reinterpret_cast<const char*>(datagram.data()),
                                    static_cast<int>(datagram.size()), 0,
                                    reinterpret_cast<const sockaddr*>(&udpDestination), udpDestinationLength);
            if (sent == static_cast<int>(datagram.size())) udpDatagramsSent.fetch_add(1);
            else udpErrors.fetch_add(1);
        }
    }

    void emitSelectedPacket(const std::array<uint8_t, 188>& packet) {
        if (udpPacketResetRequested.exchange(false)) udpPacketFill = 0;
        if (selectedRecordingRestartRequested.exchange(false)) {
            if (selectedTsOutput.is_open()) selectedTsOutput.close();
            selectedTsOutputService = -1;
            selectedRecordedPackets.store(0);
        }
        if (!selectedRecordingEnabled.load()) {
            if (selectedTsOutput.is_open()) selectedTsOutput.close();
            selectedTsOutputService = -1;
        } else {
            const int selected = selectedTsService.load();
            if (selectedTsOutputService != selected) {
                if (selectedTsOutput.is_open()) selectedTsOutput.close();
                std::string outputPath;
                {
                    std::lock_guard<std::mutex> lock(recordingConfigMutex);
                    outputPath = selectedRecordingPath;
                }
                if (appendRecordingTimestamp.load()) outputPath = timestampedRecordingPath(outputPath);
                const std::filesystem::path filesystemPath(outputPath);
                if (!filesystemPath.parent_path().empty()) {
                    std::error_code directoryError;
                    std::filesystem::create_directories(filesystemPath.parent_path(), directoryError);
                }
                selectedTsOutput.open(outputPath, std::ios::binary | std::ios::trunc);
                selectedTsOutputService = selectedTsOutput ? selected : -1;
                selectedRecordedPackets.store(0);
            }
            if (selectedTsOutput.is_open()) {
                selectedTsOutput.write(reinterpret_cast<const char*>(packet.data()), packet.size());
                selectedRecordedPackets.fetch_add(1);
            }
        }
        if (!udpStreamingEnabled.load()) {
            udpPacketFill = 0;
            return;
        }
        std::memcpy(udpPacketBuffer.data() + udpPacketFill * packet.size(), packet.data(), packet.size());
        if (++udpPacketFill < 7) return;
        {
            std::lock_guard<std::mutex> lock(udpQueueMutex);
            if (udpDatagramQueue.size() >= UDP_DATAGRAM_QUEUE_CAPACITY) {
                udpDatagramsDropped.fetch_add(1);
            } else {
                udpDatagramQueue.push_back(udpPacketBuffer);
                udpQueueDepth.store(static_cast<int>(udpDatagramQueue.size()));
                udpQueueCv.notify_one();
            }
        }
        udpPacketFill = 0;
    }

    void signalTransportDiscontinuity() {
        // Do not deliver pre-loss datagrams after newly recovered transport
        // packets.  Keeping the UDP socket open lets VLC retain the selected
        // program while the PCR discontinuity below resets its clock.
        udpPacketFill = 0;
        {
            std::lock_guard<std::mutex> lock(udpQueueMutex);
            udpDatagramQueue.clear();
            udpQueueDepth.store(0);
        }
        selectedDiscontinuityPending.store(true);
    }

    void writeSelectedTsPacket(const std::array<uint8_t, 188>& packet) {
        const int selected = selectedTsService.load();
        if (selected < 0) return;
        TsServiceInfo service;
        {
            std::lock_guard<std::mutex> lock(tsServiceMutex);
            const auto found = tsServices.find(selected);
            if (found == tsServices.end() || found->second.pmtPid < 0) return;
            service = found->second;
        }
        const int pid = ((packet[1] & 0x1F) << 8) | packet[2];
        bool include = pid == 0x0000 || pid == 0x0001 || pid == 0x0010 || pid == 0x0011 ||
            pid == 0x0012 || pid == 0x0014 || pid == service.pmtPid || pid == service.pcrPid;
        if (!include) {
            for (const auto& stream : service.streams) {
                if (pid == stream.pid) {
                    include = true;
                    break;
                }
            }
        }
        if (!include) return;
        if (pid != 0x0000) {
            std::array<uint8_t, 188> selectedPacket = packet;
            if (pid == service.pcrPid && selectedDiscontinuityPending.load()) {
                const int adaptationControl = (selectedPacket[3] >> 4) & 0x03;
                if ((adaptationControl == 2 || adaptationControl == 3) && selectedPacket[4] >= 1) {
                    selectedPacket[5] |= 0x80; // adaptation_field discontinuity_indicator
                    selectedDiscontinuityPending.store(false);
                }
            }
            selectedTsPacketsWritten.fetch_add(1);
            emitSelectedPacket(selectedPacket);
            return;
        }
        // Replace the multiplex PAT with a one-program PAT so players select
        // the requested service immediately rather than a missing first entry.
        std::array<uint8_t, 188> selectedPat{};
        selectedPat.fill(0xFF);
        selectedPat[0] = 0x47;
        selectedPat[1] = 0x40;
        selectedPat[2] = 0x00;
        selectedPat[3] = static_cast<uint8_t>(0x10 | (packet[3] & 0x0F));
        selectedPat[4] = 0x00;
        selectedPat[5] = 0x00;
        selectedPat[6] = 0xB0;
        selectedPat[7] = 0x0D;
        const int transportStreamId = tsTransportStreamId.load();
        selectedPat[8] = static_cast<uint8_t>(transportStreamId >> 8);
        selectedPat[9] = static_cast<uint8_t>(transportStreamId);
        // Receivers only apply changed PAT contents after the version changes.
        // The source multiplex version is constant while our selected program changes.
        selectedPat[10] = static_cast<uint8_t>(0xC1 | ((selectedPatVersion.load() & 0x1F) << 1));
        selectedPat[11] = 0x00;
        selectedPat[12] = 0x00;
        selectedPat[13] = static_cast<uint8_t>(selected >> 8);
        selectedPat[14] = static_cast<uint8_t>(selected);
        selectedPat[15] = static_cast<uint8_t>(0xE0 | ((service.pmtPid >> 8) & 0x1F));
        selectedPat[16] = static_cast<uint8_t>(service.pmtPid);
        const uint32_t crc = mpegSectionCrc(selectedPat.data() + 5, 12);
        selectedPat[17] = static_cast<uint8_t>(crc >> 24);
        selectedPat[18] = static_cast<uint8_t>(crc >> 16);
        selectedPat[19] = static_cast<uint8_t>(crc >> 8);
        selectedPat[20] = static_cast<uint8_t>(crc);
        selectedTsPacketsWritten.fetch_add(1);
        emitSelectedPacket(selectedPat);
    }

    void processHighEfficiencyBbFrame(const std::vector<uint8_t>& bits, int dfl, int syncd, bool npd,
                                      int fecBlock, int firstTiBlocks) {
        if (dfl <= 0 || dfl > static_cast<int>(bits.size()) - 80 || (dfl & 7) != 0 ||
            syncd < 0 || syncd > dfl || (syncd & 7) != 0 || syncd == 65535) return;
        std::lock_guard<std::mutex> lock(tsOutputMutex);
        std::error_code error;
        std::filesystem::create_directories("recordings", error);
        const int payloadBytes = dfl / 8;
        const int syncBytes = syncd / 8;
        std::vector<uint8_t> payload(payloadBytes);
        for (int byte = 0; byte < payloadBytes; ++byte) {
            uint8_t value = 0;
            for (int bit = 0; bit < 8; ++bit) value = static_cast<uint8_t>((value << 1) | bits[80 + byte * 8 + bit]);
            payload[byte] = value;
        }

        auto writePacket = [&](const std::array<uint8_t, 188>& packet) {
            inspectTsPacket(packet);
            if (fullRecordingRestartRequested.exchange(false)) {
                if (tsOutput.is_open()) tsOutput.close();
                fullRecordedPackets.store(0);
            }
            if (!fullRecordingEnabled.load()) {
                if (tsOutput.is_open()) tsOutput.close();
            } else {
                if (!tsOutput.is_open()) {
                    std::string outputPath;
                    {
                        std::lock_guard<std::mutex> lock(recordingConfigMutex);
                        outputPath = fullRecordingPath;
                    }
                    if (appendRecordingTimestamp.load()) outputPath = timestampedRecordingPath(outputPath);
                    const std::filesystem::path filesystemPath(outputPath);
                    if (!filesystemPath.parent_path().empty()) {
                        std::error_code directoryError;
                        std::filesystem::create_directories(filesystemPath.parent_path(), directoryError);
                    }
                    tsOutput.open(outputPath, std::ios::binary | std::ios::trunc);
                }
                if (tsOutput.is_open()) {
                    tsOutput.write(reinterpret_cast<const char*>(packet.data()), packet.size());
                    fullRecordedPackets.fetch_add(1);
                }
            }
            tsPacketsWritten.fetch_add(1);
            tsBytesWritten.fetch_add(188);
            writeSelectedTsPacket(packet);
        };
        auto restoreDeletedNulls = [&](uint8_t count) {
            static const std::array<uint8_t, 188> nullPacket = [] {
                std::array<uint8_t, 188> packet{};
                packet.fill(0xFF);
                packet[0] = 0x47;
                packet[1] = 0x1F;
                packet[2] = 0xFF;
                packet[3] = 0x10;
                return packet;
            }();
            for (int i = 0; i < count; ++i) writePacket(nullPacket);
            tsNullPacketsReinserted.fetch_add(count);
        };
        auto appendDataFieldByte = [&](uint8_t value) {
            if (npd && tsAwaitingDnp) {
                // DNP follows a transmitted UP but describes null packets that
                // preceded that UP.  Delay the useful packet by one byte so the
                // original constant-rate packet order can be reconstructed.
                restoreDeletedNulls(value);
                writePacket(tsPendingUsefulPacket);
                tsAwaitingDnp = false;
                return;
            }
            if (tsPacketPosition == 0) tsPacketBuffer[tsPacketPosition++] = 0x47;
            tsPacketBuffer[tsPacketPosition++] = value;
            if (tsPacketPosition == 188) {
                tsPacketPosition = 0;
                if (npd) {
                    tsPendingUsefulPacket = tsPacketBuffer;
                    tsAwaitingDnp = true;
                } else {
                    writePacket(tsPacketBuffer);
                }
            }
        };
        int index = 0;
        if (!tsSynchronized) {
            index = syncBytes;
            tsPacketPosition = 0;
            tsAwaitingDnp = false;
            tsSynchronized = true;
        } else {
            const int expectedContinuation = tsAwaitingDnp ? 1 :
                (tsPacketPosition == 0 ? 0 : 188 - tsPacketPosition + (npd ? 1 : 0));
            if (syncBytes != expectedContinuation) {
                index = syncBytes;
                tsPacketPosition = 0;
                tsAwaitingDnp = false;
                signalTransportDiscontinuity();
                tsResyncs.fetch_add(1);
                if (fecBlock == 0) tsResyncFirstBlock.fetch_add(1);
                else if (fecBlock == firstTiBlocks) tsResyncTiBoundary.fetch_add(1);
                else tsResyncOtherBlock.fetch_add(1);
                tsLastResyncBlock.store(fecBlock);
                tsLastActualSyncBytes.store(syncBytes);
                tsLastExpectedSyncBytes.store(expectedContinuation);
                if (fecBlock >= 0 && fecBlock < TS_RESYNC_BLOCK_BUCKETS)
                    tsResyncByBlock[fecBlock].fetch_add(1);
                // With NPD disabled, advancing or retreating by one complete
                // BBFRAME changes SYNCD by DFL modulo the 187-byte HEM UP.
                const int framePhase = payloadBytes % 187;
                const int delta = (syncBytes - expectedContinuation + 187) % 187;
                if (!npd && delta == (187 - framePhase) % 187)
                    tsResyncForwardOne.fetch_add(1);
                else if (!npd && delta == framePhase)
                    tsResyncBackwardOne.fetch_add(1);
                else
                    tsResyncIrregular.fetch_add(1);
            } else {
                for (; index < syncBytes; ++index) appendDataFieldByte(payload[index]);
            }
        }
        for (; index < payloadBytes; ++index) appendDataFieldByte(payload[index]);
        // Let the stream buffer coalesce BBFRAME writes.  Flushing here forced
        // up to 133 synchronous filesystem operations per received RF frame.
    }

    void computeWorkerLoop(int workerIndex) {
        lowerDecoderThreadPriority();
        uint64_t observedGeneration = 0;
        std::unique_lock<std::mutex> lock(computeMutex);
        while (true) {
            computeCv.wait(lock, [&] { return computeStop || computeGeneration != observedGeneration; });
            if (computeStop) return;
            observedGeneration = computeGeneration;
            const bool participating = workerIndex < computeActiveWorkers;
            auto task = computeTask;
            lock.unlock();
            if (participating) task(workerIndex);
            lock.lock();
            if (participating && --computeWorkersRemaining == 0) computeDoneCv.notify_one();
        }
    }

    void runComputeWorkers(int workerCount, std::function<void(int)> task) {
        std::unique_lock<std::mutex> lock(computeMutex);
        computeTask = std::move(task);
        computeActiveWorkers = (std::max)(1, (std::min)(computeWorkerLimit, workerCount));
        computeWorkersRemaining = computeActiveWorkers;
        ++computeGeneration;
        computeCv.notify_all();
        computeDoneCv.wait(lock, [&] { return computeWorkersRemaining == 0; });
        computeTask = {};
    }

    bool decodeCancelled() const {
        return activeFrameGeneration.load() != decodeGeneration.load();
    }

    void abandonCancelledFrame() {
        std::lock_guard<std::mutex> lock(frameWorkerMutex);
        queuedFrameCount = 0;
        frameWorkPending = false;
        frameWorkerBusy = false;
        workerQueueDepth.store(0);
        for (auto& queued : queuedFrames) queued.p2Payload.clear();
    }

    void timeDeinterleaveAndDemap(int plpBlocks, int modulation, int codeRate, int fecType, int rotation,
                                  int maxBlocks, int timeLength, int timeType, int frequencyMapping) {
        const auto demapStarted = std::chrono::steady_clock::now();
        if (decodeCancelled()) return;
        if (modulation != 3 || timeType != 0 || timeLength <= 0 || plpBlocks <= 0) return;
        const int fecBits = fecType == 1 ? 64800 : 16200;
        const int cellsPerBlock = fecBits / 8;
        const int rows = cellsPerBlock / 5;
        maxBlocks = (std::max)(maxBlocks, plpBlocks);
        if (workerPermutationMaxBlocks != maxBlocks ||
            workerPermutationCellsPerBlock != cellsPerBlock) {
            if (!generateCellPermutation(maxBlocks, cellsPerBlock, workerCellPermutation)) return;
            workerPermutationMaxBlocks = maxBlocks;
            workerPermutationCellsPerBlock = cellsPerBlock;
        }
        const auto& permutation = workerCellPermutation;
        workerTimeDeinterleaved.assign(static_cast<size_t>(plpBlocks) * cellsPerBlock, {});
        size_t inputOffset = 0;
        uint64_t completedTiBlocks = 0;
        uint64_t reorderedFecBlocks = 0;
        const int baseBlocks = plpBlocks / timeLength;
        const int remainder = plpBlocks % timeLength;
        if (workerExtractedPlp.size() < static_cast<size_t>(plpBlocks) * cellsPerBlock) return;
        for (int ti = 0; ti < timeLength; ++ti) {
            if (decodeCancelled()) return;
            const int blocks = baseBlocks + (ti >= timeLength - remainder ? 1 : 0);
            if (blocks <= 0) continue;
            const int tiSize = blocks * cellsPerBlock;
            if (inputOffset + tiSize > workerExtractedPlp.size()) return;
            const size_t tiOffset = inputOffset;
            const int cycleLength = blocks * 5;
            const int tiWorkers = computeWorkerLimit;
            runComputeWorkers(tiWorkers, [&](int worker) {
                const int first = tiSize * worker / tiWorkers;
                const int last = tiSize * (worker + 1) / tiWorkers;
                for (int i = first; i < last; ++i) {
                    const int d = (i % cycleLength) * rows + i / cycleLength;
                    const int iAddress = permutation[d];
                    const auto& cell = workerExtractedPlp[tiOffset + i];
                    const int blockBase = (iAddress / cellsPerBlock) * cellsPerBlock;
                    const int local = iAddress - blockBase;
                    const int qLocal = (local + cellsPerBlock - 1) % cellsPerBlock;
                    workerTimeDeinterleaved[tiOffset + blockBase + qLocal].im = cell.im;
                    workerTimeDeinterleaved[tiOffset + iAddress].re = cell.re;
                }
            });
            inputOffset += tiSize;
            ++completedTiBlocks;
            reorderedFecBlocks += blocks;
        }
        if (workerTimeDeinterleaved.size() != static_cast<size_t>(plpBlocks) * cellsPerBlock) return;

        if (frequencyMapping == 0 && !debugTimeDeinterleaverWritten.exchange(true)) {
            struct TimeTraceHeader {
                uint32_t magic, version;
                int32_t plpBlocks, maxBlocks, timeLength, timeType, cellsPerBlock;
                uint64_t inputCells, outputCells;
            } traceHeader{0x32495444u, 1u, plpBlocks, maxBlocks, timeLength, timeType,
                          cellsPerBlock, static_cast<uint64_t>(workerExtractedPlp.size()),
                          static_cast<uint64_t>(workerTimeDeinterleaved.size())};
            std::error_code error;
            std::filesystem::create_directories("recordings", error);
            std::ofstream output("recordings/dvbt2_debug_time_deinterleaver.bin",
                                 std::ios::binary | std::ios::trunc);
            if (output) {
                output.write(reinterpret_cast<const char*>(&traceHeader), sizeof(traceHeader));
                output.write(reinterpret_cast<const char*>(workerTimeDeinterleaved.data()),
                             static_cast<std::streamsize>(workerTimeDeinterleaved.size() * sizeof(dsp::complex_t)));
                debugTimeDeinterleaverSaved.store(output.good());
            }
        }

        const float rot = rotation ? -ROT_QAM256 : 0.0f;
        const float cs = std::cos(rot), sn = std::sin(rot);
        const int qamWorkers = computeWorkerLimit;
        const size_t qamCellCount = workerTimeDeinterleaved.size();
        std::array<double, MAX_COMPUTE_WORKERS> energySums{};
        std::array<size_t, MAX_COMPUTE_WORKERS> energyCounts{};
        runComputeWorkers(qamWorkers, [&](int worker) {
            const size_t first = qamCellCount * worker / qamWorkers;
            const size_t last = qamCellCount * (worker + 1) / qamWorkers;
            double sum = 0.0;
            size_t count = 0;
            for (size_t index = first; index < last; ++index) {
                auto& cell = workerTimeDeinterleaved[index];
                const float re = cell.re * cs - cell.im * sn;
                const float im = cell.re * sn + cell.im * cs;
                cell = {re, im};
                const float power = magnitudeSquared(cell);
                if (std::isfinite(power)) { sum += power; ++count; }
            }
            energySums[worker] = sum;
            energyCounts[worker] = count;
        });
        double meanEnergySum = 0.0;
        size_t validEnergyCells = 0;
        for (int worker = 0; worker < qamWorkers; ++worker) {
            meanEnergySum += energySums[worker];
            validEnergyCells += energyCounts[worker];
        }
        constexpr float norm = NORM_FACTOR_QAM256;
        constexpr float maximumRadius = 2.2f;
        const double meanEnergy = validEnergyCells > 0
            ? meanEnergySum / static_cast<double>(validEnergyCells) : 0.0;
        const float qamScale = meanEnergy > 1.0e-18
            ? static_cast<float>(1.0 / std::sqrt(meanEnergy)) : 1.0f;
        double signalEnergy = 0.0, errorEnergy = 0.0;
        auto nearest = [](float value) {
            int level = static_cast<int>(std::lround((std::abs(value) / NORM_FACTOR_QAM256 - 1.0f) * 0.5f));
            level = (std::max)(0, (std::min)(7, level));
            return std::copysign((2 * level + 1) * NORM_FACTOR_QAM256, value);
        };
        std::array<double, MAX_COMPUTE_WORKERS> signalSums{};
        std::array<double, MAX_COMPUTE_WORKERS> errorSums{};
        runComputeWorkers(qamWorkers, [&](int worker) {
            const size_t first = qamCellCount * worker / qamWorkers;
            const size_t last = qamCellCount * (worker + 1) / qamWorkers;
            double localSignal = 0.0, localError = 0.0;
            for (size_t index = first; index < last; ++index) {
                auto& cell = workerTimeDeinterleaved[index];
                cell.re *= qamScale;
                cell.im *= qamScale;
                const float radiusSquared = magnitudeSquared(cell);
                if (!std::isfinite(radiusSquared)) {
                    cell = {0.0f, 0.0f};
                } else if (radiusSquared > maximumRadius * maximumRadius) {
                    const float limiter = maximumRadius / std::sqrt(radiusSquared);
                    cell.re *= limiter;
                    cell.im *= limiter;
                }
                const float idealRe = nearest(cell.re), idealIm = nearest(cell.im);
                localSignal += idealRe * idealRe + idealIm * idealIm;
                const float er = cell.re - idealRe, ei = cell.im - idealIm;
                localError += er * er + ei * ei;
            }
            signalSums[worker] = localSignal;
            errorSums[worker] = localError;
        });
        {
            std::array<dsp::complex_t, QAM_CONSTELLATION_POINTS> snapshot{};
            const size_t snapshotCount = (std::min)(QAM_CONSTELLATION_POINTS, qamCellCount);
            for (size_t sample = 0; sample < snapshotCount; ++sample) {
                snapshot[sample] = workerTimeDeinterleaved[sample * qamCellCount / snapshotCount];
            }
            std::lock_guard<std::mutex> lock(qamConstellationMutex);
            qamConstellationHead = (qamConstellationHead + 1) % QAM_CONSTELLATION_HISTORY;
            qamConstellationCounts[qamConstellationHead] = snapshotCount;
            std::copy_n(snapshot.begin(), snapshotCount, qamConstellationHistory[qamConstellationHead].begin());
        }
        for (int worker = 0; worker < qamWorkers; ++worker) {
            signalEnergy += signalSums[worker];
            errorEnergy += errorSums[worker];
        }
        const float snr = errorEnergy > 0.0 ? static_cast<float>(10.0 * std::log10(signalEnergy / errorEnergy)) : 60.0f;
        qamSnrDb.store(snr);
        const float reliability = 8.0f * norm * static_cast<float>(signalEnergy / (std::max)(errorEnergy, 1.0e-9));
        const float precision = (std::min)(120.0f, (std::max)(2.0f, reliability));
        workerSoftBits.resize(workerTimeDeinterleaved.size() * 8);
        auto quantize = [](float value) { return static_cast<int8_t>((std::max)(-127.0f, (std::min)(127.0f, std::round(value)))); };
        const int llrWorkers = computeWorkerLimit;
        const size_t cellCount = workerTimeDeinterleaved.size();
        runComputeWorkers(llrWorkers, [&](int worker) {
                const size_t first = cellCount * worker / llrWorkers;
                const size_t last = cellCount * (worker + 1) / llrWorkers;
                for (size_t index = first; index < last; ++index) {
                    const auto& cell = workerTimeDeinterleaved[index];
                    const float aRe = std::abs(cell.re), aIm = std::abs(cell.im);
                    const float l2r = aRe - 8.0f * norm, l2i = aIm - 8.0f * norm;
                    const float l4r = std::abs(l2r) - 4.0f * norm, l4i = std::abs(l2i) - 4.0f * norm;
                    const size_t out = index * 8;
                    workerSoftBits[out] = quantize(cell.re * precision);
                    workerSoftBits[out + 1] = quantize(cell.im * precision);
                    workerSoftBits[out + 2] = quantize(l2r * precision);
                    workerSoftBits[out + 3] = quantize(l2i * precision);
                    workerSoftBits[out + 4] = quantize(l4r * precision);
                    workerSoftBits[out + 5] = quantize(l4i * precision);
                    workerSoftBits[out + 6] = quantize((std::abs(l4r) - 2.0f * norm) * precision);
                    workerSoftBits[out + 7] = quantize((std::abs(l4i) - 2.0f * norm) * precision);
                }
            });
        tiBlocksCompleted.store(completedTiBlocks);
        fecBlocksReordered.store(reorderedFecBlocks);
        softQamValues.store(workerSoftBits.size());

        // DVB-T2 bit deinterleaver and bit-to-cell demultiplexer for normal-frame 256-QAM, rate 2/3.
        if (fecType != 1 || codeRate != 2 || fecBits != 64800) return;
        static const auto bitAddress = [] {
            constexpr int twist[16] = {0, 2, 2, 2, 2, 3, 7, 15, 16, 20, 22, 22, 27, 27, 28, 32};
            constexpr int demux[16] = {3, 15, 1, 7, 4, 11, 5, 0, 12, 2, 9, 14, 13, 6, 8, 10};
            constexpr int columns = 4050;
            constexpr int rowsPerWord = 16;
            std::array<int, 64800> addresses{};
            std::array<int, 64800> columnAddress{};
            for (int column = 0; column < columns; ++column) {
                for (int rowIndex = 0; rowIndex < rowsPerWord; ++rowIndex) {
                    columnAddress[column * rowsPerWord + rowIndex] =
                        columns * rowIndex + (column + columns - twist[rowIndex]) % columns;
                }
            }
            int k = 0, n = 0;
            for (int i = 0; i < 64800; ++i) {
                addresses[i] = columnAddress[demux[n] + k];
                if (++n == rowsPerWord) { n = 0; k += rowsPerWord; }
            }
            return addresses;
        }();
        workerFecSoftBits.resize(workerSoftBits.size());
        std::atomic<int> nextReorderBlock{0};
        runComputeWorkers(4, [&](int) {
                while (true) {
                    const int block = nextReorderBlock.fetch_add(1);
                    if (block >= plpBlocks) break;
                    const size_t offset = static_cast<size_t>(block) * 64800;
                    for (int i = 0; i < 64800; ++i)
                        workerFecSoftBits[offset + bitAddress[i]] = workerSoftBits[offset + i];
                }
            });
        bitDeinterleavedBlocks.store(plpBlocks);

        if (frequencyMapping == 0 && !debugFecInputWritten.exchange(true)) {
            struct FecTraceHeader {
                uint32_t magic, version;
                int32_t blocks, fecBits, modulation, codeRate, rotation;
                uint64_t rawValues, fecValues;
            } traceHeader{0x32434546u, 1u, plpBlocks, fecBits, modulation, codeRate, rotation,
                          static_cast<uint64_t>(workerSoftBits.size()),
                          static_cast<uint64_t>(workerFecSoftBits.size())};
            std::error_code error;
            std::filesystem::create_directories("recordings", error);
            std::ofstream output("recordings/dvbt2_debug_fec_input.bin", std::ios::binary | std::ios::trunc);
            if (output) {
                output.write(reinterpret_cast<const char*>(&traceHeader), sizeof(traceHeader));
                output.write(reinterpret_cast<const char*>(workerSoftBits.data()),
                             static_cast<std::streamsize>(workerSoftBits.size()));
                output.write(reinterpret_cast<const char*>(workerFecSoftBits.data()),
                             static_cast<std::streamsize>(workerFecSoftBits.size()));
                debugFecInputSaved.store(output.good());
            }
        }

        // Test one SIMD group per received frame. This bounds CPU use while providing a real parity-check result.
        constexpr int lanes = 32;
        constexpr int kLdpc = 43200;
        constexpr int qLdpc = 60;
        using LdpcSimd = SIMD<int8_t, lanes>;
        using LdpcUpdate = gnr::NormalUpdate<LdpcSimd>;
        using LdpcAlgorithm2 = gnr::OffsetMinSumAlgorithm<LdpcSimd, LdpcUpdate, 2>;
        using LdpcAlgorithmAlternate = gnr::MinSumAlgorithm<LdpcSimd, LdpcUpdate>;
        static auto* ldpcTable = new LDPC<DVB_T2_TABLE_NORMAL_C2_3>();
        static auto* ldpcDecoder2 = [] {
            auto* decoder = new LDPCDecoder<LdpcSimd, LdpcAlgorithm2>();
            decoder->init(ldpcTable);
            return decoder;
        }();
        if (!ldpcSelfTestRun.exchange(true)) {
            std::vector<uint8_t> testBits(64800, 0);
            uint32_t state = 0x13579BDFu;
            for (int i = 0; i < kLdpc; ++i) {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                testBits[i] = static_cast<uint8_t>(state & 1u);
            }
            std::vector<uint8_t> checks(64800 - kLdpc, 0);
            ldpcTable->first_bit();
            for (int bit = 0; bit < kLdpc; ++bit) {
                const int degree = ldpcTable->bit_deg();
                const int* positions = ldpcTable->acc_pos();
                if (testBits[bit]) {
                    for (int edge = 0; edge < degree; ++edge) checks[positions[edge]] ^= 1;
                }
                ldpcTable->next_bit();
            }
            for (int row = 0; row < 360; ++row) {
                for (int group = 0; group < qLdpc; ++group) {
                    const int parityIndex = group * 360 + row;
                    const int checkIndex = qLdpc * row + group;
                    uint8_t previous = 0;
                    if (group > 0) previous = testBits[kLdpc + (group - 1) * 360 + row];
                    else if (row > 0) previous = testBits[kLdpc + (qLdpc - 1) * 360 + row - 1];
                    testBits[kLdpc + parityIndex] = checks[checkIndex] ^ previous;
                }
            }
            std::vector<LdpcSimd> testWords(64800);
            for (int lane = 0; lane < lanes; ++lane) {
                for (int i = 0; i < kLdpc; ++i)
                    testWords[i].v[lane] = testBits[i] ? -80 : 80;
                for (int group = 0; group < qLdpc; ++group) {
                    for (int row = 0; row < 360; ++row) {
                        const int source = kLdpc + group * 360 + row;
                        testWords[kLdpc + qLdpc * row + group].v[lane] = testBits[source] ? -80 : 80;
                    }
                }
                // A small deterministic error pattern exercises actual correction,
                // rather than merely accepting an already-valid codeword.
                for (int error = 0; error < 12; ++error) {
                    const int position = (error * 5003 + lane * 97) % 64800;
                    testWords[position].v[lane] = -testWords[position].v[lane];
                }
            }
            const int trialsRemaining = (*ldpcDecoder2)(testWords.data(), testWords.data() + kLdpc, 25, lanes);
            ldpcSelfTestStatus.store(trialsRemaining >= 0 ? 1 : 0);
        }
        const auto ldpcStarted = std::chrono::steady_clock::now();
        pendingDemapMilliseconds = std::chrono::duration<float, std::milli>(
            ldpcStarted - demapStarted).count();
        const int batchCount = (plpBlocks + lanes - 1) / lanes;
        std::vector<std::vector<LdpcSimd>> batchWords(
            batchCount, std::vector<LdpcSimd>(64800));
        std::vector<int> batchActive(batchCount);
        for (int batch = 0; batch < batchCount; ++batch) {
            if (decodeCancelled()) return;
            const int firstBlock = batch * lanes;
            const int active = (std::min)(lanes, plpBlocks - firstBlock);
            batchActive[batch] = active;
            auto& words = batchWords[batch];
            for (int lane = 0; lane < active; ++lane) {
                const int8_t* source = workerFecSoftBits.data() +
                    static_cast<size_t>(firstBlock + lane) * 64800;
                for (int i = 0; i < kLdpc; ++i) words[i].v[lane] = source[i];
                for (int t = 0; t < qLdpc; ++t) {
                    for (int s = 0; s < 360; ++s)
                        words[kLdpc + qLdpc * s + t].v[lane] = source[kLdpc + 360 * t + s];
                }
            }
            for (int lane = active; lane < lanes; ++lane) {
                for (auto& word : words) word.v[lane] = word.v[0];
            }
        }

        const int decoderThreads = computeWorkerLimit;
        static auto parallelDecoders = [&] {
            std::array<LDPCDecoder<LdpcSimd, LdpcAlgorithm2>*, MAX_COMPUTE_WORKERS> decoders{};
            for (auto& decoder : decoders) {
                decoder = new LDPCDecoder<LdpcSimd, LdpcAlgorithm2>();
                decoder->init(ldpcTable);
            }
            return decoders;
        }();
        static auto* alternateDecoder = [] {
            auto* decoder = new LDPCDecoder<LdpcSimd, LdpcAlgorithmAlternate>();
            decoder->init(ldpcTable);
            return decoder;
        }();
        static auto* halfScaleDecoder = [] {
            auto* decoder = new LDPCDecoder<LdpcSimd, LdpcAlgorithm2>();
            decoder->init(ldpcTable);
            return decoder;
        }();
        static auto* quarterScaleDecoder = [] {
            auto* decoder = new LDPCDecoder<LdpcSimd, LdpcAlgorithm2>();
            decoder->init(ldpcTable);
            return decoder;
        }();
        std::atomic<int> nextBatch{0};
        std::vector<int> batchTrialsRemaining(batchCount, -1);
        const int activeWorkers = (std::min)(decoderThreads, batchCount);
        runComputeWorkers(activeWorkers, [&](int worker) {
                while (true) {
                    if (decodeCancelled()) break;
                    const int batch = nextBatch.fetch_add(1);
                    if (batch >= batchCount) break;
                    auto& words = batchWords[batch];
                    batchTrialsRemaining[batch] = (*parallelDecoders[worker])(
                        words.data(), words.data() + kLdpc, 35, batchActive[batch]);
                }
            });
        if (decodeCancelled()) return;

        uint64_t frameConverged = 0;
        uint64_t frameTested = 0;
        double frameDecodedUnsatisfied = 0.0;
        for (int batch = 0; batch < batchCount; ++batch) {
        if (decodeCancelled()) return;
        const int active = batchActive[batch];
        auto& words = batchWords[batch];
        auto countUnsatisfied = [&](const std::vector<LdpcSimd>& values, uint64_t& converged,
                                    std::array<uint8_t, lanes>* validLanes = nullptr) {
            std::array<std::vector<int>, 32> checksByLane;
            for (auto& checks : checksByLane) checks.assign(64800 - kLdpc, 0);
            if (validLanes) validLanes->fill(0);
            ldpcTable->first_bit();
            for (int bit = 0; bit < kLdpc; ++bit) {
                const int degree = ldpcTable->bit_deg();
                const int* positions = ldpcTable->acc_pos();
                for (int lane = 0; lane < active; ++lane) {
                    if (values[bit].v[lane] >= 0) continue;
                    for (int edge = 0; edge < degree; ++edge) checksByLane[lane][positions[edge]] ^= 1;
                }
                ldpcTable->next_bit();
            }
            uint64_t total = 0;
            converged = 0;
            for (int lane = 0; lane < active; ++lane) {
                int unsatisfied = 0;
                for (int group = 0; group < qLdpc; ++group) {
                    for (int row = 0; row < 360; ++row) {
                        const int checkIndex = qLdpc * row + group;
                        int syndrome = checksByLane[lane][checkIndex];
                        syndrome ^= values[kLdpc + qLdpc * row + group].v[lane] < 0;
                        if (group != 0) syndrome ^= values[kLdpc + qLdpc * row + group - 1].v[lane] < 0;
                        else if (row != 0) syndrome ^= values[kLdpc + qLdpc * (row - 1) + qLdpc - 1].v[lane] < 0;
                        unsatisfied += syndrome;
                    }
                }
                total += unsatisfied;
                if (unsatisfied == 0) {
                    ++converged;
                    if (validLanes) (*validLanes)[lane] = 1;
                }
            }
            return active > 0 ? static_cast<float>(total) / active : 0.0f;
        };
        // The reference decoder's return value is all-or-nothing for the SIMD group.
        // Re-evaluate every lane independently so one weak block cannot hide valid ones.
        uint64_t converged = 0;
        std::array<uint8_t, lanes> validLanes{};
        float meanUnsatisfied = 0.0f;
        if (batchTrialsRemaining[batch] >= 0) {
            // LDPCDecoder only returns a non-negative trial count after its
            // parity check has accepted every active SIMD lane.
            converged = active;
            for (int lane = 0; lane < active; ++lane) validLanes[lane] = 1;
        } else {
            // Preserve partial-batch recovery for marginal signals.
            meanUnsatisfied = countUnsatisfied(words, converged, &validLanes);
        }

        // Quantized LLRs can leave offset min-sum in a tiny trapping set even
        // when the hard decisions are close to a codeword.  Offline replay of
        // captured residual lanes showed that halving their magnitudes escapes
        // those sets without changing any signs.  Try this fast-converging path
        // immediately after the primary decoder and accept it lane-by-lane.
        std::vector<LdpcSimd> halfScaleWords;
        std::array<uint8_t, lanes> halfScaleValidLanes{};
        if (!decodeCancelled() && converged < static_cast<uint64_t>(active)) {
            halfScaleWords.assign(64800, LdpcSimd{});
            const int firstBlock = batch * lanes;
            auto halveLlr = [](int8_t value) {
                const int widened = value;
                return static_cast<int8_t>(widened >= 0 ? (widened + 1) / 2 : (widened - 1) / 2);
            };
            for (int lane = 0; lane < active; ++lane) {
                const int8_t* source = workerFecSoftBits.data() +
                    static_cast<size_t>(firstBlock + lane) * 64800;
                for (int i = 0; i < kLdpc; ++i) halfScaleWords[i].v[lane] = halveLlr(source[i]);
                for (int t = 0; t < qLdpc; ++t) {
                    for (int s = 0; s < 360; ++s)
                        halfScaleWords[kLdpc + qLdpc * s + t].v[lane] =
                            halveLlr(source[kLdpc + 360 * t + s]);
                }
            }
            for (int lane = active; lane < lanes; ++lane) {
                for (auto& word : halfScaleWords) word.v[lane] = word.v[0];
            }
            ldpcHalfScaleAttempts.fetch_add(static_cast<uint64_t>(active) - converged);
            const int halfScaleTrials = (*halfScaleDecoder)(
                halfScaleWords.data(), halfScaleWords.data() + kLdpc, 25, active);
            uint64_t halfScaleConverged = 0;
            if (halfScaleTrials >= 0) {
                halfScaleConverged = active;
                for (int lane = 0; lane < active; ++lane) halfScaleValidLanes[lane] = 1;
            } else {
                countUnsatisfied(halfScaleWords, halfScaleConverged, &halfScaleValidLanes);
            }
            uint64_t newlyRecovered = 0;
            for (int lane = 0; lane < active; ++lane) {
                if (!validLanes[lane] && halfScaleValidLanes[lane]) ++newlyRecovered;
            }
            converged += newlyRecovered;
            ldpcHalfScaleRecovered.fetch_add(newlyRecovered);
        }

        // A smaller second magnitude step covers the rare residual trapping
        // set that survives half-scale.  Captured-block replay showed rapid
        // convergence, so this path is both uncommon and short-lived.
        std::vector<LdpcSimd> quarterScaleWords;
        std::array<uint8_t, lanes> quarterScaleValidLanes{};
        if (!decodeCancelled() && converged < static_cast<uint64_t>(active)) {
            quarterScaleWords.assign(64800, LdpcSimd{});
            const int firstBlock = batch * lanes;
            auto quarterLlr = [](int8_t value) {
                const int widened = value;
                return static_cast<int8_t>(widened >= 0 ? (widened + 2) / 4 : (widened - 2) / 4);
            };
            for (int lane = 0; lane < active; ++lane) {
                const int8_t* source = workerFecSoftBits.data() +
                    static_cast<size_t>(firstBlock + lane) * 64800;
                for (int i = 0; i < kLdpc; ++i) quarterScaleWords[i].v[lane] = quarterLlr(source[i]);
                for (int t = 0; t < qLdpc; ++t) {
                    for (int s = 0; s < 360; ++s)
                        quarterScaleWords[kLdpc + qLdpc * s + t].v[lane] =
                            quarterLlr(source[kLdpc + 360 * t + s]);
                }
            }
            for (int lane = active; lane < lanes; ++lane) {
                for (auto& word : quarterScaleWords) word.v[lane] = word.v[0];
            }
            ldpcQuarterScaleAttempts.fetch_add(static_cast<uint64_t>(active) - converged);
            const int quarterScaleTrials = (*quarterScaleDecoder)(
                quarterScaleWords.data(), quarterScaleWords.data() + kLdpc, 25, active);
            uint64_t quarterScaleConverged = 0;
            if (quarterScaleTrials >= 0) {
                quarterScaleConverged = active;
                for (int lane = 0; lane < active; ++lane) quarterScaleValidLanes[lane] = 1;
            } else {
                countUnsatisfied(quarterScaleWords, quarterScaleConverged, &quarterScaleValidLanes);
            }
            uint64_t newlyRecovered = 0;
            for (int lane = 0; lane < active; ++lane) {
                if (!validLanes[lane] && !halfScaleValidLanes[lane] && quarterScaleValidLanes[lane])
                    ++newlyRecovered;
            }
            converged += newlyRecovered;
            ldpcQuarterScaleRecovered.fetch_add(newlyRecovered);
        }

        // Retain plain min-sum as a final LDPC alternative, but run it only for
        // lanes that the primary and both scaled paths reject.
        std::vector<LdpcSimd> alternateWords;
        std::array<uint8_t, lanes> alternateValidLanes{};
        if (!decodeCancelled() && converged < static_cast<uint64_t>(active)) {
            alternateWords.assign(64800, LdpcSimd{});
            const int firstBlock = batch * lanes;
            for (int lane = 0; lane < active; ++lane) {
                const int8_t* source = workerFecSoftBits.data() +
                    static_cast<size_t>(firstBlock + lane) * 64800;
                for (int i = 0; i < kLdpc; ++i) alternateWords[i].v[lane] = source[i];
                for (int t = 0; t < qLdpc; ++t) {
                    for (int s = 0; s < 360; ++s)
                        alternateWords[kLdpc + qLdpc * s + t].v[lane] = source[kLdpc + 360 * t + s];
                }
            }
            for (int lane = active; lane < lanes; ++lane) {
                for (auto& word : alternateWords) word.v[lane] = word.v[0];
            }
            ldpcAlternateAttempts.fetch_add(static_cast<uint64_t>(active) - converged);
            const int alternateTrials = (*alternateDecoder)(
                alternateWords.data(), alternateWords.data() + kLdpc, 30, active);
            uint64_t alternateConverged = 0;
            if (alternateTrials >= 0) {
                alternateConverged = active;
                for (int lane = 0; lane < active; ++lane) alternateValidLanes[lane] = 1;
            } else {
                countUnsatisfied(alternateWords, alternateConverged, &alternateValidLanes);
            }
            uint64_t newlyRecovered = 0;
            for (int lane = 0; lane < active; ++lane) {
                if (!validLanes[lane] && !halfScaleValidLanes[lane] &&
                    !quarterScaleValidLanes[lane] && alternateValidLanes[lane])
                    ++newlyRecovered;
            }
            converged += newlyRecovered;
            ldpcAlternateRecovered.fetch_add(newlyRecovered);
        }

        // A normal-frame rate-2/3 LDPC codeword contains a 43,200-bit BCH
        // codeword.  Its first 43,040 bits are the scrambled BBFRAME payload.
        // Validate that payload immediately with the BBHEADER CRC residue.  This
        // is also an independent end-to-end proof that the decoded LDPC word is
        // not merely a parity-valid false codeword.
        static const std::array<uint8_t, 43040> bbDescrambler = [] {
            std::array<uint8_t, 43040> sequence{};
            int state = 0x4A80;
            for (auto& bit : sequence) {
                bit = static_cast<uint8_t>((state ^ (state >> 1)) & 1);
                state >>= 1;
                if (bit) state |= 0x4000;
            }
            return sequence;
        }();
        static const bch::galois_field<uint32_t> bchField(
            bch::gf2_poly<uint32_t>(0b10000000000101101u));
        static const bch::bch_codec<uint32_t, bch::bitset256_t> bchDecoder(
            &bchField, 10, 43200);
        auto readBbBits = [](const std::array<uint8_t, 80>& bits, int first, int count) {
            int value = 0;
            for (int i = 0; i < count; ++i) value = (value << 1) | bits[first + i];
            return value;
        };
        for (int lane = 0; lane < active; ++lane) {
            std::vector<uint8_t> bbBits(43040);
            const bool halfScaleParityValid = !validLanes[lane] && halfScaleValidLanes[lane];
            const bool quarterScaleParityValid = !validLanes[lane] && !halfScaleParityValid &&
                quarterScaleValidLanes[lane];
            const bool alternateParityValid = !validLanes[lane] && !halfScaleParityValid &&
                !quarterScaleParityValid &&
                alternateValidLanes[lane];
            const auto& selectedWords = halfScaleParityValid ? halfScaleWords :
                (quarterScaleParityValid ? quarterScaleWords :
                    (alternateParityValid ? alternateWords : words));
            if (validLanes[lane] || halfScaleParityValid || quarterScaleParityValid ||
                alternateParityValid) {
                for (int i = 0; i < 43040; ++i)
                    bbBits[i] = static_cast<uint8_t>((selectedWords[i].v[lane] < 0) ^ bbDescrambler[i]);
            } else {
                // The DVB-T2 rate-2/3 normal FECFRAME uses shortened
                // BCH(43200,43040), t=10 outside the LDPC code.  Apply it only
                // to a lane that still has a residual LDPC syndrome.
                std::array<uint8_t, 5400> codeword{};
                std::array<uint8_t, 5380> decoded{};
                bchRecoveryAttempts.fetch_add(1);
                auto tryBch = [&](const std::vector<LdpcSimd>& candidate) {
                    codeword.fill(0);
                    for (int bit = 0; bit < 43200; ++bit) {
                        if (candidate[bit].v[lane] < 0)
                            codeword[bit / 8] |= static_cast<uint8_t>(0x80u >> (bit & 7));
                    }
                    return bchDecoder.decode(codeword.data(), decoded.data(), true);
                };
                int corrected = tryBch(words);
                if (corrected < 0 && !halfScaleWords.empty()) corrected = tryBch(halfScaleWords);
                if (corrected < 0 && !quarterScaleWords.empty()) corrected = tryBch(quarterScaleWords);
                if (corrected < 0 && !alternateWords.empty()) corrected = tryBch(alternateWords);
                if (corrected < 0) {
                    // Preserve the first truly unrecoverable lane exactly as it
                    // entered LDPC decoding.  It can then be replayed offline
                    // while tuning algorithms, iteration limits, and LLR scale.
                    const int captureIndex = residualFecCapturesAttempted.load();
                    if (captureIndex < 16) {
                        residualFecCapturesAttempted.fetch_add(1);
                        struct ResidualFecTraceHeader {
                            uint32_t magic;
                            uint32_t version;
                            int32_t fecBits;
                            int32_t informationBits;
                            int32_t fecBlock;
                            int32_t dynamicFrame;
                            int32_t modulation;
                            int32_t codeRate;
                            int32_t rotation;
                            float snrDb;
                        } traceHeader{
                            0x45463252u, 1u, 64800, kLdpc, batch * lanes + lane,
                            workerDynamicFrame.load(), modulation, codeRate, rotation,
                            qamSnrDb.load()
                        };
                        std::error_code error;
                        std::filesystem::create_directories("recordings", error);
                        const std::string capturePath = "recordings/dvbt2_residual_fec_" +
                            std::to_string(captureIndex) + ".bin";
                        std::ofstream output(capturePath,
                                             std::ios::binary | std::ios::trunc);
                        if (output) {
                            const int fecBlock = batch * lanes + lane;
                            const int8_t* source = workerFecSoftBits.data() +
                                static_cast<size_t>(fecBlock) * 64800;
                            output.write(reinterpret_cast<const char*>(&traceHeader), sizeof(traceHeader));
                            output.write(reinterpret_cast<const char*>(source), 64800);
                            if (output.good()) residualFecCapturesSaved.fetch_add(1);
                        }
                    }
                    continue;
                }
                bchRecoveredBlocks.fetch_add(1);
                bchCorrectedBits.fetch_add(static_cast<uint64_t>(corrected));
                for (int i = 0; i < 43040; ++i) {
                    const uint8_t bit = static_cast<uint8_t>(
                        (decoded[i / 8] >> (7 - (i & 7))) & 1u);
                    bbBits[i] = static_cast<uint8_t>(bit ^ bbDescrambler[i]);
                }
            }
            std::array<uint8_t, 80> header{};
            std::copy_n(bbBits.begin(), 80, header.begin());
            uint8_t residue = 0;
            for (const uint8_t bit : header) {
                const uint8_t feedback = static_cast<uint8_t>(bit ^ (residue & 1u));
                residue >>= 1;
                if (feedback) residue ^= 0xABu;
            }
            bbHeadersTested.fetch_add(1);
            if (residue != 0 && residue != 0xABu) continue;
            bbHeadersValid.fetch_add(1);
            bbInputMode.store(residue == 0 ? 0 : 1);
            bbTsGs.store(readBbBits(header, 0, 2));
            const bool npd = header[5] != 0;
            bbNpd.store(npd);
            bbUpl.store(readBbBits(header, 16, 16));
            const int dfl = readBbBits(header, 32, 16);
            bbDfl.store(dfl);
            bbSync.store(readBbBits(header, 48, 8));
            const int syncd = readBbBits(header, 56, 16);
            bbSyncd.store(syncd);
            if (residue == 0xABu && bbTsGs.load() == 3) {
                const int fecBlock = batch * lanes + lane;
                const int firstTiBlocks = timeLength > 0 ? plpBlocks / timeLength : plpBlocks;
                processHighEfficiencyBbFrame(bbBits, dfl, syncd, npd, fecBlock, firstTiBlocks);
            }
        }
        frameConverged += converged;
        frameTested += active;
        frameDecodedUnsatisfied += static_cast<double>(meanUnsatisfied) * active;
        }
        ldpcBlocksTested.store(frameTested);
        ldpcBlocksConverged.store(frameConverged);
        ldpcFirstBlockTested.store(0);
        ldpcMeanRawUnsatisfied.store(0.0f);
        ldpcMeanUnsatisfied.store(frameTested > 0 ? static_cast<float>(frameDecodedUnsatisfied / frameTested) : 0.0f);
        pendingLdpcMilliseconds = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - ldpcStarted).count();
    }

    void frameWorkerLoop() {
        lowerDecoderThreadPriority();
        while (true) {
            int sampleCount = 0;
            int boundary = 0;
            int guard = 0;
            int symbolCount = 0;
            int carrierShift = 0;
            int plpStart = 0;
            int plpBlocks = 0;
            int plpModulation = 0;
            int plpCode = 0;
            int plpFecType = 0;
            int plpRotation = 0;
            int plpMaxBlocks = 0;
            int timeIlLength = 0;
            int timeIlType = 0;
            int frequencyMapping = 0;
            int equalizerVariant = 0;
            uint64_t frameGeneration = 0;
            double frequency = 0.0;
            std::vector<dsp::complex_t> frameCells;
            std::chrono::steady_clock::time_point frameStarted;
            {
                std::unique_lock<std::mutex> lock(frameWorkerMutex);
                frameWorkerCv.wait(lock, [&] { return frameWorkerStop || frameWorkPending; });
                if (frameWorkerStop) return;
                frameWorkPending = false;
                frameWorkerBusy = true;
                sampleCount = workerFrameSamples;
                frameGeneration = workerFrameGeneration;
                activeFrameGeneration.store(frameGeneration);
                boundary = workerBoundary;
                guard = workerGuard;
                symbolCount = workerSymbolCount;
                frequency = workerFrequency;
                carrierShift = workerCarrierShift;
                plpStart = workerPlpStart;
                plpBlocks = workerPlpBlocks;
                plpModulation = workerPlpModulation;
                plpCode = workerPlpCode;
                plpFecType = workerPlpFecType;
                plpRotation = workerPlpRotation;
                plpMaxBlocks = workerPlpMaxBlocks;
                timeIlLength = workerTimeIlLength;
                timeIlType = workerTimeIlType;
                frequencyMapping = workerFrequencyMapping;
                equalizerVariant = 0;
                frameCells = workerP2Payload;
            }
            // Start timing after the condition-variable wait.  Measuring before
            // it incorrectly counted idle time between captured RF frames as
            // decoder work and produced apparent multi-second spikes.
            frameStarted = std::chrono::steady_clock::now();

            if (decodeCancelled()) {
                abandonCancelledFrame();
                continue;
            }

            constexpr int fftSize = 32768;
            constexpr int fftLag = 35840;
            constexpr double inputPerDvbSample = INPUT_SAMPLE_RATE / DVB_SAMPLE_RATE;
            const int symbolLength = fftLag + guard;
            const int ordinaryDataSymbols = (std::max)(symbolCount - dataParams.l_fc, 0);
            int equalizedSymbols = 0;
            uint64_t equalizedCells = 0;
            double pilotMetricSum = 0.0;
            frameCells.reserve(frameCells.size() + 1100000);
            const uint64_t p2SectionCells = frameCells.size();
            std::vector<dsp::complex_t> traceRawCells;
            std::vector<dsp::complex_t> traceMappedCells;
            if (frequencyMapping == 0 && !debugAllSymbolsWritten.load()) {
                traceRawCells.reserve(1100000);
                traceMappedCells.reserve(1100000);
            }
            std::vector<dsp::complex_t> previousSymbolChannel(dataParams.k_total);
            std::vector<uint8_t> previousSymbolChannelValid(dataParams.k_total, 0);
            // Reuse the large per-symbol work buffers.  Reallocating these for
            // every OFDM symbol adds allocator traffic to the hottest serial
            // section of the frame worker without changing their required size.
            std::vector<dsp::complex_t> pilotChannels(dataParams.k_total);
            std::vector<dsp::complex_t> nextSymbolChannel(dataParams.k_total);
            std::vector<uint8_t> nextSymbolChannelValid(dataParams.k_total, 0);
            std::vector<dsp::complex_t> pendingValues;
            std::vector<int> pendingCarriers;
            std::vector<dsp::complex_t> symbolEqualized;
            std::vector<dsp::complex_t> frequencyDeinterleaved(dataParams.c_data);
            pendingValues.reserve(16);
            pendingCarriers.reserve(16);
            symbolEqualized.reserve(dataParams.c_data);
            float resampleMilliseconds = 0.0f;
            float fftMilliseconds = 0.0f;
            float equalizeMilliseconds = 0.0f;

            // Resampling dominates the OFDM stage but every symbol is
            // independent.  Prepare all ordinary symbols concurrently, then
            // feed the single FFTW plan in order.  This keeps FFT/equalizer
            // state deterministic while using the otherwise idle CPU cores.
            int processSymbols = ordinaryDataSymbols;
            for (int symbol = 0; symbol < ordinaryDataSymbols; ++symbol) {
                const int usefulStart = boundary + (dataParams.n_p2 + symbol) * symbolLength + guard;
                if (usefulStart + static_cast<int>(std::ceil((fftSize - 1) * inputPerDvbSample)) + 1 >= sampleCount) {
                    processSymbols = symbol;
                    break;
                }
            }
            std::vector<dsp::complex_t> resampledSymbols(
                static_cast<size_t>(processSymbols) * fftSize);
            const auto resampleStarted = std::chrono::steady_clock::now();
            const int resampleWorkers = computeWorkerLimit;
            const int activeResampleWorkers = (std::min)(resampleWorkers, processSymbols);
            runComputeWorkers(activeResampleWorkers, [&](int worker) {
                for (int symbol = worker; symbol < processSymbols; symbol += activeResampleWorkers) {
                    if (decodeCancelled()) break;
                    const int usefulStart = boundary + (dataParams.n_p2 + symbol) * symbolLength + guard;
                    const double phase0 = -2.0 * PI * frequency * usefulStart / INPUT_SAMPLE_RATE;
                    const double phaseStep = -2.0 * PI * frequency * inputPerDvbSample / INPUT_SAMPLE_RATE;
                    const double stepRe = std::cos(phaseStep);
                    const double stepIm = std::sin(phaseStep);
                    double oscillatorRe = std::cos(phase0);
                    double oscillatorIm = std::sin(phase0);
                    auto* destination = resampledSymbols.data() + static_cast<size_t>(symbol) * fftSize;
                    for (int i = 0; i < fftSize; ++i) {
                        const double position = usefulStart + i * inputPerDvbSample;
                        const auto sample = resampleDvbAt(frameWorkBuffer.data(), sampleCount, position);
                        const float cs = static_cast<float>(oscillatorRe);
                        const float sn = static_cast<float>(oscillatorIm);
                        destination[i] = {sample.re * cs - sample.im * sn,
                                          sample.re * sn + sample.im * cs};
                        const double nextRe = oscillatorRe * stepRe - oscillatorIm * stepIm;
                        oscillatorIm = oscillatorRe * stepIm + oscillatorIm * stepRe;
                        oscillatorRe = nextRe;
                    }
                }
            });
            if (decodeCancelled()) {
                abandonCancelledFrame();
                continue;
            }
            resampleMilliseconds = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - resampleStarted).count();

            for (int symbol = 0; symbol < processSymbols; ++symbol) {
                std::copy_n(resampledSymbols.data() + static_cast<size_t>(symbol) * fftSize,
                            fftSize, workerFftInput);
                const auto fftStarted = std::chrono::steady_clock::now();
                fftwf_execute(workerFftPlan);
                const auto equalizeStarted = std::chrono::steady_clock::now();
                fftMilliseconds += std::chrono::duration<float, std::milli>(
                    equalizeStarted - fftStarted).count();

                const int* map = p2Pilots.data_carrier_map[symbol];
                const float* references = p2Pilots.data_pilot_refer[symbol];
                dsp::complex_t timeCorrelation{0.0f, 0.0f};
                for (int carrier = 0; carrier < dataParams.k_total; ++carrier) {
                    if (map[carrier] == DATA_CARRIER || map[carrier] == TRPAPR_CARRIER ||
                        references[carrier] == 0.0f) continue;
                    const int shiftedBin = dataParams.l_nulls + carrier;
                    const int rawBin = (shiftedBin + fftSize / 2 + carrierShift) & (fftSize - 1);
                    const auto& pilot = workerFftOutput[rawBin];
                    const float inverseReference = 1.0f / references[carrier];
                    const dsp::complex_t channel{pilot.re * inverseReference, pilot.im * inverseReference};
                    pilotChannels[carrier] = channel;
                    if (previousSymbolChannelValid[carrier]) {
                        const auto product = multiplyConjugate(channel, previousSymbolChannel[carrier]);
                        timeCorrelation.re += product.re;
                        timeCorrelation.im += product.im;
                    }
                }
                const float timePhase = std::atan2(timeCorrelation.im, timeCorrelation.re);
                const float timeCs = std::cos(timePhase), timeSn = std::sin(timePhase);
                std::fill(nextSymbolChannelValid.begin(), nextSymbolChannelValid.end(), 0);
                dsp::complex_t previous{0.0f, 0.0f};
                dsp::complex_t correlationSum{0.0f, 0.0f};
                double previousEnergy = 0.0;
                double currentEnergy = 0.0;
                bool havePrevious = false;
                int previousPilotCarrier = -1;
                int pilotPairs = 0;
                uint64_t symbolCells = 0;
                pendingValues.clear();
                pendingCarriers.clear();
                symbolEqualized.clear();
                for (int carrier = 0; carrier < dataParams.k_total; ++carrier) {
                    const int shiftedBin = dataParams.l_nulls + carrier;
                    const int rawBin = (shiftedBin + fftSize / 2 + carrierShift) & (fftSize - 1);
                    const auto& cell = workerFftOutput[rawBin];
                    if (map[carrier] == DATA_CARRIER) {
                        pendingValues.push_back(cell);
                        pendingCarriers.push_back(carrier);
                        continue;
                    }
                    if (map[carrier] == TRPAPR_CARRIER || references[carrier] == 0.0f) continue;
                    const dsp::complex_t channel = pilotChannels[carrier];
                    nextSymbolChannel[carrier] = channel;
                    nextSymbolChannelValid[carrier] = 1;
                    if (havePrevious) {
                        const auto product = multiplyConjugate(channel, previous);
                        correlationSum.re += product.re;
                        correlationSum.im += product.im;
                        currentEnergy += magnitudeSquared(channel);
                        previousEnergy += magnitudeSquared(previous);
                        ++pilotPairs;
                        const int carrierSpan = carrier - previousPilotCarrier;
                        const dsp::complex_t left = equalizerVariant == 0 ? previous : dsp::complex_t{
                            0.75f * previous.re + 0.25f * channel.re,
                            0.75f * previous.im + 0.25f * channel.im};
                        const dsp::complex_t right = equalizerVariant == 0 ? channel : dsp::complex_t{
                            0.25f * previous.re + 0.75f * channel.re,
                            0.25f * previous.im + 0.75f * channel.im};
                        for (int j = 0; j < static_cast<int>(pendingValues.size()); ++j) {
                            const float t = carrierSpan > 0
                                ? static_cast<float>(pendingCarriers[j] - previousPilotCarrier) / carrierSpan
                                : 0.5f;
                            dsp::complex_t estimate = interpolateChannelPolar(left, right, t);
                            const int dataCarrier = pendingCarriers[j];
                            if (equalizerVariant == 2 && previousSymbolChannelValid[dataCarrier] &&
                                (timeCorrelation.re != 0.0f || timeCorrelation.im != 0.0f)) {
                                const auto& old = previousSymbolChannel[dataCarrier];
                                const dsp::complex_t aligned{
                                    old.re * timeCs - old.im * timeSn,
                                    old.re * timeSn + old.im * timeCs};
                                estimate.re = 0.5f * (estimate.re + aligned.re);
                                estimate.im = 0.5f * (estimate.im + aligned.im);
                            }
                            nextSymbolChannel[dataCarrier] = estimate;
                            nextSymbolChannelValid[dataCarrier] = 1;
                            const float power = magnitudeSquared(estimate);
                            if (power > 1.0e-20f) {
                                const auto& value = pendingValues[j];
                                const float equalizedRe = (value.re * estimate.re + value.im * estimate.im) / power;
                                const float equalizedIm = (value.im * estimate.re - value.re * estimate.im) / power;
                                if (std::isfinite(equalizedRe) && std::isfinite(equalizedIm)) {
                                    symbolEqualized.push_back({equalizedRe, equalizedIm});
                                    ++symbolCells;
                                }
                            }
                        }
                    }
                    pendingValues.clear();
                    pendingCarriers.clear();
                    previous = channel;
                    previousPilotCarrier = carrier;
                    havePrevious = true;
                }
                previousSymbolChannel.swap(nextSymbolChannel);
                previousSymbolChannelValid.swap(nextSymbolChannelValid);
                const float metric = pilotPairs > 0 ? static_cast<float>(
                    std::sqrt(magnitudeSquared(correlationSum)) /
                    std::sqrt((std::max)(currentEnergy * previousEnergy, 1.0e-20))) : 0.0f;
                pilotMetricSum += metric;
                if (metric >= 0.35f) {
                    normalizeQamCells(symbolEqualized);
                    if (frequencyMapping >= 3) std::reverse(symbolEqualized.begin(), symbolEqualized.end());
                    ++equalizedSymbols;
                    equalizedCells += symbolCells;
                    if (frequencyMapping == 2) {
                        frameCells.insert(frameCells.end(), symbolEqualized.begin(), symbolEqualized.end());
                    } else {
                        const int absoluteSymbol = dataParams.n_p2 + symbol;
                        const bool swappedMapping = frequencyMapping == 1 || frequencyMapping == 4;
                        const bool useOdd = (absoluteSymbol % 2 == 0) ^ swappedMapping;
                        const int* frequencyAddress = useOdd ? p2Addresses.h_odd_data : p2Addresses.h_even_data;
                        std::fill(frequencyDeinterleaved.begin(), frequencyDeinterleaved.end(), dsp::complex_t{});
                        const int count = (std::min)(static_cast<int>(symbolEqualized.size()), dataParams.c_data);
                        if (frequencyMapping == 5) {
                            for (int i = 0; i < count; ++i) frequencyDeinterleaved[i] = symbolEqualized[frequencyAddress[i]];
                        } else {
                            for (int i = 0; i < count; ++i) frequencyDeinterleaved[frequencyAddress[i]] = symbolEqualized[i];
                        }
                        if (!traceRawCells.empty() || (symbol == 0 && frequencyMapping == 0 && !debugAllSymbolsWritten.load())) {
                            traceRawCells.insert(traceRawCells.end(), symbolEqualized.begin(), symbolEqualized.begin() + count);
                            traceMappedCells.insert(traceMappedCells.end(), frequencyDeinterleaved.begin(), frequencyDeinterleaved.begin() + count);
                        }
                        if (symbol == 0 && frequencyMapping == 0 && !debugSymbolWritten.exchange(true)) {
                            struct SymbolHeader {
                                uint32_t magic, version;
                                int32_t dataSymbol, absoluteSymbol, cellCount, useOdd;
                            } symbolHeader{0x324D5953u, 1u, symbol, absoluteSymbol, count, useOdd ? 1 : 0};
                            std::error_code error;
                            std::filesystem::create_directories("recordings", error);
                            std::ofstream output("recordings/dvbt2_debug_symbol.bin", std::ios::binary | std::ios::trunc);
                            if (output) {
                                output.write(reinterpret_cast<const char*>(&symbolHeader), sizeof(symbolHeader));
                                output.write(reinterpret_cast<const char*>(symbolEqualized.data()),
                                             static_cast<std::streamsize>(count * sizeof(dsp::complex_t)));
                                output.write(reinterpret_cast<const char*>(frequencyDeinterleaved.data()),
                                             static_cast<std::streamsize>(count * sizeof(dsp::complex_t)));
                                debugSymbolSaved.store(output.good());
                            }
                        }
                        frameCells.insert(frameCells.end(), frequencyDeinterleaved.begin(), frequencyDeinterleaved.end());
                    }
                }
                equalizeMilliseconds += std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - equalizeStarted).count();
            }

            const uint64_t dataSectionCells = frameCells.size() - p2SectionCells;
            const uint64_t beforeClosingCells = frameCells.size();

            if (dataParams.l_fc && p2Pilots.fc_carrier_map != nullptr) {
                const int usefulStart = boundary +
                    (dataParams.n_p2 + ordinaryDataSymbols) * symbolLength + guard;
                if (usefulStart + static_cast<int>(std::ceil((fftSize - 1) * inputPerDvbSample)) + 1 < sampleCount) {
                    const double phase0 = -2.0 * PI * frequency * usefulStart / INPUT_SAMPLE_RATE;
                    const double phaseStep = -2.0 * PI * frequency * inputPerDvbSample / INPUT_SAMPLE_RATE;
                    const double stepRe = std::cos(phaseStep);
                    const double stepIm = std::sin(phaseStep);
                    double oscillatorRe = std::cos(phase0);
                    double oscillatorIm = std::sin(phase0);
                    for (int i = 0; i < fftSize; ++i) {
                        const double position = usefulStart + i * inputPerDvbSample;
                        const auto sample = resampleDvbAt(frameWorkBuffer.data(), sampleCount, position);
                        const float cs = static_cast<float>(oscillatorRe);
                        const float sn = static_cast<float>(oscillatorIm);
                        workerFftInput[i] = {sample.re * cs - sample.im * sn, sample.re * sn + sample.im * cs};
                        const double nextRe = oscillatorRe * stepRe - oscillatorIm * stepIm;
                        oscillatorIm = oscillatorRe * stepIm + oscillatorIm * stepRe;
                        oscillatorRe = nextRe;
                    }
                    fftwf_execute(workerFftPlan);
                    dsp::complex_t previous{0.0f, 0.0f};
                    dsp::complex_t sum{0.0f, 0.0f};
                    double previousEnergy = 0.0;
                    double currentEnergy = 0.0;
                    bool havePrevious = false;
                    int previousPilotCarrier = -1;
                    int pairs = 0;
                    int dataCells = 0;
                    std::vector<dsp::complex_t> pendingValues;
                    std::vector<int> pendingCarriers;
                    std::vector<dsp::complex_t> closingEqualized;
                    pendingValues.reserve(16);
                    pendingCarriers.reserve(16);
                    closingEqualized.reserve(dataParams.n_fc);
                    for (int carrier = 0; carrier < dataParams.k_total; ++carrier) {
                        const int shiftedBin = dataParams.l_nulls + carrier;
                        const int rawBin = (shiftedBin + fftSize / 2 + carrierShift) & (fftSize - 1);
                        const auto& cell = workerFftOutput[rawBin];
                        if (p2Pilots.fc_carrier_map[carrier] == DATA_CARRIER) {
                            pendingValues.push_back(cell);
                            pendingCarriers.push_back(carrier);
                            continue;
                        }
                        if (p2Pilots.fc_carrier_map[carrier] == TRPAPR_CARRIER ||
                            p2Pilots.fc_pilot_refer[carrier] == 0.0f) continue;
                        const float reference = p2Pilots.fc_pilot_refer[carrier];
                        const float inverseReference = 1.0f / reference;
                        const dsp::complex_t channel{cell.re * inverseReference, cell.im * inverseReference};
                        if (havePrevious) {
                            const auto product = multiplyConjugate(channel, previous);
                            sum.re += product.re;
                            sum.im += product.im;
                            currentEnergy += magnitudeSquared(channel);
                            previousEnergy += magnitudeSquared(previous);
                            ++pairs;
                            const int carrierSpan = carrier - previousPilotCarrier;
                            const dsp::complex_t left = equalizerVariant == 0 ? previous : dsp::complex_t{
                                0.75f * previous.re + 0.25f * channel.re,
                                0.75f * previous.im + 0.25f * channel.im};
                            const dsp::complex_t right = equalizerVariant == 0 ? channel : dsp::complex_t{
                                0.25f * previous.re + 0.75f * channel.re,
                                0.25f * previous.im + 0.75f * channel.im};
                            for (int j = 0; j < static_cast<int>(pendingValues.size()); ++j) {
                                const float t = carrierSpan > 0
                                    ? static_cast<float>(pendingCarriers[j] - previousPilotCarrier) / carrierSpan
                                    : 0.5f;
                                const dsp::complex_t estimate = interpolateChannelPolar(left, right, t);
                                const float power = magnitudeSquared(estimate);
                                if (power > 1.0e-20f) {
                                    const auto& value = pendingValues[j];
                                    const dsp::complex_t equalized{
                                        (value.re * estimate.re + value.im * estimate.im) / power,
                                        (value.im * estimate.re - value.re * estimate.im) / power
                                    };
                                    if (std::isfinite(equalized.re) && std::isfinite(equalized.im)) {
                                        closingEqualized.push_back(equalized);
                                        ++dataCells;
                                    }
                                }
                            }
                        }
                        pendingValues.clear();
                        pendingCarriers.clear();
                        previous = channel;
                        previousPilotCarrier = carrier;
                        havePrevious = true;
                    }
                    const float metric = pairs > 0 ? static_cast<float>(std::sqrt(magnitudeSquared(sum)) /
                        std::sqrt((std::max)(currentEnergy * previousEnergy, 1.0e-20))) : 0.0f;
                    frameClosingPilotCorrelation.store(metric);
                    frameClosingAcquired.store(metric >= 0.35f);
                    frameClosingCells.store(metric >= 0.35f ? dataCells : 0);
                    if (metric >= 0.35f) {
                        normalizeQamCells(closingEqualized);
                        if (frequencyMapping >= 3) std::reverse(closingEqualized.begin(), closingEqualized.end());
                        if (frequencyMapping == 2) {
                            frameCells.insert(frameCells.end(), closingEqualized.begin(), closingEqualized.end());
                        } else {
                            const int closingSymbol = dataParams.len_frame - 1;
                            const bool swappedMapping = frequencyMapping == 1 || frequencyMapping == 4;
                            const bool useOdd = (closingSymbol % 2 == 0) ^ swappedMapping;
                            const int* frequencyAddress = useOdd ? p2Addresses.h_odd_fc : p2Addresses.h_even_fc;
                            std::vector<dsp::complex_t> frequencyDeinterleaved(dataParams.n_fc);
                            const int count = (std::min)(static_cast<int>(closingEqualized.size()), dataParams.n_fc);
                            if (frequencyMapping == 5) {
                                for (int i = 0; i < count; ++i) frequencyDeinterleaved[i] = closingEqualized[frequencyAddress[i]];
                            } else {
                                for (int i = 0; i < count; ++i) frequencyDeinterleaved[frequencyAddress[i]] = closingEqualized[i];
                            }
                            if (!traceRawCells.empty()) {
                                traceRawCells.insert(traceRawCells.end(), closingEqualized.begin(), closingEqualized.begin() + count);
                                traceMappedCells.insert(traceMappedCells.end(), frequencyDeinterleaved.begin(), frequencyDeinterleaved.begin() + count);
                            }
                            frameCells.insert(frameCells.end(), frequencyDeinterleaved.begin(), frequencyDeinterleaved.end());
                        }
                    }
                }
            }

            const uint64_t closingSectionCells = frameCells.size() - beforeClosingCells;
            frameP2SectionCells.store(p2SectionCells);
            frameDataSectionCells.store(dataSectionCells);
            frameClosingSectionCells.store(closingSectionCells);
            if (frequencyMapping == 0 && !traceRawCells.empty() &&
                traceRawCells.size() == traceMappedCells.size() && !debugAllSymbolsWritten.exchange(true)) {
                struct AllSymbolsHeader {
                    uint32_t magic, version;
                    int32_t ordinarySymbols, dataCellsPerSymbol, closingCells, nP2, closingAbsoluteSymbol;
                    uint64_t totalCells;
                } traceHeader{0x324C4C41u, 1u, ordinaryDataSymbols, dataParams.c_data,
                              static_cast<int32_t>(closingSectionCells), dataParams.n_p2,
                              dataParams.len_frame - 1, static_cast<uint64_t>(traceRawCells.size())};
                std::error_code error;
                std::filesystem::create_directories("recordings", error);
                std::ofstream output("recordings/dvbt2_debug_all_symbols.bin", std::ios::binary | std::ios::trunc);
                if (output) {
                    output.write(reinterpret_cast<const char*>(&traceHeader), sizeof(traceHeader));
                    output.write(reinterpret_cast<const char*>(traceRawCells.data()),
                                 static_cast<std::streamsize>(traceRawCells.size() * sizeof(dsp::complex_t)));
                    output.write(reinterpret_cast<const char*>(traceMappedCells.data()),
                                 static_cast<std::streamsize>(traceMappedCells.size() * sizeof(dsp::complex_t)));
                    debugAllSymbolsSaved.store(output.good());
                }
            }

            if (decodeCancelled()) {
                abandonCancelledFrame();
                continue;
            }

            const int bitsPerCell = (plpModulation >= 0 && plpModulation <= 3) ? 2 * (plpModulation + 1) : 0;
            const int fecBits = plpFecType == 1 ? 64800 : 16200;
            const uint64_t requiredCells = bitsPerCell > 0
                ? static_cast<uint64_t>(plpBlocks) * static_cast<uint64_t>(fecBits / bitsPerCell) : 0;
            const uint64_t availableCells = frameCells.size();
            const uint64_t extractable = plpStart >= 0 && static_cast<uint64_t>(plpStart) < availableCells
                ? (std::min)(requiredCells, availableCells - static_cast<uint64_t>(plpStart)) : 0;
            plpCellsAvailable.store(availableCells);
            plpCellsRequired.store(requiredCells);
            plpCellsExtracted.store(extractable);
            const bool validDebugFrame = requiredCells > 0 && extractable == requiredCells &&
                plpBlocks > 0 && plpModulation == 3 && plpFecType == 1 &&
                plpMaxBlocks > 0 && timeIlLength > 0 && timeIlType >= 0;
            if (frequencyMapping == 0 && validDebugFrame && !debugFrameWritten.exchange(true)) {
                struct DebugHeader {
                    uint32_t magic;
                    uint32_t version;
                    int32_t plpStart, plpBlocks, modulation, codeRate, fecType, rotation;
                    int32_t maxBlocks, timeLength, timeType, frequencyMapping;
                    uint64_t cellCount;
                    uint64_t p2Cells, dataCells, closingCells;
                } header{0x32545644u, 2u, plpStart, plpBlocks, plpModulation, plpCode,
                         plpFecType, plpRotation, plpMaxBlocks, timeIlLength, timeIlType,
                         frequencyMapping, static_cast<uint64_t>(frameCells.size()),
                         p2SectionCells, dataSectionCells, closingSectionCells};
                std::error_code error;
                std::filesystem::create_directories("recordings", error);
                std::ofstream output("recordings/dvbt2_debug_frame.bin", std::ios::binary | std::ios::trunc);
                if (output) {
                    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
                    output.write(reinterpret_cast<const char*>(frameCells.data()),
                                 static_cast<std::streamsize>(frameCells.size() * sizeof(dsp::complex_t)));
                    debugFrameSaved.store(output.good());
                }
            }
            if (requiredCells > 0 && extractable == requiredCells) {
                const float ofdmElapsed = std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - frameStarted).count();
                const uint64_t captured = (std::min)(requiredCells + 8,
                    availableCells - static_cast<uint64_t>(plpStart));
                workerExtractedPlp.assign(frameCells.begin() + plpStart,
                                          frameCells.begin() + plpStart + static_cast<std::ptrdiff_t>(captured));
                plpFramesAssembled.fetch_add(1);
                timeDeinterleaveAndDemap(plpBlocks, plpModulation, plpCode, plpFecType, plpRotation,
                                         plpMaxBlocks, timeIlLength, timeIlType, frequencyMapping);
                if (decodeCancelled()) {
                    abandonCancelledFrame();
                    continue;
                }
                const float elapsed = std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - frameStarted).count();
                // Publish every stage only after this same full frame has
                // completed, so the UI cannot mix adjacent-frame timings.
                workerOfdmMilliseconds.store(ofdmElapsed);
                workerDemapMilliseconds.store(pendingDemapMilliseconds);
                workerLdpcMilliseconds.store(pendingLdpcMilliseconds);
                workerResampleMilliseconds.store(resampleMilliseconds);
                workerFftMilliseconds.store(fftMilliseconds);
                workerEqualizeMilliseconds.store(equalizeMilliseconds);
                workerFrameMilliseconds.store(elapsed);
                const float previousAverage = workerFrameAverageMilliseconds.load();
                workerFrameAverageMilliseconds.store(previousAverage == 0.0f
                    ? elapsed : previousAverage * 0.9f + elapsed * 0.1f);
                workerFrameMaximumMilliseconds.store((std::max)(
                    workerFrameMaximumMilliseconds.load(), elapsed));
            }

            workerSymbolsEqualized.store(equalizedSymbols);
            workerDataCells.store(equalizedCells);
            workerExpectedDataSymbols.store(ordinaryDataSymbols);
            workerMeanPilotCorrelation.store(ordinaryDataSymbols > 0 ? static_cast<float>(pilotMetricSum / ordinaryDataSymbols) : 0.0f);
            workerFramesProcessed.fetch_add(1);
            {
                std::lock_guard<std::mutex> lock(frameWorkerMutex);
                if (queuedFrameCount > 0) {
                    auto& queued = queuedFrames[queuedFrameHead];
                    frameWorkBuffer.swap(queued.samples);
                    workerFrameSamples = queued.sampleCount;
                    workerFrameGeneration = queued.generation;
                    workerBoundary = queued.boundary;
                    workerGuard = queued.guard;
                    workerSymbolCount = queued.symbolCount;
                    workerFrequency = queued.frequency;
                    workerCarrierShift = queued.carrierShift;
                    workerP2Payload = std::move(queued.p2Payload);
                    workerDynamicFrame.store(queued.dynamicFrame);
                    workerPlpStart = queued.plpStart;
                    workerPlpBlocks = queued.plpBlocks;
                    workerPlpModulation = queued.plpModulation;
                    workerPlpCode = queued.plpCode;
                    workerPlpFecType = queued.plpFecType;
                    workerPlpRotation = queued.plpRotation;
                    workerPlpMaxBlocks = queued.plpMaxBlocks;
                    workerTimeIlLength = queued.timeIlLength;
                    workerTimeIlType = queued.timeIlType;
                    workerFrequencyMapping = 0;
                    queuedFrameHead = (queuedFrameHead + 1) % FRAME_QUEUE_CAPACITY;
                    --queuedFrameCount;
                    workerQueueDepth.store(queuedFrameCount);
                    frameWorkPending = true;
                } else {
                    frameWorkPending = false;
                }
                frameWorkerBusy = false;
            }
        }
    }

    void decodeL1Pre(int usefulStart, double frequency) {
        constexpr int fftSize = 32768;
        constexpr double inputPerDvbSample = INPUT_SAMPLE_RATE / DVB_SAMPLE_RATE;
        if (!p2FftPlan || usefulStart < 0 ||
            usefulStart + static_cast<int>(std::ceil((fftSize - 1) * inputPerDvbSample)) + 1 >= P2_CAPTURE_SAMPLES) {
            l1LiveValid.store(false);
            return;
        }
        l1CrcAttempts.fetch_add(1);

        for (int i = 0; i < fftSize; ++i) {
            const double position = usefulStart + i * inputPerDvbSample;
            const auto sample = resampleDvbAt(p2Capture.data(), P2_CAPTURE_SAMPLES, position);
            const double phase = -2.0 * PI * frequency * position / INPUT_SAMPLE_RATE;
            const float cs = static_cast<float>(std::cos(phase));
            const float sn = static_cast<float>(std::sin(phase));
            p2FftInput[i] = {sample.re * cs - sample.im * sn, sample.re * sn + sample.im * cs};
        }
        fftwf_execute(p2FftPlan);

        const int* map = p2Pilots.p2_carrier_map;
        const float* references = p2Pilots.p2_pilot_refer[0];
        std::array<uint8_t, 200> bits{};
        std::vector<dsp::complex_t> deinterleaved(dvbt2Params.c_p2);
        std::vector<dsp::complex_t> pending;
        pending.reserve(6);

        float bestDecisionQuality = 0.0f;
        int bestCandidateShift = 0;
        bool bestCandidateMirrored = false;
        bool bestCandidateEven = false;
        std::array<float, 200> bestSoft{};
        auto tryAlignment = [&](int binShift, bool mirrored, bool evenPhase, bool polarInterpolation) {
            const int* addresses = evenPhase ? p2Addresses.h_even_p2 : p2Addresses.h_odd_p2;
            std::fill(deinterleaved.begin(), deinterleaved.end(), dsp::complex_t{0.0f, 0.0f});
            pending.clear();
            dsp::complex_t previousChannel{0.0f, 0.0f};
            bool havePreviousPilot = false;
            int dataIndex = 0;
            for (int carrier = 0; carrier < dvbt2Params.k_total; ++carrier) {
                int shiftedBin = dvbt2Params.l_nulls + carrier;
                if (mirrored) shiftedBin = fftSize - shiftedBin;
                const int rawBin = (shiftedBin + fftSize / 2 + binShift) & (fftSize - 1);
                const dsp::complex_t cell = p2FftOutput[rawBin];
                if (map[carrier] == DATA_CARRIER) {
                    pending.push_back(cell);
                    continue;
                }
                if (map[carrier] != P2CARRIER && map[carrier] != P2CARRIER_INVERTED) continue;

                const float reference = references[carrier];
                const float inverseReference = 1.0f / reference;
                const dsp::complex_t channel{cell.re * inverseReference, cell.im * inverseReference};
                if (havePreviousPilot) {
                    const int span = static_cast<int>(pending.size()) + 1;
                    float previousAmplitude = 0.0f;
                    float channelAmplitude = 0.0f;
                    float previousPhase = 0.0f;
                    float phaseDifference = 0.0f;
                    if (polarInterpolation) {
                        previousAmplitude = std::sqrt(magnitudeSquared(previousChannel));
                        channelAmplitude = std::sqrt(magnitudeSquared(channel));
                        previousPhase = std::atan2(previousChannel.im, previousChannel.re);
                        const float channelPhase = std::atan2(channel.im, channel.re);
                        phaseDifference = channelPhase - previousPhase;
                        while (phaseDifference > static_cast<float>(PI)) phaseDifference -= static_cast<float>(2.0 * PI);
                        while (phaseDifference < static_cast<float>(-PI)) phaseDifference += static_cast<float>(2.0 * PI);
                    }
                    for (int j = 0; j < static_cast<int>(pending.size()) && dataIndex < dvbt2Params.c_p2; ++j) {
                        const float t = static_cast<float>(j + 1) / span;
                        dsp::complex_t estimate;
                        if (polarInterpolation) {
                            const float amplitude = previousAmplitude + (channelAmplitude - previousAmplitude) * t;
                            const float phase = previousPhase + phaseDifference * t;
                            estimate = {amplitude * std::cos(phase), amplitude * std::sin(phase)};
                        }
                        else {
                            estimate = {
                                previousChannel.re + (channel.re - previousChannel.re) * t,
                                previousChannel.im + (channel.im - previousChannel.im) * t
                            };
                        }
                        const float power = (std::max)(magnitudeSquared(estimate), 1.0e-20f);
                        const auto& value = pending[j];
                        const dsp::complex_t equalized{
                            (value.re * estimate.re + value.im * estimate.im) / power,
                            (value.im * estimate.re - value.re * estimate.im) / power
                        };
                        deinterleaved[addresses[dataIndex++]] = equalized;
                    }
                }
                pending.clear();
                previousChannel = channel;
                havePreviousPilot = true;
            }
            if (dataIndex < 200) return false;
            float decisionQuality = 0.0f;
            for (int i = 0; i < 200; ++i) {
                bits[i] = deinterleaved[i].re > 0.0f ? 0 : 1;
                const float absRe = std::abs(deinterleaved[i].re);
                const float absIm = std::abs(deinterleaved[i].im);
                decisionQuality += absRe / (absRe + absIm + 1.0e-12f);
            }
            decisionQuality /= 200.0f;
            if (decisionQuality > bestDecisionQuality) {
                bestDecisionQuality = decisionQuality;
                bestCandidateShift = binShift;
                bestCandidateMirrored = mirrored;
                bestCandidateEven = evenPhase;
                for (int i = 0; i < 200; ++i) {
                    const float power = std::sqrt(magnitudeSquared(deinterleaved[i])) + 1.0e-12f;
                    bestSoft[i] = deinterleaved[i].re / power;
                }
            }
            uint32_t crc = 0xFFFFFFFFu;
            for (int i = 0; i < 168; ++i) {
                const uint32_t feedback = bits[i] ^ ((crc >> 31) & 1u);
                crc <<= 1;
                if (feedback) crc ^= 0x04C11DB7u;
            }
            return crc == readBits(bits, 168, 32);
        };

        bool crcValid = false;
        int acceptedShift = 0;
        bool acceptedMirrored = false;
        bool acceptedEven = false;
        for (int magnitude = 0; magnitude <= 64 && !crcValid; ++magnitude) {
            const int shifts[2] = {magnitude, -magnitude};
            const int count = magnitude == 0 ? 1 : 2;
            for (int s = 0; s < count && !crcValid; ++s) {
                const int shift = shifts[s];
                for (int orientation = 0; orientation < 2 && !crcValid; ++orientation) {
                    const bool mirrored = orientation != 0;
                    for (int phase = 0; phase < 2 && !crcValid; ++phase) {
                        const bool evenPhase = phase != 0;
                        if (tryAlignment(shift, mirrored, evenPhase, false)) {
                            crcValid = true;
                            acceptedShift = shift;
                            acceptedMirrored = mirrored;
                            acceptedEven = evenPhase;
                        }
                    }
                }
            }
        }
        l1DecisionQuality.store(bestDecisionQuality);
        if (!crcValid) {
            l1CarrierShift.store(bestCandidateShift);
            l1SpectrumMirrored.store(bestCandidateMirrored);
        }
        if (!crcValid && bestDecisionQuality > 0.55f) {
            if (l1SoftCount > 0 &&
                (l1SoftMirrored != bestCandidateMirrored || l1SoftEven != bestCandidateEven)) {
                l1SoftAccumulator.fill(0.0f);
                l1SoftCount = 0;
            }
            if (l1SoftCount >= 16) {
                l1SoftAccumulator.fill(0.0f);
                l1SoftCount = 0;
            }
            l1SoftShift = bestCandidateShift;
            l1SoftMirrored = bestCandidateMirrored;
            l1SoftEven = bestCandidateEven;
            for (int i = 0; i < 200; ++i) l1SoftAccumulator[i] += bestSoft[i];
            ++l1SoftCount;
            if (l1SoftCount >= 2) {
                for (int i = 0; i < 200; ++i) bits[i] = l1SoftAccumulator[i] > 0.0f ? 0 : 1;
                uint32_t combinedCrc = 0xFFFFFFFFu;
                for (int i = 0; i < 168; ++i) {
                    const uint32_t feedback = bits[i] ^ ((combinedCrc >> 31) & 1u);
                    combinedCrc <<= 1;
                    if (feedback) combinedCrc ^= 0x04C11DB7u;
                }
                crcValid = combinedCrc == readBits(bits, 168, 32);
                if (crcValid) {
                    acceptedShift = bestCandidateShift;
                    acceptedMirrored = bestCandidateMirrored;
                    acceptedEven = bestCandidateEven;
                }
            }
        }
        if (!crcValid) {
            l1LiveValid.store(false);
            return;
        }

        l1CrcValidFrames.fetch_add(1);
        l1SoftAccumulator.fill(0.0f);
        l1SoftCount = 0;
        l1AlignmentLocked = true;
        l1LockedShift = acceptedShift;
        l1LockedFrequency = frequency;
        l1LockedMirrored = acceptedMirrored;
        l1LockedEven = acceptedEven;
        l1CarrierShift.store(acceptedShift);
        l1SpectrumMirrored.store(acceptedMirrored);

        // A soft-combined L1-pre decision may have succeeded after the search
        // buffer was last populated by a different, failed alignment.  Always
        // rebuild the complete P2 cell vector with the accepted parameters so
        // its payload and the decoded signalling belong to the same frame.
        if (!tryAlignment(acceptedShift, acceptedMirrored, acceptedEven, false)) {
            l1LiveValid.store(false);
            currentP2PayloadReady = false;
            return;
        }
        l1PilotPattern.store(static_cast<int>(readBits(bits, 68, 4)));
        l1PostMod.store(static_cast<int>(readBits(bits, 24, 4)));
        l1PostSize.store(static_cast<int>(readBits(bits, 32, 18)));
        l1PostInfoSize.store(static_cast<int>(readBits(bits, 50, 18)));
        l1BandwidthExtended.store(static_cast<int>(readBits(bits, 8, 1)));
        l1GuardMode.store(static_cast<int>(readBits(bits, 17, 3)));
        l1PaprMode.store(static_cast<int>(readBits(bits, 20, 4)));
        l1S2Field2.store(static_cast<int>(readBits(bits, 15, 1)));
        l1NumRf.store(static_cast<int>(readBits(bits, 152, 3)));
        const int t2Version = static_cast<int>(readBits(bits, 158, 4));
        l1PostScrambled.store(t2Version > 1 && readBits(bits, 162, 1) != 0);
        l1CellId.store(static_cast<int>(readBits(bits, 80, 16)));
        l1NetworkId.store(static_cast<int>(readBits(bits, 96, 16)));
        l1SystemId.store(static_cast<int>(readBits(bits, 112, 16)));
        l1Frames.store(static_cast<int>(readBits(bits, 128, 8)));
        l1DataSymbols.store(static_cast<int>(readBits(bits, 136, 12)));
        l1LiveValid.store(true);
        l1PreValid.store(true);
        currentP2PayloadReady = false;
        const uint64_t validPostFramesBefore = l1PostValidFrames.load();
        decodeL1Post(deinterleaved);
        const bool currentL1PostValid = l1PostValidFrames.load() > validPostFramesBefore;
        const int payloadStart = 1840 + l1PostSize.load();
        if (currentL1PostValid && payloadStart >= 0 && payloadStart < static_cast<int>(deinterleaved.size())) {
            currentP2Payload.assign(deinterleaved.begin() + payloadStart, deinterleaved.end());
            normalizeQamCells(currentP2Payload);
            currentP2PayloadReady = true;
        }
        prepareDataPilots();
        analyzeFirstDataSymbol();
    }

    void initP1Randomizer() {
        int shiftRegister = 0x4e46;
        for (int i = 0; i < dvbt2_p1_tables::ACTIVE_CARRIER_COUNT; ++i) {
            const int bit = (shiftRegister ^ (shiftRegister >> 1)) & 1;
            p1Randomizer[i] = bit == 0 ? 1 : -1;
            shiftRegister >>= 1;
            if (bit) shiftRegister |= 0x4000;
        }
    }

    bool decodeP1Window(double start) {
        constexpr double inputSamplesPerDvbSample = INPUT_SAMPLE_RATE / DVB_SAMPLE_RATE;
        for (int i = 0; i < 1024; ++i) {
            const double position = start + static_cast<double>(i) * inputSamplesPerDvbSample;
            const int index = static_cast<int>(std::floor(position));
            const float fraction = static_cast<float>(position - std::floor(position));
            const auto& a = captureRing[index % captureRing.size()];
            const auto& b = captureRing[(index + 1) % captureRing.size()];
            fftInput[i] = {a.re + (b.re - a.re) * fraction, a.im + (b.im - a.im) * fraction};
        }
        fftwf_execute(fftPlan);

        for (int carrierShift = 76; carrierShift <= 96; ++carrierShift) {
            std::array<int, dvbt2_p1_tables::ACTIVE_CARRIER_COUNT> signs{};
            int oldSign = -1;
            signs[0] = oldSign * p1Randomizer[0];
            for (int i = 1; i < dvbt2_p1_tables::ACTIVE_CARRIER_COUNT; ++i) {
                const int currentBin = (carrierShift + dvbt2_p1_tables::activeCarriers[i] + 512) & 1023;
                const int previousBin = (carrierShift + dvbt2_p1_tables::activeCarriers[i - 1] + 512) & 1023;
                const auto& current = fftOutput[currentBin];
                const auto& previous = fftOutput[previousBin];
                const dsp::complex_t difference = multiplyConjugate(current, previous);
                const int sign = std::abs(std::atan2(difference.im, difference.re)) > PI * 0.5 ? -oldSign : oldSign;
                oldSign = sign;
                signs[i] = sign * p1Randomizer[i];
            }

            std::array<uint8_t, 48> data{};
            oldSign = 1;
            for (int i = 0; i < dvbt2_p1_tables::ACTIVE_CARRIER_COUNT; ++i) {
                const uint8_t bit = signs[i] == oldSign ? 0 : 1;
                oldSign = signs[i];
                data[i / 8] = static_cast<uint8_t>((data[i / 8] << 1) | bit);
            }

            bool repeated = true;
            for (int i = 0; i < 8; ++i) repeated = repeated && data[i] == data[i + 40];
            if (!repeated) continue;

            int s1 = -1;
            for (int row = 0; row < 8; ++row) {
                bool match = true;
                for (int column = 0; column < 8; ++column) match = match && data[column] == dvbt2_p1_tables::s1Patterns[row][column];
                if (match) { s1 = row; break; }
            }
            int s2 = -1;
            for (int row = 0; row < 16; ++row) {
                bool match = true;
                for (int column = 0; column < 32; ++column) match = match && data[8 + column] == dvbt2_p1_tables::s2Patterns[row][column];
                if (match) { s2 = row; break; }
            }
            if (s1 < 0 || s1 > 4 || s2 < 0) continue;

            decodedPreamble.store(s1);
            decodedFft.store(s2 >> 1);
            decodedMixed.store((s2 & 1) != 0);
            coarseOffset.store(coarseOffset.load() + static_cast<float>((carrierShift - 86) * (DVB_SAMPLE_RATE / 1024.0)));
            return true;
        }
        return false;
    }

    bool decodeP1() {
        // The correlation pulse ends near the end of P1. Search the possible A
        // section alignment in the retained samples and accept only exact S1/S2.
        const int oldest = static_cast<int>(capturePos);
        for (int relative = 6300; relative <= 6850; relative += 4) {
            if (decodeP1Window((oldest + relative) % captureRing.size())) return true;
        }
        return false;
    }

    static void iqHandler(dsp::complex_t* samples, int count, void* ctx) {
        auto* self = static_cast<Dvbt2DecoderModule*>(ctx);
        for (int i = 0; i < count; ++i) self->processSample(samples[i]);
    }

    std::string name;
    bool enabled = true;
    int displayMode = 0;
    VFOManager::VFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;

    std::array<dsp::complex_t, P1_B> delayed{};
    std::array<dsp::complex_t, 8192> captureRing{};
    std::array<dsp::complex_t, P1_B> products{};
    std::array<float, P1_B> currentEnergy{};
    std::array<float, P1_B> delayedEnergy{};
    int delayPos = 0;
    int productPos = 0;
    size_t capturePos = 0;
    dsp::complex_t sumProduct{0.0f, 0.0f};
    dsp::complex_t peakArgument{0.0f, 0.0f};
    double sumCurrentEnergy = 0.0;
    double sumDelayedEnergy = 0.0;
    float oscillatorRe = 1.0f;
    float oscillatorIm = 0.0f;
    uint32_t oscillatorSamples = 0;
    uint64_t samplesSeen = 0;
    uint64_t lastP1Sample = 0;
    uint64_t lastCandidateSample = 0;
    uint64_t nextPredictedFrameSample = 0;
    int candidateSamples = 0;
    int repeatConfirmations = 0;
    double expectedFrameSamples = 0.0;
    float peakMetric = 0.0f;
    float displayedMetric = 0.0f;

    std::atomic<float> threshold{0.40f};
    std::atomic<float> correlation{0.0f};
    std::atomic<float> levelDb{-120.0f};
    std::atomic<float> coarseOffset{0.0f};
    std::atomic<bool> p1Locked{false};
    std::atomic<uint64_t> p1Count{0};
    std::atomic<uint64_t> p1PredictedCaptures{0};
    std::atomic<bool> resetRequested{false};
    std::array<int, dvbt2_p1_tables::ACTIVE_CARRIER_COUNT> p1Randomizer{};
    dsp::complex_t* fftInput = nullptr;
    dsp::complex_t* fftOutput = nullptr;
    fftwf_plan fftPlan = nullptr;
    dsp::complex_t* p2FftInput = nullptr;
    dsp::complex_t* p2FftOutput = nullptr;
    fftwf_plan p2FftPlan = nullptr;
    dvbt2_parameters dvbt2Params{};
    dvbt2_parameters dataParams{};
    pilot_generator p2Pilots;
    address_freq_deinterleaver p2Addresses;
    std::atomic<int> decodedPreamble{-1};
    std::atomic<int> decodedFft{-1};
    std::atomic<bool> decodedMixed{false};
    std::array<dsp::complex_t, P2_CAPTURE_SAMPLES> p2Capture{};
    bool p2Collecting = false;
    int p2Fill = 0;
    std::atomic<bool> p2Synchronized{false};
    std::atomic<float> p2Correlation{0.0f};
    std::atomic<int> guardInterval{-1};
    std::atomic<uint64_t> p2Count{0};
    std::atomic<bool> l1PreValid{false};
    std::atomic<bool> l1LiveValid{false};
    std::atomic<uint64_t> l1CrcAttempts{0};
    std::atomic<uint64_t> l1CrcValidFrames{0};
    std::array<float, 200> l1SoftAccumulator{};
    int l1SoftCount = 0;
    int l1SoftShift = 0;
    bool l1SoftMirrored = false;
    bool l1SoftEven = false;
    bool l1AlignmentLocked = false;
    int l1LockedShift = 0;
    double l1LockedFrequency = 0.0;
    bool l1LockedMirrored = false;
    bool l1LockedEven = false;
    std::atomic<int> l1PilotPattern{-1};
    std::atomic<int> l1CellId{-1};
    std::atomic<int> l1NetworkId{-1};
    std::atomic<int> l1SystemId{-1};
    std::atomic<int> l1Frames{-1};
    std::atomic<int> l1DataSymbols{-1};
    std::atomic<int> l1CarrierShift{0};
    std::atomic<float> l1DecisionQuality{0.0f};
    std::atomic<bool> l1SpectrumMirrored{false};
    std::atomic<int> l1PostMod{-1};
    std::atomic<int> l1PostSize{-1};
    std::atomic<int> l1PostInfoSize{-1};
    std::atomic<int> l1BandwidthExtended{0};
    std::atomic<int> l1GuardMode{0};
    std::atomic<int> l1PaprMode{0};
    std::atomic<int> l1S2Field2{0};
    std::atomic<int> l1NumRf{1};
    std::atomic<bool> l1PostScrambled{false};
    std::atomic<bool> l1PostValid{false};
    std::atomic<uint64_t> l1PostAttempts{0};
    std::atomic<uint64_t> l1PostValidFrames{0};
    std::atomic<int> l1NumPlps{-1};
    std::atomic<int> l1SubSlicesPerFrame{-1};
    std::array<std::atomic<int>, 8> plpIds{};
    std::array<std::atomic<int>, 8> plpTypes{};
    std::array<std::atomic<int>, 8> plpMods{};
    std::array<std::atomic<int>, 8> plpCodes{};
    std::array<std::atomic<int>, 8> plpFecTypes{};
    std::array<std::atomic<int>, 8> plpRotations{};
    std::atomic<int> plpNumBlocksMax{-1};
    std::atomic<int> plpFrameInterval{-1};
    std::atomic<int> plpTimeIlLength{-1};
    std::atomic<int> plpTimeIlType{-1};
    std::atomic<bool> l1DynamicValid{false};
    std::atomic<int> l1DynamicFrame{-1};
    std::atomic<int> dynamicPlpStart{-1};
    std::atomic<int> dynamicPlpBlocks{-1};
    bool dataPilotsReady = false;
    int currentFirstDataUsefulStart = -1;
    double currentFrameFrequency = 0.0;
    std::atomic<bool> dataSymbolSynchronized{false};
    std::atomic<float> dataPilotCorrelation{0.0f};
    std::atomic<uint64_t> dataSymbolCount{0};
    std::vector<dsp::complex_t> fullFrameCapture;
    bool fullFrameCollecting = false;
    int fullFrameFill = 0;
    int fullFrameTarget = 0;
    int fullFrameP2Boundary = -1;
    int fullFrameGuardSamples = 0;
    bool fullFrameSnapshotReady = false;
    int fullFrameSnapshotSymbols = 0;
    int fullFrameSnapshotCarrierShift = 0;
    int fullFrameSnapshotDynamicFrame = -1;
    int fullFrameSnapshotPlpStart = 0;
    int fullFrameSnapshotPlpBlocks = 0;
    int fullFrameSnapshotPlpModulation = 0;
    int fullFrameSnapshotPlpCode = 0;
    int fullFrameSnapshotPlpFecType = 0;
    int fullFrameSnapshotPlpRotation = 0;
    int fullFrameSnapshotMaxBlocks = 0;
    int fullFrameSnapshotTimeLength = 0;
    int fullFrameSnapshotTimeType = 0;
    double fullFrameSnapshotFrequency = 0.0;
    std::vector<dsp::complex_t> fullFrameP2Payload;
    std::atomic<uint64_t> fullFrameCount{0};
    std::atomic<int> frameSymbolsVerified{0};
    std::atomic<float> frameMeanCpCorrelation{0.0f};
    std::atomic<int> frameTimingCorrection{0};
    std::vector<dsp::complex_t> frameWorkBuffer;
    struct QueuedFrame {
        std::vector<dsp::complex_t> samples;
        std::vector<dsp::complex_t> p2Payload;
        int sampleCount = 0;
        int boundary = 0;
        int guard = 0;
        int symbolCount = 0;
        int carrierShift = 0;
        int dynamicFrame = -1;
        int plpStart = 0;
        int plpBlocks = 0;
        int plpModulation = 0;
        int plpCode = 0;
        int plpFecType = 0;
        int plpRotation = 0;
        int plpMaxBlocks = 0;
        int timeIlLength = 0;
        int timeIlType = 0;
        double frequency = 0.0;
        uint64_t generation = 0;
    };
    std::array<QueuedFrame, FRAME_QUEUE_CAPACITY> queuedFrames;
    int queuedFrameHead = 0;
    int queuedFrameCount = 0;
    dsp::complex_t* workerFftInput = nullptr;
    dsp::complex_t* workerFftOutput = nullptr;
    fftwf_plan workerFftPlan = nullptr;
    std::thread frameWorker;
    std::mutex frameWorkerMutex;
    std::condition_variable frameWorkerCv;
    std::vector<std::thread> computeWorkers;
    unsigned int logicalProcessorCount = 1;
    int computeWorkerLimit = 4;
    std::mutex computeMutex;
    std::condition_variable computeCv;
    std::condition_variable computeDoneCv;
    std::function<void(int)> computeTask;
    uint64_t computeGeneration = 0;
    int computeActiveWorkers = 0;
    int computeWorkersRemaining = 0;
    bool computeStop = false;
    bool frameWorkerStop = false;
    bool frameWorkerBusy = false;
    bool frameWorkPending = false;
    int workerFrameSamples = 0;
    uint64_t workerFrameGeneration = 0;
    int workerBoundary = 0;
    int workerGuard = 0;
    int workerSymbolCount = 0;
    int workerCarrierShift = 0;
    double workerFrequency = 0.0;
    std::atomic<uint64_t> workerFramesProcessed{0};
    std::atomic<uint64_t> workerFramesSkipped{0};
    std::atomic<uint64_t> workerFramesQueued{0};
    std::atomic<int> workerQueueDepth{0};
    std::atomic<int> workerQueuePeak{0};
    std::atomic<float> workerFrameMilliseconds{0.0f};
    std::atomic<float> workerLdpcMilliseconds{0.0f};
    std::atomic<float> workerOfdmMilliseconds{0.0f};
    std::atomic<float> workerDemapMilliseconds{0.0f};
    std::atomic<float> workerResampleMilliseconds{0.0f};
    std::atomic<float> workerFftMilliseconds{0.0f};
    std::atomic<float> workerEqualizeMilliseconds{0.0f};
    float pendingDemapMilliseconds = 0.0f;
    float pendingLdpcMilliseconds = 0.0f;
    std::atomic<float> workerFrameAverageMilliseconds{0.0f};
    std::atomic<float> workerFrameMaximumMilliseconds{0.0f};
    std::atomic<int> workerDynamicFrame{-1};
    std::atomic<int> workerSymbolsEqualized{0};
    std::atomic<int> workerExpectedDataSymbols{0};
    std::atomic<float> workerMeanPilotCorrelation{0.0f};
    std::atomic<uint64_t> workerDataCells{0};
    std::mutex qamConstellationMutex;
    std::array<std::array<dsp::complex_t, QAM_CONSTELLATION_POINTS>, QAM_CONSTELLATION_HISTORY> qamConstellationHistory{};
    std::array<size_t, QAM_CONSTELLATION_HISTORY> qamConstellationCounts{};
    size_t qamConstellationHead = 0;
    std::atomic<uint64_t> frameP2SectionCells{0};
    std::atomic<uint64_t> frameDataSectionCells{0};
    std::atomic<uint64_t> frameClosingSectionCells{0};
    std::atomic<bool> frameClosingAcquired{false};
    std::atomic<float> frameClosingPilotCorrelation{0.0f};
    std::atomic<int> frameClosingCells{0};
    std::vector<dsp::complex_t> currentP2Payload;
    bool currentP2PayloadReady = false;
    std::vector<dsp::complex_t> workerP2Payload;
    std::vector<dsp::complex_t> previousFrameP2Payload;
    std::vector<dsp::complex_t> workerExtractedPlp;
    std::vector<dsp::complex_t> workerTimeDeinterleaved;
    std::vector<int8_t> workerSoftBits;
    std::vector<int8_t> workerFecSoftBits;
    std::vector<int> workerCellPermutation;
    int workerPermutationMaxBlocks = -1;
    int workerPermutationCellsPerBlock = -1;
    int workerPlpStart = 0;
    int workerPlpBlocks = 0;
    int workerPlpModulation = 0;
    int workerPlpCode = 0;
    int workerPlpFecType = 0;
    int workerPlpRotation = 0;
    int workerPlpMaxBlocks = 0;
    int workerTimeIlLength = 0;
    int workerTimeIlType = 0;
    int workerFrequencyMapping = 0;
    std::atomic<uint64_t> decodeGeneration{0};
    std::atomic<uint64_t> activeFrameGeneration{0};
    std::atomic<uint64_t> plpFramesAssembled{0};
    std::atomic<uint64_t> plpCellsAvailable{0};
    std::atomic<uint64_t> plpCellsRequired{0};
    std::atomic<uint64_t> plpCellsExtracted{0};
    std::atomic<uint64_t> tiBlocksCompleted{0};
    std::atomic<uint64_t> fecBlocksReordered{0};
    std::atomic<uint64_t> softQamValues{0};
    std::atomic<float> qamSnrDb{-120.0f};
    std::atomic<uint64_t> bitDeinterleavedBlocks{0};
    std::atomic<uint64_t> ldpcBlocksTested{0};
    std::atomic<uint64_t> ldpcBlocksConverged{0};
    std::atomic<uint64_t> ldpcAlternateAttempts{0};
    std::atomic<uint64_t> ldpcAlternateRecovered{0};
    std::atomic<uint64_t> ldpcHalfScaleAttempts{0};
    std::atomic<uint64_t> ldpcHalfScaleRecovered{0};
    std::atomic<uint64_t> ldpcQuarterScaleAttempts{0};
    std::atomic<uint64_t> ldpcQuarterScaleRecovered{0};
    std::atomic<uint64_t> bchRecoveryAttempts{0};
    std::atomic<uint64_t> bchRecoveredBlocks{0};
    std::atomic<uint64_t> bchCorrectedBits{0};
    std::atomic<int> residualFecCapturesAttempted{0};
    std::atomic<int> residualFecCapturesSaved{0};
    std::atomic<int> ldpcFirstBlockTested{0};
    std::atomic<float> ldpcMeanRawUnsatisfied{0.0f};
    std::atomic<float> ldpcMeanUnsatisfied{0.0f};
    std::atomic<uint64_t> bbHeadersTested{0};
    std::atomic<uint64_t> bbHeadersValid{0};
    std::atomic<int> bbInputMode{-1};
    std::atomic<int> bbTsGs{-1};
    std::atomic<int> bbUpl{-1};
    std::atomic<int> bbDfl{-1};
    std::atomic<int> bbSync{-1};
    std::atomic<int> bbSyncd{-1};
    std::atomic<bool> bbNpd{false};
    std::ofstream tsOutput;
    std::ofstream selectedTsOutput;
    std::mutex tsOutputMutex;
    std::mutex tsServiceMutex;
    std::map<int, TsServiceInfo> tsServices;
    std::set<int> tsPmtPids;
    std::map<int, PsiAssemblyState> tsPsiStates;
    std::atomic<int> selectedTsService{-1};
    std::atomic<int> selectedPatVersion{0};
    int selectedTsOutputService = -1;
    std::atomic<uint64_t> selectedTsPacketsWritten{0};
    std::atomic<bool> selectedRecordingEnabled{false};
    std::atomic<bool> selectedRecordingRestartRequested{false};
    std::atomic<uint64_t> selectedRecordedPackets{0};
    std::atomic<bool> fullRecordingEnabled{false};
    std::atomic<bool> fullRecordingRestartRequested{false};
    std::atomic<uint64_t> fullRecordedPackets{0};
    std::mutex recordingConfigMutex;
    char selectedRecordingPath[512] = "recordings/dvbt2_selected.ts";
    char fullRecordingPath[512] = "recordings/dvbt2_output.ts";
    std::atomic<bool> appendRecordingTimestamp{false};
    std::mutex udpConfigMutex;
    char udpHost[64] = "127.0.0.1";
    int udpPort = 1234;
    char playerCommand[1024] =
        "vlc.exe --video-on-top --network-caching=750\n--live-caching=750 udp://@:$port";
    std::atomic<bool> udpStreamingEnabled{false};
    std::atomic<bool> udpRestartRequested{false};
    std::atomic<bool> udpPacketResetRequested{false};
    std::atomic<bool> selectedDiscontinuityPending{false};
    Dvbt2UdpSocket udpSocket = INVALID_DVBT2_UDP_SOCKET;
    sockaddr_storage udpDestination{};
    int udpDestinationLength = 0;
    std::array<uint8_t, 188 * 7> udpPacketBuffer{};
    int udpPacketFill = 0;
    std::atomic<uint64_t> udpDatagramsSent{0};
    std::atomic<uint64_t> udpErrors{0};
    std::atomic<uint64_t> udpDatagramsDropped{0};
    std::atomic<int> udpQueueDepth{0};
    std::mutex udpQueueMutex;
    std::condition_variable udpQueueCv;
    std::deque<std::array<uint8_t, 188 * 7>> udpDatagramQueue;
    std::thread udpSenderThread;
    bool udpSenderStop = false;
    std::atomic<int> playerLaunchStatus{0};
#ifdef _WIN32
    bool udpWinsockReady = false;
#endif
    std::atomic<int> tsTransportStreamId{1};
    std::atomic<int> tsPatVersion{0};
    bool tsSynchronized = false;
    bool tsAwaitingDnp = false;
    int tsPacketPosition = 0;
    std::array<uint8_t, 188> tsPacketBuffer{};
    std::array<uint8_t, 188> tsPendingUsefulPacket{};
    std::atomic<uint64_t> tsPacketsWritten{0};
    std::atomic<uint64_t> tsBytesWritten{0};
    std::atomic<uint64_t> tsResyncs{0};
    std::atomic<uint64_t> tsNullPacketsReinserted{0};
    std::atomic<uint64_t> tsResyncFirstBlock{0};
    std::atomic<uint64_t> tsResyncTiBoundary{0};
    std::atomic<uint64_t> tsResyncOtherBlock{0};
    std::atomic<int> tsLastExpectedSyncBytes{-1};
    std::atomic<int> tsLastActualSyncBytes{-1};
    std::atomic<int> tsLastResyncBlock{-1};
    std::array<std::atomic<uint64_t>, TS_RESYNC_BLOCK_BUCKETS> tsResyncByBlock{};
    std::atomic<uint64_t> tsResyncForwardOne{0};
    std::atomic<uint64_t> tsResyncBackwardOne{0};
    std::atomic<uint64_t> tsResyncIrregular{0};
    std::atomic<bool> ldpcSelfTestRun{false};
    std::atomic<int> ldpcSelfTestStatus{-1};
    std::atomic<bool> debugFrameWritten{false};
    std::atomic<bool> debugFrameSaved{false};
    std::atomic<bool> debugSymbolWritten{false};
    std::atomic<bool> debugSymbolSaved{false};
    std::atomic<bool> debugAllSymbolsWritten{false};
    std::atomic<bool> debugAllSymbolsSaved{false};
    std::atomic<bool> debugTimeDeinterleaverWritten{false};
    std::atomic<bool> debugTimeDeinterleaverSaved{false};
    std::atomic<bool> debugFecInputWritten{false};
    std::atomic<bool> debugFecInputSaved{false};
    std::atomic<bool> debugRawFrameWritten{false};
    std::atomic<bool> debugRawFrameSaved{false};
};

MOD_EXPORT void _INIT_() {
    config.setPath(core::args["root"].s() + "/dvbt2_decoder_config.json");
    config.load(json({}));
    config.enableAutoSave();
}
MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new Dvbt2DecoderModule(std::move(name));
}
MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<Dvbt2DecoderModule*>(instance);
}
MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
