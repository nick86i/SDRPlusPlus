#pragma once

#include <dsp/stream.h>
#include <dsp/types.h>

// ModuleCom contract used by Radio to host optional decoder modules without
// linking their DSP or codec dependencies into radio.dll.
enum ExternalDecoderCommand {
    EXTERNAL_DECODER_GET_INFO = 0,
    EXTERNAL_DECODER_ATTACH,
    EXTERNAL_DECODER_DETACH,
    EXTERNAL_DECODER_DRAW_MENU,
};

struct ExternalDecoderInfo {
    const char* id = nullptr;
    const char* displayName = nullptr;
    double inputSampleRate = 48000.0;
    double audioSampleRate = 48000.0;
    double defaultBandwidth = 10000.0;
    double minBandwidth = 10000.0;
    double maxBandwidth = 10000.0;
    double snapInterval = 1000.0;
    dsp::stream<dsp::stereo_t>* audioOutput = nullptr;
};

struct ExternalDecoderAttachArgs {
    dsp::stream<dsp::complex_t>* iqInput = nullptr;
};
