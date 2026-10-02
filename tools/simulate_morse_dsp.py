import sys
import wave

import numpy as np


path = sys.argv[1]
vfo_offset = float(sys.argv[2])
with wave.open(path, "rb") as wav:
    sample_rate = wav.getframerate()
    values = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2")

iq = values[0::2].astype(np.float64) + 1j * values[1::2].astype(np.float64)
n = np.arange(iq.size)
shifted = iq * np.exp(-2j * np.pi * vfo_offset * n / sample_rate)

# Idealized equivalent of the Radio VFO's 1 kHz channel filter. Zero-padding
# prevents the FFT filter's end from wrapping into the beginning of the capture.
padded_size = 1 << int(np.ceil(np.log2(shifted.size + sample_rate)))
spectrum = np.fft.fft(shifted, padded_size)
frequencies = np.fft.fftfreq(padded_size, 1.0 / sample_rate)
spectrum[np.abs(frequencies) > 500.0] = 0
channel = np.fft.ifft(spectrum)[: shifted.size]

decimation = sample_rate // 8000
channel = channel[::decimation]
block_size = 80
channel = channel[: (channel.size // block_size) * block_size].reshape(-1, block_size)
sample_indices = np.arange(block_size)
oscillators = np.exp(
    -2j * np.pi * np.arange(-18, 19)[:, None] * 25.0 * sample_indices[None, :] / 8000.0
)

total_power = np.maximum(np.mean(np.abs(channel) ** 2, axis=1), 1e-15)
coherent = np.abs(channel @ oscillators.T) ** 2 / (block_size * block_size)
scores = np.clip(np.max(coherent, axis=1) / total_power, 0.0, 1.0)
carrier_amplitude = np.sqrt(np.max(coherent, axis=1))

# Adaptive envelope detector proposed for the live module. The noise estimate
# falls quickly but rises very slowly; the signal peak attacks immediately and
# decays slowly. This preserves absolute keying contrast instead of mistaking
# the strongest narrowband-noise bin for a carrier.
envelope_scores = np.zeros_like(carrier_amplitude)
noise_floor = float(carrier_amplitude[0])
signal_peak = noise_floor
envelope_ready_blocks = 0
for i, amplitude in enumerate(carrier_amplitude):
    noise_alpha = 0.20 if amplitude < noise_floor else 0.002
    noise_floor += (amplitude - noise_floor) * noise_alpha
    if amplitude > signal_peak:
        signal_peak = float(amplitude)
    else:
        signal_peak += (amplitude - signal_peak) * 0.001
    dynamic_range = signal_peak - noise_floor
    if signal_peak > noise_floor * 2.0 and dynamic_range > 1e-12:
        envelope_ready_blocks += 1
    else:
        envelope_ready_blocks = 0
    if envelope_ready_blocks >= 3:
        envelope_scores[i] = np.clip((amplitude - noise_floor) / dynamic_range, 0.0, 1.0)

print("score_percentiles=" + "/".join(f"{x:.3f}" for x in np.percentile(scores, [1, 10, 20, 50, 75, 90, 99])))
for threshold in (0.20, 0.30, 0.40, 0.50, 0.60, np.sqrt(0.40), 0.70):
    down = scores >= threshold
    changes = np.flatnonzero(np.diff(down.astype(np.int8)) != 0) + 1
    edges = np.concatenate(([0], changes, [down.size]))
    durations = np.diff(edges) * 10.0
    states = down[edges[:-1]]
    marks = durations[states]
    gaps = durations[~states]
    mark_mid = np.percentile(marks, [25, 50, 75]) if marks.size else [0, 0, 0]
    gap_mid = np.percentile(gaps, [25, 50, 75]) if gaps.size else [0, 0, 0]
    print(
        f"threshold={threshold:.2f} transitions={max(0, edges.size-2)} "
        f"marks={marks.size} mark_p25/p50/p75={'/'.join(f'{x:.0f}' for x in mark_mid)}ms "
        f"gaps={gaps.size} gap_p25/p50/p75={'/'.join(f'{x:.0f}' for x in gap_mid)}ms"
    )

morse_table = {
    ".-":"A", "-...":"B", "-.-.":"C", "-..":"D", ".":"E", "..-.":"F",
    "--.":"G", "....":"H", "..":"I", ".---":"J", "-.-":"K", ".-..":"L",
    "--":"M", "-.":"N", "---":"O", ".--.":"P", "--.-":"Q", ".-.":"R",
    "...":"S", "-":"T", "..-":"U", "...-":"V", ".--":"W", "-..-":"X",
    "-.--":"Y", "--..":"Z", "-----":"0", ".----":"1", "..---":"2",
    "...--":"3", "....-":"4", ".....":"5", "-....":"6", "--...":"7",
    "---..":"8", "----.":"9", ".-.-.-":".", "..--..":"?"
}

def decode_scores(threshold, character_factor, detector_scores=scores):
    text = ""
    symbol = ""
    dot = 60.0
    history = []
    key = previous = raw_state = False
    raw_blocks = state_blocks = 0
    character_committed = word_committed = False
    for score in detector_scores:
        on_threshold = threshold
        off_threshold = threshold * 0.70
        raw = score >= (off_threshold if key else on_threshold)
        if raw == raw_state:
            raw_blocks += 1
        else:
            raw_state = raw
            raw_blocks = 1
        down = raw_state if raw_blocks >= 3 else key
        if down == previous:
            state_blocks += 1
        else:
            duration = state_blocks * 10.0
            if previous and state_blocks:
                if duration < 30.0:
                    state_blocks = 1
                    previous = down
                    key = down
                    continue
                history.append(duration)
                history = history[-24:]
                if len(history) >= 3:
                    best_dot, best_cost = dot, float("inf")
                    for candidate in np.arange(25.0, 241.0, 1.0):
                        errors = [min(abs(mark-candidate)/candidate,
                                      abs(mark-3*candidate)/(3*candidate), 1.0)
                                  for mark in history]
                        cost = sum(errors) / len(errors)
                        if cost < best_cost:
                            best_dot, best_cost = candidate, cost
                    dot = min(240.0, max(25.0, dot * 0.65 + best_dot * 0.35))
                symbol += "-" if duration > dot * 2.0 else "."
                character_committed = False
                word_committed = False
            state_blocks = 1
            previous = down
        if not down:
            gap = state_blocks * 10.0
            if not character_committed and gap >= dot * character_factor:
                if symbol:
                    text += morse_table.get(symbol, "?")
                    symbol = ""
                character_committed = True
            if not word_committed and gap >= dot * 6.0:
                if text and not text.endswith(" "):
                    text += " "
                word_committed = True
        key = down
    if symbol:
        text += morse_table.get(symbol, "?")
    return text, dot

for threshold in (0.60, 0.63, 0.65, 0.70):
    for character_factor in (2.5, 2.2):
        text, dot = decode_scores(threshold, character_factor)
        print(f"decode threshold={threshold:.2f} char_factor={character_factor:.1f} dot={dot:.1f} text={text}")

print("envelope_score_percentiles=" + "/".join(f"{x:.3f}" for x in np.percentile(envelope_scores, [1, 10, 20, 50, 75, 90, 99])))
for threshold in (0.40, 0.50, 0.65):
    text, dot = decode_scores(threshold, 2.2, envelope_scores)
    print(f"envelope_decode threshold={threshold:.2f} dot={dot:.1f} text={text}")
