import sys
import wave

import numpy as np


path = sys.argv[1]
with wave.open(path, "rb") as wav:
    sample_rate = wav.getframerate()
    channels = wav.getnchannels()
    frames = wav.getnframes()
    assert channels == 2 and wav.getsampwidth() == 2

    fft_size = 262144
    window = np.hanning(fft_size)
    spectrum = np.zeros(fft_size, dtype=np.float64)
    spectra = 0
    while spectra < 64:
        raw = wav.readframes(fft_size)
        values = np.frombuffer(raw, dtype="<i2")
        if values.size != fft_size * 2:
            break
        iq = values[0::2].astype(np.float64) + 1j * values[1::2].astype(np.float64)
        transformed = np.fft.fftshift(np.fft.fft(iq * window))
        spectrum += np.abs(transformed) ** 2
        spectra += 1

freq = np.fft.fftshift(np.fft.fftfreq(fft_size, 1.0 / sample_rate))
spectrum /= max(spectra, 1)
spectrum_db = 10.0 * np.log10(np.maximum(spectrum, 1e-30))

search = np.flatnonzero(np.abs(freq) <= 5000.0)
local = spectrum_db[search]
peak_indices = np.flatnonzero((local[1:-1] > local[:-2]) & (local[1:-1] >= local[2:])) + 1
peak_indices = peak_indices[np.argsort(local[peak_indices])[::-1]]
separated = []
minimum_spacing = max(1, int(20 * fft_size / sample_rate))
for candidate in peak_indices:
    if all(abs(int(candidate) - chosen) >= minimum_spacing for chosen in separated):
        separated.append(int(candidate))
    if len(separated) == 20:
        break
peak_indices = np.asarray(separated)
noise_db = float(np.median(local))

print(f"duration_s={frames / sample_rate:.3f} sample_rate={sample_rate} spectra={spectra}")
print(f"median_spectrum_db={noise_db:.2f}")
print("strongest_peaks_within_5khz:")
for idx in peak_indices[:12]:
    absolute = search[idx]
    print(f"  offset_hz={freq[absolute]:+.2f} level_db={spectrum_db[absolute]:.2f} above_median_db={spectrum_db[absolute]-noise_db:.2f}")

global_peaks = np.flatnonzero((spectrum_db[1:-1] > spectrum_db[:-2]) & (spectrum_db[1:-1] >= spectrum_db[2:])) + 1
global_peaks = global_peaks[np.argsort(spectrum_db[global_peaks])[::-1]]
global_separated = []
for candidate in global_peaks:
    if all(abs(int(candidate) - chosen) >= minimum_spacing for chosen in global_separated):
        global_separated.append(int(candidate))
    if len(global_separated) == 16:
        break
print("strongest_peaks_full_band:")
for absolute in global_separated[:16]:
    print(f"  offset_hz={freq[absolute]:+.2f} level_db={spectrum_db[absolute]:.2f}")

# Analyze the strongest narrow carrier in the local baseband. This also shows
# when the decoder's acquisition range does not cover the actual CW tone.
near_center = search[np.abs(freq[search]) <= 1500.0]
carrier_index = near_center[np.argmax(spectrum_db[near_center])]
carrier_hz = float(freq[carrier_index])
if len(sys.argv) > 2:
    requested_hz = float(sys.argv[2])
    around_requested = np.flatnonzero(np.abs(freq - requested_hz) <= 1500.0)
    carrier_index = around_requested[np.argmax(spectrum_db[around_requested])]
    carrier_hz = float(freq[carrier_index])
    print(f"requested_channel_hz={requested_hz:+.2f} carrier_relative_hz={carrier_hz-requested_hz:+.2f}")
print(f"selected_carrier_hz={carrier_hz:+.2f}")

target_rate = 100
decimation = sample_rate // target_rate
assert decimation * target_rate == sample_rate
envelopes = []
phase = 0
with wave.open(path, "rb") as wav:
    while True:
        raw = wav.readframes(250000)
        values = np.frombuffer(raw, dtype="<i2")
        if values.size == 0:
            break
        iq = values[0::2].astype(np.float64) + 1j * values[1::2].astype(np.float64)
        indices = np.arange(iq.size) + phase
        mixed = iq * np.exp(-2j * np.pi * carrier_hz * indices / sample_rate)
        usable = (mixed.size // decimation) * decimation
        # Coherent integration provides the narrow carrier filter; the later
        # 10 ms smoothing limits the keying-envelope bandwidth further.
        reduced = mixed[:usable].reshape(-1, decimation).mean(axis=1)
        envelopes.append(np.abs(reduced))
        phase += iq.size

envelope = np.concatenate(envelopes)
smooth_n = 1
low = float(np.percentile(envelope, 20))
high = float(np.percentile(envelope, 90))
threshold = low + 0.45 * (high - low)
keyed = envelope >= threshold

changes = np.flatnonzero(np.diff(keyed.astype(np.int8)) != 0) + 1
edges = np.concatenate(([0], changes, [keyed.size]))
durations_ms = np.diff(edges) * 1000.0 / target_rate
states = keyed[edges[:-1]]
marks = durations_ms[states]
gaps = durations_ms[~states]

def describe(label, values):
    values = values[(values >= 5) & (values <= 2000)]
    if values.size == 0:
        print(f"{label}=none")
        return
    percentiles = np.percentile(values, [10, 25, 50, 75, 90])
    print(f"{label}_count={values.size} p10/p25/p50/p75/p90_ms=" + "/".join(f"{x:.1f}" for x in percentiles))
    hist, bins = np.histogram(values, bins=np.arange(0, 1000 + 10, 10))
    top = np.argsort(hist)[-8:][::-1]
    print(f"{label}_histogram_peaks=" + ", ".join(f"{bins[i]:.0f}-{bins[i+1]:.0f}ms:{hist[i]}" for i in top if hist[i]))

print(f"envelope_p20={low:.2f} p90={high:.2f} ratio={high/max(low,1e-12):.2f} threshold={threshold:.2f}")
describe("marks", marks)
describe("gaps", gaps)

accepted_marks = marks[(marks >= 30.0) & (marks <= 2000.0)]
best_candidate = None
best_cost = float("inf")
for candidate in np.arange(25.0, 241.0, 1.0):
    errors = np.minimum(
        np.abs(accepted_marks - candidate) / candidate,
        np.abs(accepted_marks - 3.0 * candidate) / (3.0 * candidate),
    )
    cost = float(np.minimum(errors, 1.0).mean())
    if cost < best_cost:
        best_cost = cost
        best_candidate = candidate
print(f"live_estimator_fit_dot_ms={best_candidate:.1f} fit_wpm={1200.0/best_candidate:.1f} cost={best_cost:.3f}")

# Remove sub-30 ms marks and gaps, mirroring the live decoder's symmetrical
# debounce, then decode using the clearly measured 50/150 ms timing clusters.
# This is a signal-quality sanity check, not a full DSP-path simulation.
cleaned = keyed.copy()
for state_to_remove in (True, False):
    changes = np.flatnonzero(np.diff(cleaned.astype(np.int8)) != 0) + 1
    edges = np.concatenate(([0], changes, [cleaned.size]))
    for start, end in zip(edges[:-1], edges[1:]):
        if cleaned[start] == state_to_remove and (end - start) * 1000.0 / target_rate < 30.0:
            cleaned[start:end] = not state_to_remove

morse_table = {
    ".-":"A", "-...":"B", "-.-.":"C", "-..":"D", ".":"E", "..-.":"F",
    "--.":"G", "....":"H", "..":"I", ".---":"J", "-.-":"K", ".-..":"L",
    "--":"M", "-.":"N", "---":"O", ".--.":"P", "--.-":"Q", ".-.":"R",
    "...":"S", "-":"T", "..-":"U", "...-":"V", ".--":"W", "-..-":"X",
    "-.--":"Y", "--..":"Z", "-----":"0", ".----":"1", "..---":"2",
    "...--":"3", "....-":"4", ".....":"5", "-....":"6", "--...":"7",
    "---..":"8", "----.":"9"
}
changes = np.flatnonzero(np.diff(cleaned.astype(np.int8)) != 0) + 1
edges = np.concatenate(([0], changes, [cleaned.size]))
symbols = ""
decoded = ""
for start, end in zip(edges[:-1], edges[1:]):
    duration = (end - start) * 1000.0 / target_rate
    if cleaned[start]:
        symbols += "-" if duration >= 100.0 else "."
    elif symbols and duration >= 100.0:
        decoded += morse_table.get(symbols, "?")
        symbols = ""
        if duration >= 350.0 and decoded and decoded[-1] != " ":
            decoded += " "
if symbols:
    decoded += morse_table.get(symbols, "?")
print(f"offline_timing_decode={decoded}")

# Replay the thresholded envelope through the live decoder's debounce, timing
# fit, and character/word boundary rules.
live_text = ""
live_symbol = ""
live_dot = 60.0
live_marks = []
live_key = False
live_previous = False
live_raw = False
live_raw_blocks = 0
live_state_blocks = 0
live_character_committed = False
live_word_committed = False

def live_commit():
    global live_text, live_symbol
    if live_symbol:
        live_text += morse_table.get(live_symbol, "?")
        live_symbol = ""

for raw_down in keyed:
    raw_down = bool(raw_down)
    if raw_down == live_raw:
        live_raw_blocks += 1
    else:
        live_raw = raw_down
        live_raw_blocks = 1
    down = live_raw if live_raw_blocks >= 3 else live_key

    if down == live_previous:
        live_state_blocks += 1
    else:
        duration = live_state_blocks * 10.0
        if live_previous and live_state_blocks > 0:
            if duration < 30.0:
                live_state_blocks = 1
                live_previous = down
                live_key = down
                continue
            live_marks.append(duration)
            live_marks = live_marks[-24:]
            if len(live_marks) >= 3:
                fit_dot = None
                fit_cost = float("inf")
                for candidate in np.arange(25.0, 241.0, 1.0):
                    errors = [min(abs(mark-candidate)/candidate,
                                  abs(mark-3*candidate)/(3*candidate), 1.0)
                              for mark in live_marks]
                    cost = sum(errors) / len(errors)
                    if cost < fit_cost:
                        fit_cost = cost
                        fit_dot = candidate
                live_dot = min(240.0, max(25.0, live_dot * 0.65 + fit_dot * 0.35))
            live_symbol += "-" if duration > live_dot * 2.0 else "."
            live_character_committed = False
            live_word_committed = False
        live_state_blocks = 1
        live_previous = down

    if not down:
        gap = live_state_blocks * 10.0
        if not live_character_committed and gap >= live_dot * 2.2:
            live_commit()
            live_character_committed = True
        if not live_word_committed and gap >= live_dot * 6.0:
            if live_text and not live_text.endswith(" "):
                live_text += " "
            live_word_committed = True
    live_key = down

live_commit()
print(f"live_state_machine_decode={live_text}")
print(f"live_final_dot_ms={live_dot:.1f} live_final_wpm={1200.0/live_dot:.1f}")
