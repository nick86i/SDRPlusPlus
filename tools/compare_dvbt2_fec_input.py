"""Independently reconstruct DVB-T2 QAM LLRs and the LDPC input permutation."""

import struct
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FEC_TRACE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_fec_input.bin"
TIME_TRACE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_time_deinterleaver.bin"
NORM = np.float32(0.076696499)
ROTATION = np.float32(0.062418810)


def round_away(values):
    return np.copysign(np.floor(np.abs(values) + np.float32(0.5)), values)


def normalize(cells):
    magnitudes = np.sqrt(cells.real * cells.real + cells.imag * cells.imag, dtype=np.float32)
    middle = len(magnitudes) // 2
    median = np.partition(magnitudes, middle)[middle]
    cells *= np.float32(1.0) / median
    radii = np.sqrt(cells.real * cells.real + cells.imag * cells.imag, dtype=np.float32)
    mask = radii > np.float32(2.2)
    cells[mask] *= (np.float32(2.2) / radii[mask]).astype(np.float32)


def bit_addresses():
    twist = np.asarray((0, 2, 2, 2, 2, 3, 7, 15, 16, 20, 22, 22, 27, 27, 28, 32), dtype=np.int32)
    demux = np.asarray((3, 15, 1, 7, 4, 11, 5, 0, 12, 2, 9, 14, 13, 6, 8, 10), dtype=np.int32)
    columns = 4050
    column_address = np.empty(64800, dtype=np.int32)
    for column in range(columns):
        column_address[column * 16:column * 16 + 16] = (
            columns * np.arange(16, dtype=np.int32) + (column + columns - twist) % columns)
    raw = np.arange(64800, dtype=np.int32)
    return column_address[(raw // 16) * 16 + demux[raw % 16]]


def main():
    fec_raw = FEC_TRACE.read_bytes()
    magic, version, blocks, fec_bits, modulation, code_rate, rotation = struct.unpack_from("<IIiiiii", fec_raw)
    raw_count, fec_count = struct.unpack_from("<QQ", fec_raw, 32)
    assert magic == 0x32434546 and version == 1
    captured_raw = np.frombuffer(fec_raw, dtype=np.int8, count=raw_count, offset=48)
    captured_fec = np.frombuffer(fec_raw, dtype=np.int8, count=fec_count, offset=48 + raw_count)

    time_raw = TIME_TRACE.read_bytes()
    output_cells = struct.unpack_from("<Q", time_raw, 40)[0]
    cells = np.frombuffer(time_raw, dtype=np.complex64, count=output_cells, offset=48).copy()
    if rotation:
        cs, sn = np.cos(-ROTATION, dtype=np.float32), np.sin(-ROTATION, dtype=np.float32)
        re = cells.real * cs - cells.imag * sn
        im = cells.real * sn + cells.imag * cs
        cells = (re + np.complex64(1j) * im).astype(np.complex64)
    normalize(cells)

    absolute = np.abs(cells.real)
    levels_re = np.clip(np.floor((absolute / NORM - 1) * np.float32(0.5) + np.float32(0.5)), 0, 7)
    absolute_i = np.abs(cells.imag)
    levels_im = np.clip(np.floor((absolute_i / NORM - 1) * np.float32(0.5) + np.float32(0.5)), 0, 7)
    ideal_re = np.copysign((2 * levels_re + 1) * NORM, cells.real)
    ideal_im = np.copysign((2 * levels_im + 1) * NORM, cells.imag)
    signal = np.sum(ideal_re.astype(np.float64) ** 2 + ideal_im.astype(np.float64) ** 2)
    error = np.sum((cells.real - ideal_re).astype(np.float64) ** 2 +
                   (cells.imag - ideal_im).astype(np.float64) ** 2)
    precision = np.float32(np.clip(np.float32(8.0) * NORM * np.float32(signal / max(error, 1e-9)), 2, 120))

    l2r, l2i = np.abs(cells.real) - 8 * NORM, np.abs(cells.imag) - 8 * NORM
    l4r, l4i = np.abs(l2r) - 4 * NORM, np.abs(l2i) - 4 * NORM
    planes = np.stack((cells.real, cells.imag, l2r, l2i, l4r, l4i,
                       np.abs(l4r) - 2 * NORM, np.abs(l4i) - 2 * NORM), axis=1).reshape(-1)
    expected_raw = np.clip(round_away(planes * precision), -127, 127).astype(np.int8)

    address = bit_addresses()
    expected_fec = np.empty_like(expected_raw)
    for block in range(blocks):
        first = block * fec_bits
        expected_fec[first + address] = expected_raw[first:first + fec_bits]

    raw_value_mismatch = np.count_nonzero(expected_raw != captured_raw)
    raw_sign_mismatch = np.count_nonzero((expected_raw < 0) != (captured_raw < 0))
    fec_value_mismatch = np.count_nonzero(expected_fec != captured_fec)
    fec_sign_mismatch = np.count_nonzero((expected_fec < 0) != (captured_fec < 0))
    print("blocks", blocks, "raw/fec values", raw_count, fec_count, "precision", float(precision))
    print("raw exact-value mismatches", raw_value_mismatch, "hard-bit mismatches", raw_sign_mismatch)
    print("FEC exact-value mismatches", fec_value_mismatch, "hard-bit mismatches", fec_sign_mismatch)


if __name__ == "__main__":
    main()
