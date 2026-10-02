"""Compare a captured pre-map DVB-T2 data symbol with an independent 32K reference map."""

import struct
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SYMBOL_CAPTURE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_symbol.bin"
FRAME_CAPTURE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_frame.bin"


def reference_addresses(cell_count, odd):
    degree = 14
    states = 32768
    mask = 0x3FFF
    taps = (0, 1, 2, 12)
    bit_permutation = (7, 13, 3, 4, 9, 2, 12, 11, 1, 8, 10, 0, 5, 6)
    maximum = np.empty(states, dtype=np.int32)
    lfsr = 0
    for index in range(states):
        if index < 2:
            lfsr = 0
        elif index == 2:
            lfsr = 1
        else:
            feedback = 0
            for tap in taps:
                feedback ^= (lfsr >> tap) & 1
            lfsr = ((lfsr & mask) >> 1) | (feedback << (degree - 1))
        value = 0
        for bit, destination in enumerate(bit_permutation):
            value |= ((lfsr >> bit) & 1) << destination
        maximum[index] = value + ((index & 1) * (states // 2))
    tx = maximum[maximum < cell_count].copy()
    if not odd:
        inverse_tx = tx.copy()
        for index, address in enumerate(tx):
            inverse_tx[address] = index
        tx = inverse_tx
    result = np.empty(cell_count, dtype=np.int32)
    result[tx] = np.arange(cell_count, dtype=np.int32)
    return result


def main():
    raw = SYMBOL_CAPTURE.read_bytes()
    magic, version, data_symbol, absolute_symbol, count, use_odd = struct.unpack_from("<IIiiii", raw)
    assert magic == 0x324D5953 and version == 1
    offset = struct.calcsize("<IIiiii")
    source = np.frombuffer(raw, dtype=np.complex64, count=count, offset=offset)
    mapped = np.frombuffer(raw, dtype=np.complex64, count=count, offset=offset + count * 8)
    addresses = reference_addresses(count, bool(use_odd))
    expected = np.empty_like(source)
    expected[addresses] = source
    mismatches = np.flatnonzero(expected != mapped)

    frame_match = "not checked"
    if FRAME_CAPTURE.exists():
        frame = FRAME_CAPTURE.read_bytes()
        _, frame_version = struct.unpack_from("<II", frame)
        frame_header = 80 if frame_version >= 2 else 56
        p2_cells = struct.unpack_from("<Q", frame, 56)[0] if frame_version >= 2 else 19092
        frame_cells = np.frombuffer(frame, dtype=np.complex64, offset=frame_header)
        frame_match = bool(np.array_equal(mapped, frame_cells[p2_cells:p2_cells + count]))

    print("symbol", data_symbol, "absolute", absolute_symbol, "cells", count,
          "map", "odd" if use_odd else "even")
    print("reference mismatches", len(mismatches))
    if len(mismatches):
        first = int(mismatches[0])
        print("first mismatch", first, "expected", expected[first], "captured", mapped[first])
    print("assembled-frame first-symbol match", frame_match)


if __name__ == "__main__":
    main()
