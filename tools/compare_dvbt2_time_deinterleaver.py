"""Compare captured C++ DVB-T2 time deinterleaving with an independent reconstruction."""

import struct
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_time_deinterleaver.bin"
FRAME = ROOT / "root_dev" / "recordings" / "dvbt2_debug_frame.bin"


def cell_permutation(blocks, cells):
    degree = int(np.ceil(np.log2(cells)))
    taps = {11: (0, 3), 12: (0, 2), 13: (0, 1, 4, 6),
            14: (0, 1, 4, 5, 9, 11), 15: (0, 1, 2, 12)}[degree]
    mask = (1 << (degree - 1)) - 1
    first = []
    lfsr = 0
    for index in range(1 << degree):
        if index < 2:
            lfsr = 0
        elif index == 2:
            lfsr = 1
        else:
            feedback = 0
            for tap in taps:
                feedback ^= (lfsr >> tap) & 1
            lfsr = ((lfsr & mask) >> 1) | (feedback << (degree - 2))
        lfsr |= (index & 1) << (degree - 1)
        if lfsr < cells:
            first.append(lfsr)
    first = np.asarray(first, dtype=np.int32)
    result = np.empty(blocks * cells, dtype=np.int32)
    sequence = 0
    address = 0
    for block in range(blocks):
        shift = cells
        while shift >= cells:
            temporary = sequence
            sequence += 1
            shift = 0
            for _ in range(degree):
                shift |= temporary & 1
                shift <<= 1
                temporary >>= 1
        indices = ((first + shift) % cells) + block * cells
        result[indices] = np.arange(address, address + cells, dtype=np.int32)
        address += cells
    return result


def reconstruct(source, blocks, maximum_blocks, time_length, cells_per_block):
    permutation = cell_permutation(maximum_blocks, cells_per_block)
    rows = cells_per_block // 5
    base, remainder = divmod(blocks, time_length)
    output_blocks = []
    input_offset = 0
    for ti in range(time_length):
        count = base + (1 if ti >= time_length - remainder else 0)
        size = count * cells_per_block
        columns = count * 5
        sequence = np.arange(size, dtype=np.int32)
        order = (sequence % columns) * rows + sequence // columns
        addresses = permutation[:size][order]
        output = np.empty(size, dtype=np.complex64)
        segment = source[input_offset:input_offset + size]
        output.real[addresses] = segment.real
        block_base = (addresses // cells_per_block) * cells_per_block
        q_addresses = block_base + (addresses - block_base - 1) % cells_per_block
        output.imag[q_addresses] = segment.imag
        output_blocks.append(output)
        input_offset += size
    return np.concatenate(output_blocks)


def main():
    trace_raw = TRACE.read_bytes()
    fields = struct.unpack_from("<IIiiiii", trace_raw)
    magic, version, blocks, maximum, time_length, time_type, cells_per_block = fields
    input_cells, output_cells = struct.unpack_from("<QQ", trace_raw, 32)
    assert magic == 0x32495444 and version == 1 and time_type == 0
    captured = np.frombuffer(trace_raw, dtype=np.complex64, count=output_cells, offset=48)

    frame_raw = FRAME.read_bytes()
    _, frame_version, plp_start = struct.unpack_from("<IIi", frame_raw)
    frame_header = 80 if frame_version >= 2 else 56
    frame_cells = np.frombuffer(frame_raw, dtype=np.complex64, offset=frame_header)
    source = frame_cells[plp_start:plp_start + input_cells]
    expected = reconstruct(source, blocks, maximum, time_length, cells_per_block)
    mismatches = np.flatnonzero(expected != captured)

    first_ti_blocks = blocks // time_length
    second_ti_blocks = blocks - first_ti_blocks
    print("TI blocks", time_length, "FEC distribution", first_ti_blocks, second_ti_blocks,
          "cells", output_cells)
    print("complex-cell mismatches", len(mismatches))
    if len(mismatches):
        first = int(mismatches[0])
        print("first mismatch", first, "expected", expected[first], "captured", captured[first])


if __name__ == "__main__":
    main()
