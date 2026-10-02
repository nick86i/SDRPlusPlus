import math
import re
import struct
import itertools
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RECORDINGS = ROOT / "root_dev" / "recordings"


def complex_file(name, header_size):
    raw = (RECORDINGS / name).read_bytes()
    values = np.frombuffer(raw, dtype="<f4", offset=header_size)
    return raw[:header_size], values.reshape(-1, 2).copy()


def qam256_mer(cells, rotate=-0.062418810):
    cs, sn = math.cos(rotate), math.sin(rotate)
    re = cells[:, 0] * cs - cells[:, 1] * sn
    im = cells[:, 0] * sn + cells[:, 1] * cs
    rms = math.sqrt(float(np.mean(re * re + im * im)))
    # Unit-average-power 256-QAM has E[I^2+Q^2] = 170 before normalization.
    scale = math.sqrt(170.0) / rms
    re, im = re * scale, im * scale
    ideal_re = np.sign(re) * (2 * np.clip(np.rint((np.abs(re) - 1) / 2), 0, 7) + 1)
    ideal_im = np.sign(im) * (2 * np.clip(np.rint((np.abs(im) - 1) / 2), 0, 7) + 1)
    signal = np.sum(ideal_re * ideal_re + ideal_im * ideal_im)
    error = np.sum((re - ideal_re) ** 2 + (im - ideal_im) ** 2)
    return 10.0 * math.log10(float(signal / error))


def generate_cell_permutation(blocks, cells):
    degree = math.ceil(math.log2(cells))
    logic = {11: (0, 3), 12: (0, 2), 13: (0, 1, 4, 6),
             14: (0, 1, 4, 5, 9, 11), 15: (0, 1, 2, 12)}[degree]
    mask = (1 << (degree - 1)) - 1
    lfsr = 0
    first = []
    for i in range(1 << degree):
        if i < 2:
            lfsr = 0
        elif i == 2:
            lfsr = 1
        else:
            result = 0
            for bit in logic:
                result ^= (lfsr >> bit) & 1
            lfsr = (lfsr & mask) >> 1
            lfsr |= result << (degree - 2)
        lfsr |= (i & 1) << (degree - 1)
        if lfsr < cells:
            first.append(lfsr)
    out = np.empty(blocks * cells, dtype=np.int32)
    address = 0
    n = 0
    for block in range(blocks):
        shift = cells
        while shift >= cells:
            temp = n
            n += 1
            shift = 0
            for _ in range(degree):
                shift |= temp & 1
                shift <<= 1
                temp >>= 1
        base = block * cells
        for value in first:
            out[((value + shift) % cells) + base] = address
            address += 1
    return out


def bit_deinterleave(soft, inverse_twist=False, inverse_demux=False):
    twist = (0, 2, 2, 2, 2, 3, 7, 15, 16, 20, 22, 22, 27, 27, 28, 32)
    demux = (3, 15, 1, 7, 4, 11, 5, 0, 12, 2, 9, 14, 13, 6, 8, 10)
    if inverse_demux:
        inverse = [0] * 16
        for index, value in enumerate(demux):
            inverse[value] = index
        demux = tuple(inverse)
    columns = 4050
    address = np.empty(64800, dtype=np.int32)
    k = n = 0
    for i in range(64800):
        row = demux[n]
        signed_twist = twist[row] if inverse_twist else -twist[row]
        address[i] = columns * row + (k // 16 + columns + signed_twist) % columns
        n += 1
        if n == 16:
            n = 0
            k += 16
    out = np.empty_like(soft)
    for block in range(len(soft) // 64800):
        base = block * 64800
        out[base + address] = soft[base:base + 64800]
    return out


def ldpc_edges():
    text = (ROOT / "external" / "sdr_receiver_dvb_t2" / "src" / "DVB_T2" /
            "LDPC" / "dvb_t2_tables.hh").read_text()
    body = text[text.index("struct DVB_T2_TABLE_NORMAL_C2_3"):]
    match = re.search(r"POS\[\]\s*=\s*\{(.*?)\};", body, re.S)
    pos = [int(value) for value in re.findall(r"\d+", match.group(1))]
    degrees = [13] * 12 + [3] * 108
    bit_indices = []
    check_indices = []
    offset = 0
    offset = 0
    systematic_bit = 0
    for degree in degrees:
        base = pos[offset:offset + degree]
        offset += degree
        for shift in range(360):
            for value in base:
                bit_indices.append(systematic_bit)
                check_indices.append((value + 60 * shift) % 21600)
            systematic_bit += 1
    assert systematic_bit == 43200 and offset == len(pos)
    return np.asarray(bit_indices, dtype=np.int32), np.asarray(check_indices, dtype=np.int32)


def raw_ldpc_syndromes(fec, limit=None):
    bit_indices, check_indices = ldpc_edges()
    result = []
    blocks = fec.reshape(-1, 64800)
    if limit is not None:
        blocks = blocks[:limit]
    for block in blocks:
        bits = block < 0
        checks = np.zeros(21600, dtype=np.uint8)
        np.bitwise_xor.at(checks, check_indices, bits[bit_indices])
        parity = bits[43200:].reshape(60, 360)
        previous = np.empty_like(parity)
        previous[1:] = parity[:-1]
        previous[0, 0] = False
        previous[0, 1:] = parity[-1, :-1]
        result.append(int(np.count_nonzero(checks.reshape(360, 60).T ^ parity ^ previous)))
    return np.asarray(result)


def structural_sweep(soft):
    sample_blocks = min(16, len(soft) // 64800)
    sample = soft[:sample_blocks * 64800]
    print(f"structural sweep on {sample_blocks} identical FEC block positions")
    candidates = []
    for inverse_twist in (False, True):
        for inverse_demux in (False, True):
            candidate = bit_deinterleave(sample, inverse_twist, inverse_demux)
            score = float(raw_ldpc_syndromes(candidate).mean())
            candidates.append((score, f"twist={'inverse' if inverse_twist else 'standard'} "
                                      f"demux={'inverse' if inverse_demux else 'standard'}"))

    planes = sample.reshape(-1, 8)
    for permutation in itertools.permutations(range(4)):
        order = np.asarray([axis for pair in permutation for axis in (2 * pair, 2 * pair + 1)])
        candidate_soft = planes[:, order].reshape(-1)
        candidate = bit_deinterleave(candidate_soft)
        score = float(raw_ldpc_syndromes(candidate).mean())
        candidates.append((score, f"pair permutation {permutation}"))

    for mask in range(16):
        order = []
        for pair in range(4):
            swap = (mask >> pair) & 1
            order.extend((2 * pair + swap, 2 * pair + (swap ^ 1)))
        candidate_soft = planes[:, order].reshape(-1)
        candidate = bit_deinterleave(candidate_soft)
        score = float(raw_ldpc_syndromes(candidate).mean())
        candidates.append((score, f"I/Q swap mask 0x{mask:X}"))

    for score, label in sorted(candidates)[:12]:
        print(f"  {score:8.2f}  {label}")


def hard_demap_qam256(cells):
    norm = 0.076696499
    cs, sn = math.cos(-0.062418810), math.sin(-0.062418810)
    re = cells[:, 0] * cs - cells[:, 1] * sn
    im = cells[:, 0] * sn + cells[:, 1] * cs
    scale = 1.0 / math.sqrt(float(np.mean(re * re + im * im)))
    re, im = re * scale, im * scale
    l2r, l2i = np.abs(re) - 8 * norm, np.abs(im) - 8 * norm
    l4r, l4i = np.abs(l2r) - 4 * norm, np.abs(l2i) - 4 * norm
    values = np.column_stack((re, im, l2r, l2i, l4r, l4i,
                              np.abs(l4r) - 2 * norm, np.abs(l4i) - 2 * norm))
    return np.where(values >= 0, 1, -1).astype(np.int8).reshape(-1)


def rebuild_time_variant(extracted, blocks, ti_length, cells_per_block,
                         inverse=False, q_shift=-1, transpose=False, continuous=False):
    permutation = generate_cell_permutation(blocks, cells_per_block)
    inverse_permutation = np.empty_like(permutation)
    inverse_permutation[permutation] = np.arange(len(permutation), dtype=np.int32)
    base_blocks, remainder = divmod(blocks, ti_length)
    rows = cells_per_block // 5
    input_offset = completed_blocks = 0
    rebuilt = []
    for ti in range(ti_length):
        count = base_blocks + (1 if ti >= ti_length - remainder else 0)
        size = count * cells_per_block
        columns = size // rows
        out = np.empty((size, 2), dtype=np.float32)
        for i in range(size):
            row, column = divmod(i, columns)
            d = row * columns + column if transpose else column * rows + row
            perm_base = completed_blocks * cells_per_block if continuous else 0
            table = inverse_permutation if inverse else permutation
            address = int(table[d + perm_base]) - perm_base
            block_base = address // cells_per_block * cells_per_block
            local = address - block_base
            q_local = (local + q_shift) % cells_per_block
            out[address, 0] = extracted[input_offset + i, 0]
            out[block_base + q_local, 1] = extracted[input_offset + i, 1]
        rebuilt.append(out)
        input_offset += size
        completed_blocks += count
    return np.concatenate(rebuilt)


def time_structure_sweep(extracted, blocks, ti_length, cells_per_block):
    variants = [
        (False, -1, False, False, "standard"),
        (False, 0, False, False, "Q delay 0"),
        (False, 1, False, False, "Q delay +1"),
        (True, -1, False, False, "inverse cell permutation"),
        (False, -1, True, False, "transposed TI traversal"),
        (False, -1, False, True, "continuous permutation index"),
    ]
    scores = []
    for inverse, q_shift, transpose, continuous, label in variants:
        cells = rebuild_time_variant(extracted, blocks, ti_length, cells_per_block,
                                     inverse, q_shift, transpose, continuous)
        soft = hard_demap_qam256(cells)
        candidate = bit_deinterleave(soft)
        score = float(raw_ldpc_syndromes(candidate, limit=16).mean())
        scores.append((score, label))
    print("time/cell structural sweep on 16 identical FEC block positions")
    for score, label in sorted(scores):
        print(f"  {score:8.2f}  {label}")


def main():
    frame_header, frame = complex_file("dvbt2_debug_frame.bin", 80)
    fields = struct.unpack_from("<II10i4Q", frame_header)
    (_, version, plp_start, blocks, modulation, code_rate, fec_type, rotation,
     max_blocks, ti_length, ti_type, frequency_mapping,
     frame_cells, p2_cells, data_cells, closing_cells) = fields
    print("frame", dict(version=version, start=plp_start, blocks=blocks,
                        modulation=modulation, code_rate=code_rate, fec_type=fec_type,
                        rotation=rotation, max_blocks=max_blocks, ti_length=ti_length,
                        ti_type=ti_type, frequency_mapping=frequency_mapping,
                        cells=frame_cells, p2=p2_cells, data=data_cells, closing=closing_cells))

    all_header, all_values = complex_file("dvbt2_debug_all_symbols.bin", 40)
    _, _, ordinary, per_symbol, closing_count, n_p2, closing_symbol, all_count = \
        struct.unpack_from("<II5i4xQ", all_header)
    raw_symbols = all_values[:all_count]
    mapped_symbols = all_values[all_count:]
    expected_tail = frame[p2_cells:p2_cells + all_count]
    print("frame tail == frequency trace:", np.array_equal(expected_tail, mapped_symbols),
          "max abs delta", float(np.max(np.abs(expected_tail - mapped_symbols))))
    print("symbol trace", dict(ordinary=ordinary, per_symbol=per_symbol,
                               closing=closing_count, n_p2=n_p2,
                               closing_symbol=closing_symbol, cells=all_count))

    required = blocks * 8100
    extracted = frame[plp_start:plp_start + required]
    time_header, traced_time = complex_file("dvbt2_debug_time_deinterleaver.bin", 48)
    _, _, t_blocks, t_max, t_length, t_type, cells_per_block, input_cells, output_cells = \
        struct.unpack_from("<II5i4x2Q", time_header)
    permutation = generate_cell_permutation(max(t_max, t_blocks), cells_per_block)
    rebuilt = []
    source_indices = []
    input_offset = 0
    base_blocks, remainder = divmod(t_blocks, t_length)
    rows = cells_per_block // 5
    for ti in range(t_length):
        count = base_blocks + (1 if ti >= t_length - remainder else 0)
        size = count * cells_per_block
        out = np.empty((size, 2), dtype=np.float32)
        provenance = np.empty(size, dtype=np.int64)
        step = row = 0
        for i in range(size):
            address = int(permutation[step + row])
            block_base = address // cells_per_block * cells_per_block
            local = address - block_base
            q_local = (local - 1) % cells_per_block
            out[address, 0] = extracted[input_offset + i, 0]
            out[block_base + q_local, 1] = extracted[input_offset + i, 1]
            provenance[address] = input_offset + i
            step += rows
            if step == size:
                step = 0
                row += 1
        rebuilt.append(out)
        source_indices.append(provenance)
        input_offset += size
    rebuilt = np.concatenate(rebuilt)
    source_indices = np.concatenate(source_indices)
    print("time trace == independently rebuilt:", np.array_equal(traced_time, rebuilt),
          "max abs delta", float(np.max(np.abs(traced_time - rebuilt))))

    fec_raw = (RECORDINGS / "dvbt2_debug_fec_input.bin").read_bytes()
    _, _, f_blocks, f_bits, f_mod, f_code, f_rotation, raw_count, fec_count = \
        struct.unpack_from("<II5i4x2Q", fec_raw)
    soft = np.frombuffer(fec_raw, dtype=np.int8, count=raw_count, offset=48)
    fec = np.frombuffer(fec_raw, dtype=np.int8, count=fec_count, offset=48 + raw_count)
    print("FEC trace", dict(blocks=f_blocks, bits=f_bits, modulation=f_mod,
                            code=f_code, rotation=f_rotation, raw=raw_count, fec=fec_count))
    rebuilt_fec = bit_deinterleave(soft)
    print("FEC trace == independently bit-deinterleaved:", np.array_equal(fec, rebuilt_fec),
          "different bytes", int(np.count_nonzero(fec != rebuilt_fec)))

    sections = (("P2", frame[:p2_cells]),
                ("ordinary data", frame[p2_cells:p2_cells + data_cells]),
                ("closing", frame[p2_cells + data_cells:p2_cells + data_cells + closing_cells]))
    for label, cells in sections:
        print(f"MER {label}: {qam256_mer(cells):.2f} dB ({len(cells)} cells)")

    p2_limit = p2_cells - plp_start
    p2_per_fec = np.zeros(blocks, dtype=np.int32)
    for output_index, input_index in enumerate(source_indices):
        if input_index < p2_limit:
            p2_per_fec[output_index // cells_per_block] += 1
    nonzero = p2_per_fec[p2_per_fec > 0]
    print("P2 cells spread across FEC blocks:", len(nonzero), "/", blocks,
          "min/max/mean", int(nonzero.min()), int(nonzero.max()), float(nonzero.mean()))
    print("soft/fec finite and nonzero:", bool(np.any(soft)), bool(np.any(fec)))
    syndromes = raw_ldpc_syndromes(fec)
    touched = p2_per_fec > 0
    print("raw LDPC unsatisfied all min/mean/max:", int(syndromes.min()),
          float(syndromes.mean()), int(syndromes.max()))
    print("raw LDPC P2-touched mean:", float(syndromes[touched].mean()),
          "untouched mean:", float(syndromes[~touched].mean()))
    structural_sweep(soft)
    time_structure_sweep(extracted, blocks, ti_length, cells_per_block)


if __name__ == "__main__":
    main()
