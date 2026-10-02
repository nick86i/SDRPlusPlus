import re
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_frame.bin"
TABLE = ROOT / "external" / "sdr_receiver_dvb_t2" / "src" / "DVB_T2" / "LDPC" / "dvb_t2_tables.hh"


def numbers(body):
    return [int(value) for value in re.findall(r"-?\d+", body)]


def table_arrays():
    text = TABLE.read_text(encoding="utf-8")
    block = re.search(r"struct DVB_T2_TABLE_NORMAL_C2_3\s*\{(.*?)\n\};", text, re.S).group(1)
    result = {}
    for name in ("DEG", "LEN", "POS"):
        result[name] = numbers(re.search(rf"{name}\[\]\s*=\s*\{{(.*?)\}}", block, re.S).group(1))
    return result


def ldpc_links():
    arrays = table_arrays()
    links = []
    pos_index = 0
    q = 60
    for degree, length in zip(arrays["DEG"], arrays["LEN"]):
        if degree == 0 or length == 0:
            break
        base = np.asarray(arrays["POS"][pos_index:pos_index + degree], dtype=np.int32)
        pos_index += degree
        for _ in range(length):
            for row in range(360):
                links.append((base + q * row) % 21600)
    assert len(links) == 43200
    return links


def syndrome_count(bits, links):
    syndrome = np.zeros(21600, dtype=np.uint8)
    for bit, positions in zip(bits[:43200], links):
        if bit:
            syndrome[positions] ^= 1
    parity = bits[43200:].reshape(60, 360)
    # LDPC table positions are numbered row-major (360 rows x q groups),
    # while transmitted parity bits are grouped as q x 360.
    syndrome = syndrome.reshape(360, 60).T
    syndrome ^= parity
    syndrome[1:] ^= parity[:-1]
    syndrome[0, 1:] ^= parity[-1, :-1]
    return int(syndrome.sum())


def permutation(blocks=133, cells=8100):
    degree = int(np.ceil(np.log2(cells)))
    logic = {11: [0, 3], 12: [0, 2], 13: [0, 1, 4, 6], 14: [0, 1, 4, 5, 9, 11], 15: [0, 1, 2, 12]}[degree]
    mask = (1 << (degree - 1)) - 1
    first = []
    lfsr = 0
    for i in range(1 << degree):
        if i < 2:
            lfsr = 0
        elif i == 2:
            lfsr = 1
        else:
            result = 0
            for tap in logic:
                result ^= (lfsr >> tap) & 1
            lfsr = (lfsr & mask) >> 1
            lfsr |= result << (degree - 2)
        lfsr |= (i & 1) << (degree - 1)
        if lfsr < cells:
            first.append(lfsr)
    first = np.asarray(first, dtype=np.int32)
    out = np.empty(blocks * cells, dtype=np.int32)
    n = 0
    address = 0
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
        indices = ((first + shift) % cells) + block * cells
        out[indices] = np.arange(address, address + cells, dtype=np.int32)
        address += cells
    return out


def time_deinterleave(source, perm, blocks, inverse=False, qshift=-1):
    size = blocks * 8100
    rows = 1620
    columns = blocks * 5
    order = (np.arange(size, dtype=np.int32) % columns) * rows + (np.arange(size, dtype=np.int32) // columns)
    addresses = perm[:size][order]
    if inverse:
        inv = np.empty(size, dtype=np.int32)
        inv[perm[:size]] = np.arange(size, dtype=np.int32)
        addresses = inv[order]
    output = np.empty(size, dtype=np.complex64)
    output.real[addresses] = source[:size].real
    block_base = (addresses // 8100) * 8100
    qaddr = block_base + (addresses - block_base + qshift) % 8100
    output.imag[qaddr] = source[:size].imag
    return output


def bit_address(demux):
    twist = np.asarray([0, 2, 2, 2, 2, 3, 7, 15, 16, 20, 22, 22, 27, 27, 28, 32])
    demux = np.asarray(demux)
    column_address = np.empty(64800, dtype=np.int32)
    for column in range(4050):
        column_address[column * 16:column * 16 + 16] = 4050 * np.arange(16) + (column + 4050 - twist) % 4050
    raw = np.arange(64800, dtype=np.int32)
    return column_address[(raw // 16) * 16 + demux[raw % 16]]


def hard_bits(cells, rotation, address=None, inverse_address=False, swap_iq=False, invert=False):
    cells = cells[:8100] * np.exp(1j * rotation)
    if swap_iq:
        cells = cells.imag + 1j * cells.real
    cells = cells / np.median(np.abs(cells))
    norm = np.float32(0.076696499)
    re, im = cells.real, cells.imag
    planes = np.column_stack((
        re, im,
        np.abs(re) - 8 * norm, np.abs(im) - 8 * norm,
        np.abs(np.abs(re) - 8 * norm) - 4 * norm,
        np.abs(np.abs(im) - 8 * norm) - 4 * norm,
        np.abs(np.abs(np.abs(re) - 8 * norm) - 4 * norm) - 2 * norm,
        np.abs(np.abs(np.abs(im) - 8 * norm) - 4 * norm) - 2 * norm,
    )).reshape(-1)
    raw_bits = planes < 0
    result = np.empty(64800, dtype=np.uint8)
    if address is None:
        result[:] = raw_bits
    elif inverse_address:
        result[:] = raw_bits[address]
    else:
        result[address] = raw_bits
    if invert:
        result ^= 1
    return result


raw = CAPTURE.read_bytes()
magic, version = struct.unpack_from("<II", raw)
assert magic == 0x32545644
header_size = 80 if version >= 2 else 56
if version >= 2:
    cell_count, p2_cells, data_cells, closing_cells = struct.unpack_from("<QQQQ", raw, 48)
    print("CAPTURE sections", "P2", p2_cells, "data", data_cells, "closing", closing_cells,
          "total", cell_count)
cells = np.frombuffer(raw, dtype=np.complex64, offset=header_size).copy()
assert len(cells) == 1087024
PERM = permutation()
DEMUXES = {
    "2/3": [3, 15, 1, 7, 4, 11, 5, 0, 12, 2, 9, 14, 13, 6, 8, 10],
    "3/5": [4, 6, 0, 2, 3, 14, 12, 10, 7, 5, 8, 1, 15, 9, 11, 13],
    "normal": [15, 1, 13, 3, 10, 7, 9, 11, 4, 6, 8, 5, 12, 2, 14, 0],
}
BIT_ADDRESS = bit_address(DEMUXES["2/3"])
LINKS = ldpc_links()

best = []
for offset in range(-4, 5):
    start = max(0, offset)
    for distribution in ((66, 67), (67, 66)):
        first_blocks = distribution[0]
        source = cells[start:start + first_blocks * 8100]
        if len(source) != first_blocks * 8100:
            continue
        for inverse in (False, True):
            for qshift in (-1, 0, 1):
                ti = time_deinterleave(source, PERM, first_blocks, inverse, qshift)
                for rotation in (-0.062418810, 0.0, 0.062418810):
                    bits = hard_bits(ti, rotation)
                    score = syndrome_count(bits, LINKS)
                    best.append((score, offset, distribution, inverse, qshift, rotation))

for item in sorted(best)[:25]:
    print(item)

baseline = time_deinterleave(cells[:66 * 8100], PERM, 66, False, -1)
baseline *= np.exp(-1j * 0.062418810)
baseline /= np.sqrt(np.mean(np.abs(baseline) ** 2))
block_scores = []
for block in range(len(baseline) // 8100):
    first = block * 8100
    block_scores.append(syndrome_count(hard_bits(baseline[first:first + 8100], 0.0, BIT_ADDRESS), LINKS))
print("STANDARD per-block syndrome min/median/max", min(block_scores), float(np.median(block_scores)), max(block_scores),
      "best block", int(np.argmin(block_scores)))
norm = 0.076696499
for label, axis in (("I", baseline.real), ("Q", baseline.imag)):
    levels = np.clip(np.rint((np.abs(axis) / norm - 1) / 2), 0, 7).astype(int)
    counts = np.bincount(levels, minlength=8)
    print(label, "level counts", counts.tolist(), "fractions", np.round(counts / counts.sum(), 4).tolist())
    signed = np.sign(axis) * (2 * levels + 1) * norm
    raw_evm = np.sqrt(np.mean((axis - signed) ** 2) / np.mean(signed ** 2))
    print(label, "nearest-level EVM", float(raw_evm), "decision SNR dB", float(-20 * np.log10(raw_evm)))

focused = []
for demux_name, demux in DEMUXES.items():
    address = bit_address(demux)
    for inverse_address in (False, True):
        for rotation in (-0.062418810, 0.0, 0.062418810):
            for swap_iq in (False, True):
                for invert in (False, True):
                    bits = hard_bits(baseline, rotation, address, inverse_address, swap_iq, invert)
                    focused.append((syndrome_count(bits, LINKS), demux_name, inverse_address,
                                    rotation, swap_iq, invert))
for item in sorted(focused)[:25]:
    print("BIT", item)
