"""Exhaustively rank feasible DVB-T2 PLP cell-start offsets by LDPC syndrome."""

import numpy as np

import analyze_dvbt2_debug as analyzer


def check_index_matrix():
    checks = [[] for _ in range(21600)]
    for bit, positions in enumerate(analyzer.LINKS):
        for position in positions:
            checks[int(position)].append(bit)
    for group in range(60):
        for row in range(360):
            check = row * 60 + group
            checks[check].append(43200 + group * 360 + row)
            if group:
                checks[check].append(43200 + (group - 1) * 360 + row)
            elif row:
                checks[check].append(43200 + 59 * 360 + row - 1)
    width = max(map(len, checks))
    indices = np.full((21600, width), 64800, dtype=np.int32)
    for check, values in enumerate(checks):
        indices[check, :len(values)] = values
    return indices


def first_fec_input_addresses(blocks=66):
    size = blocks * 8100
    rows = 1620
    columns = blocks * 5
    sequence = np.arange(size, dtype=np.int32)
    order = (sequence % columns) * rows + sequence // columns
    addresses = analyzer.PERM[:size][order]
    inverse = np.empty(size, dtype=np.int32)
    inverse[addresses] = sequence
    real = inverse[:8100]
    imag = inverse[np.roll(np.arange(8100, dtype=np.int32), -1)]
    return real, imag


def raw_bits(cells):
    cells = cells * np.exp(-1j * np.float32(0.062418810))
    cells /= np.median(np.abs(cells), axis=1)[:, None]
    norm = np.float32(0.076696499)
    re, im = cells.real, cells.imag
    planes = np.stack((
        re, im,
        np.abs(re) - 8 * norm, np.abs(im) - 8 * norm,
        np.abs(np.abs(re) - 8 * norm) - 4 * norm,
        np.abs(np.abs(im) - 8 * norm) - 4 * norm,
        np.abs(np.abs(np.abs(re) - 8 * norm) - 4 * norm) - 2 * norm,
        np.abs(np.abs(np.abs(im) - 8 * norm) - 4 * norm) - 2 * norm,
    ), axis=2).reshape(len(cells), 64800) < 0
    result = np.empty_like(planes)
    result[:, analyzer.BIT_ADDRESS] = planes
    return result


def main():
    check_indices = check_index_matrix()
    real_addresses, imag_addresses = first_fec_input_addresses()
    cells = analyzer.cells
    maximum_offset = len(cells) - 133 * 8100
    best = []
    batch_size = 32
    for first in range(0, maximum_offset + 1, batch_size):
        offsets = np.arange(first, min(first + batch_size, maximum_offset + 1), dtype=np.int32)
        candidate = cells[offsets[:, None] + real_addresses[None, :]].real.astype(np.complex64)
        candidate.imag = cells[offsets[:, None] + imag_addresses[None, :]].imag
        bits = raw_bits(candidate)
        bits = np.pad(bits, ((0, 0), (0, 1)))
        syndrome = np.bitwise_xor.reduce(bits[:, check_indices], axis=2).sum(axis=1)
        best.extend((int(score), int(offset)) for score, offset in zip(syndrome, offsets))
        if first % 1024 == 0:
            score, offset = min(best)
            print("progress", first, "best", offset, score, flush=True)
    print("BEST OFFSETS")
    for score, offset in sorted(best)[:25]:
        print(offset, score)


if __name__ == "__main__":
    main()
