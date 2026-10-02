"""Verify every captured DVB-T2 data/closing symbol against the reference frequency map."""

import struct
from pathlib import Path

import numpy as np

from compare_dvbt2_data_symbol import reference_addresses


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "root_dev" / "recordings" / "dvbt2_debug_all_symbols.bin"
FRAME = ROOT / "root_dev" / "recordings" / "dvbt2_debug_frame.bin"


def main():
    raw = TRACE.read_bytes()
    magic, version, ordinary, data_cells, closing_cells, n_p2, closing_absolute = struct.unpack_from(
        "<IIiiiii", raw)
    total_cells = struct.unpack_from("<Q", raw, 32)[0]
    assert magic == 0x324C4C41 and version == 1
    header_size = 40
    source = np.frombuffer(raw, dtype=np.complex64, count=total_cells, offset=header_size)
    mapped = np.frombuffer(raw, dtype=np.complex64, count=total_cells,
                           offset=header_size + total_cells * 8)
    assert total_cells == ordinary * data_cells + closing_cells

    address_cache = {}
    failures = []
    offset = 0
    for symbol in range(ordinary):
        absolute = n_p2 + symbol
        use_odd = absolute % 2 == 0
        key = (data_cells, use_odd)
        if key not in address_cache:
            address_cache[key] = reference_addresses(*key)
        addresses = address_cache[key]
        expected = np.empty(data_cells, dtype=np.complex64)
        expected[addresses] = source[offset:offset + data_cells]
        mismatch = np.flatnonzero(expected != mapped[offset:offset + data_cells])
        if len(mismatch):
            failures.append(("data", symbol, absolute, int(mismatch[0]), len(mismatch)))
        offset += data_cells

    if closing_cells:
        use_odd = closing_absolute % 2 == 0
        addresses = reference_addresses(closing_cells, use_odd)
        expected = np.empty(closing_cells, dtype=np.complex64)
        expected[addresses] = source[offset:offset + closing_cells]
        mismatch = np.flatnonzero(expected != mapped[offset:offset + closing_cells])
        if len(mismatch):
            failures.append(("closing", 0, closing_absolute, int(mismatch[0]), len(mismatch)))

    frame_raw = FRAME.read_bytes()
    _, frame_version = struct.unpack_from("<II", frame_raw)
    frame_header = 80 if frame_version >= 2 else 56
    p2_cells = struct.unpack_from("<Q", frame_raw, 56)[0] if frame_version >= 2 else 19092
    frame_cells = np.frombuffer(frame_raw, dtype=np.complex64, offset=frame_header)
    assembled_match = np.array_equal(mapped, frame_cells[p2_cells:p2_cells + total_cells])

    print("ordinary symbols", ordinary, "data cells/symbol", data_cells,
          "closing cells", closing_cells, "total", total_cells)
    print("reference mapping failures", len(failures))
    for failure in failures[:10]:
        print("failure", failure)
    print("complete mapped section matches assembled frame", bool(assembled_match))


if __name__ == "__main__":
    main()
