from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "root_dev" / "recordings" / "dvbt2_output.ts"


def main():
    data = PATH.read_bytes()
    best_offset = max(range(188), key=lambda offset: sum(data[i] == 0x47 for i in range(offset, len(data), 188)))
    packets = [data[i:i + 188] for i in range(best_offset, len(data) - 187, 188)]
    sync_errors = sum(packet[0] != 0x47 for packet in packets)
    pids = Counter()
    continuity_errors = Counter()
    last_cc = {}
    payload_starts = defaultdict(int)
    for packet in packets:
        if packet[0] != 0x47:
            continue
        pid = ((packet[1] & 0x1F) << 8) | packet[2]
        pids[pid] += 1
        if packet[1] & 0x40:
            payload_starts[pid] += 1
        afc = (packet[3] >> 4) & 3
        cc = packet[3] & 0x0F
        has_payload = afc in (1, 3)
        if has_payload and pid != 0x1FFF and pid in last_cc and cc != ((last_cc[pid] + 1) & 0x0F):
            continuity_errors[pid] += 1
        if has_payload:
            last_cc[pid] = cc
    print(f"bytes={len(data)} offset={best_offset} packets={len(packets)} sync_errors={sync_errors}")
    print("top PIDs:")
    for pid, count in pids.most_common(20):
        print(f"  0x{pid:04X}: packets={count} payload_starts={payload_starts[pid]} continuity_errors={continuity_errors[pid]}")
    print("total continuity errors:", sum(continuity_errors.values()))


if __name__ == "__main__":
    main()
