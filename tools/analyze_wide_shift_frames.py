#!/usr/bin/env python3
"""Cluster captured 113-bit wide-shift FSK frames without protocol assumptions."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path


def unpack_frames(path: Path) -> list[tuple[int, ...]]:
    raw = path.read_bytes()
    if len(raw) % 15:
        raise ValueError(f"{path}: size is not a multiple of 15 bytes")
    result = []
    for offset in range(0, len(raw), 15):
        bits = []
        for value in raw[offset : offset + 15]:
            bits.extend((value >> shift) & 1 for shift in range(7, -1, -1))
        result.append(tuple(bits[:113]))
    return result


def distance(a: tuple[int, ...], b: tuple[int, ...]) -> int:
    return sum(x != y for x, y in zip(a, b))


def cluster(frames: list[tuple[int, ...]], radius: int) -> tuple[list[tuple[int, ...]], list[int]]:
    """Greedy, deterministic codebook: most frequent words become medoids first."""
    counts = Counter(frames)
    medoids: list[tuple[int, ...]] = []
    for word, _ in sorted(counts.items(), key=lambda item: (-item[1], item[0])):
        if not medoids or min(distance(word, medoid) for medoid in medoids) > radius:
            medoids.append(word)
    assignments = []
    for frame in frames:
        nearest = min(range(len(medoids)), key=lambda i: distance(frame, medoids[i]))
        assignments.append(nearest if distance(frame, medoids[nearest]) <= radius else -1)
    return medoids, assignments


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("captures", nargs="+", type=Path)
    parser.add_argument("--radius", type=int, default=3)
    args = parser.parse_args()

    sessions = [unpack_frames(path) for path in args.captures]
    # The first 30 bits are the already-known synchronization core.
    payload_sessions = [[frame[30:] for frame in frames] for frames in sessions]
    combined = [frame for session in payload_sessions for frame in session]
    medoids, combined_assignments = cluster(combined, args.radius)
    support = Counter(combined_assignments)

    print(f"sessions: {len(sessions)}; frames: {len(combined)}; radius: {args.radius}")
    print(f"codebook entries: {len(medoids)}")
    print("largest entries (ID: support, sessions, exact-support):")
    cursor = 0
    session_sets = []
    for session in payload_sessions:
        session_sets.append(set(combined_assignments[cursor : cursor + len(session)]))
        cursor += len(session)
    for state, count in support.most_common(20):
        if state < 0:
            continue
        present = sum(state in states for states in session_sets)
        exact = combined.count(medoids[state])
        print(f"  S{state:03d}: {count:4d}, {present}/{len(sessions)}, exact {exact:4d}")

    transitions: Counter[tuple[int, int]] = Counter()
    cursor = 0
    for session in payload_sessions:
        ids = combined_assignments[cursor : cursor + len(session)]
        cursor += len(session)
        transitions.update(zip(ids, ids[1:]))
    print("most common transitions:")
    for (left, right), count in transitions.most_common(20):
        print(f"  S{left:03d} -> S{right:03d}: {count}")

    # Report runs because an idle/control protocol often holds one state.
    runs = defaultdict(list)
    cursor = 0
    for session in payload_sessions:
        ids = combined_assignments[cursor : cursor + len(session)]
        cursor += len(session)
        if not ids:
            continue
        start = 0
        for i in range(1, len(ids) + 1):
            if i == len(ids) or ids[i] != ids[start]:
                runs[ids[start]].append(i - start)
                start = i
    print("longest repeated-state runs:")
    for state, lengths in sorted(runs.items(), key=lambda item: -max(item[1]))[:15]:
        print(f"  S{state:03d}: max {max(lengths)}, runs {len(lengths)}")

    print("best sequence alignments (payload Hamming distance):")
    for left_index in range(len(payload_sessions)):
        for right_index in range(left_index, len(payload_sessions)):
            left, right = payload_sessions[left_index], payload_sessions[right_index]
            candidates = []
            minimum_lag = 1 if left_index == right_index else -len(right) + 10
            for lag in range(minimum_lag, len(left) - 9):
                pairs = [(left[i], right[i - lag]) for i in range(max(0, lag), min(len(left), len(right) + lag))]
                if len(pairs) < 10:
                    continue
                exact = sum(a == b for a, b in pairs)
                near = sum(distance(a, b) <= args.radius for a, b in pairs)
                candidates.append((near / len(pairs), exact / len(pairs), len(pairs), lag))
            for near_rate, exact_rate, overlap, lag in sorted(candidates, reverse=True)[:5]:
                print(f"  session {left_index + 1} vs {right_index + 1}, lag {lag:+d}: "
                      f"near {near_rate:.1%}, exact {exact_rate:.1%}, overlap {overlap}")


if __name__ == "__main__":
    main()
