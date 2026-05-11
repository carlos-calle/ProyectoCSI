#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print variance ranges for the synthetic CSI generator."""

from __future__ import annotations

from statistics import fmean

from segmentation_test import EdgeSegmenter, SyntheticCSIGenerator


def main() -> None:
    generator = SyntheticCSIGenerator(seed=42)
    segmenter = EdgeSegmenter()
    variances: list[float] = []

    def process(frame):
        segmenter.process_csi_frame(frame)
        value = fmean(segmenter.variance_vector)
        variances.append(value)
        return value

    print("=== DEBUG VARIANCE M1 ===\n")

    static_values = []
    for _ in range(50):
        static_values.append(process(generator.generate_static_frame()))

    activity_values = []
    for _ in range(100):
        activity_values.append(process(generator.generate_activity_frame()))

    print("[STATIC]")
    print(f"  final: {static_values[-1]:.6f}")
    print(f"  min:   {min(static_values):.6f}")
    print(f"  max:   {max(static_values):.6f}")
    print(f"  mean:  {fmean(static_values):.6f}\n")

    print("[ACTIVITY]")
    print(f"  final: {activity_values[-1]:.6f}")
    print(f"  min:   {min(activity_values):.6f}")
    print(f"  max:   {max(activity_values):.6f}")
    print(f"  mean:  {fmean(activity_values):.6f}\n")

    print("[NAIVE VARIANCE HEURISTIC]")
    print("  Final parameters come from parameter_search.py.")
    print(f"  start around: {max(activity_values) * 0.85:.2f}")
    print(f"  stop around:  {fmean(activity_values) * 0.90:.2f}")


if __name__ == "__main__":
    main()
