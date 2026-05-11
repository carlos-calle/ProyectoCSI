#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grid search for the Semana 3 CSI segmenter parameters."""

from __future__ import annotations

from itertools import product
from statistics import fmean

from segmentation_test import SCENARIOS, run_test_scenario


def float_range(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    current = start
    while current < stop - 1e-9:
        values.append(round(current, 2))
        current += step
    return values


def evaluate_params(
    threshold_start: float,
    threshold_stop: float,
    hysteresis_samples: int,
    max_segment_length: int,
) -> dict:
    scenario_results = []
    for scenario in SCENARIOS:
        result = run_test_scenario(
            scenario["scenario_id"],
            scenario["samples_per_activity"],
            scenario["static_samples_before"],
            scenario["static_samples_between"],
            scenario["static_samples_after"],
            variance_thresh_start=threshold_start,
            variance_thresh_stop=threshold_stop,
            hysteresis_samples=hysteresis_samples,
            max_segment_length=max_segment_length,
        )
        scenario_results.append(result)

    f1_scores = [r["metrics"]["f1_score"] for r in scenario_results]
    precision_scores = [r["metrics"]["precision"] for r in scenario_results]
    recall_scores = [r["metrics"]["recall"] for r in scenario_results]

    return {
        "params": {
            "threshold_start": threshold_start,
            "threshold_stop": threshold_stop,
            "hysteresis_samples": hysteresis_samples,
            "max_segment_length": max_segment_length,
        },
        "f1_scores": f1_scores,
        "avg_f1": round(fmean(f1_scores), 4),
        "avg_precision": round(fmean(precision_scores), 4),
        "avg_recall": round(fmean(recall_scores), 4),
    }


def main() -> None:
    threshold_starts = [2.50, 2.60, 2.70, 2.80]
    threshold_stops = [2.30, 2.40, 2.50, 2.60]
    hysteresis_values = [10]
    max_segment_lengths = [100, 120, 150, 180]

    results = []
    tested = 0
    for threshold_start, threshold_stop, hysteresis, max_len in product(
        threshold_starts,
        threshold_stops,
        hysteresis_values,
        max_segment_lengths,
    ):
        if threshold_stop >= threshold_start:
            continue

        tested += 1
        results.append(
            evaluate_params(threshold_start, threshold_stop, hysteresis, max_len)
        )
        if tested % 10 == 0:
            print(f"Evaluadas {tested} combinaciones...", flush=True)

    results.sort(key=lambda item: item["avg_f1"], reverse=True)

    print("=== BUSQUEDA DE PARAMETROS M1 ===")
    print(f"Combinaciones evaluadas: {tested}\n")
    print("TOP 10")
    print("-" * 80)
    for idx, result in enumerate(results[:10], start=1):
        params = result["params"]
        print(
            f"{idx}. start={params['threshold_start']:.2f} "
            f"stop={params['threshold_stop']:.2f} "
            f"hist={params['hysteresis_samples']} "
            f"max_len={params['max_segment_length']} "
            f"avg_f1={result['avg_f1']:.4f} "
            f"avg_p={result['avg_precision']:.4f} "
            f"avg_r={result['avg_recall']:.4f}"
        )
        print(f"   f1_scores={result['f1_scores']}")

    best = results[0]
    print("\nMEJOR CONFIGURACION")
    print("-" * 80)
    for key, value in best["params"].items():
        print(f"{key}: {value}")
    print(f"avg_f1: {best['avg_f1']:.4f}")


if __name__ == "__main__":
    main()
