#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synthetic CSI segmentation evaluation for M1 / Semana 3.

This script intentionally uses only the Python standard library so the Semana 3
deliverable can be reproduced in a clean ESP-IDF/Python environment.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
import random
from statistics import fmean
from typing import Iterable, List, Sequence, Tuple


IQFrame = List[Tuple[float, float]]


@dataclass(frozen=True)
class TimeRange:
    """Temporal range [start_idx, end_idx)."""

    start_idx: int
    end_idx: int

    def duration(self) -> int:
        return max(0, self.end_idx - self.start_idx)


@dataclass(frozen=True)
class RangeMetrics:
    precision: float
    recall: float
    f1_score: float
    tp_range: int
    fp_range: int
    fn_range: int


class SyntheticCSIGenerator:
    """Deterministic synthetic CSI stream generator."""

    def __init__(self, seed: int = 42, num_subcarriers: int = 64):
        self.rng = random.Random(seed)
        self.num_subcarriers = num_subcarriers
        self.frame_counter = 0

    def generate_static_frame(self) -> IQFrame:
        frame: IQFrame = []
        for _ in range(self.num_subcarriers):
            noise = self.rng.gauss(0.0, 0.05)
            frame.append((noise, noise))
        self.frame_counter += 1
        return frame

    def generate_activity_frame(self) -> IQFrame:
        activity_amplitude = 2.0 + 1.0 * math.sin(
            2.0 * math.pi * self.frame_counter / 50.0
        )
        frame: IQFrame = []
        for k in range(self.num_subcarriers):
            phase = (
                2.0 * math.pi * k / self.num_subcarriers
                + activity_amplitude
                * math.sin(2.0 * math.pi * self.frame_counter / 20.0)
            )
            real = activity_amplitude * math.cos(phase) + self.rng.gauss(0.0, 0.25)
            imag = activity_amplitude * math.sin(phase) + self.rng.gauss(0.0, 0.25)
            frame.append((real, imag))
        self.frame_counter += 1
        return frame

    def generate_continuous_stream(
        self,
        samples_per_activity: int,
        static_samples_before: int,
        static_samples_between: int,
        static_samples_after: int,
    ) -> Tuple[List[IQFrame], List[TimeRange]]:
        frames: List[IQFrame] = []
        ground_truth: List[TimeRange] = []
        current_frame = 0

        for _ in range(static_samples_before):
            frames.append(self.generate_static_frame())
            current_frame += 1

        activity1_start = current_frame
        for _ in range(samples_per_activity):
            frames.append(self.generate_activity_frame())
            current_frame += 1
        ground_truth.append(TimeRange(activity1_start, current_frame))

        for _ in range(static_samples_between):
            frames.append(self.generate_static_frame())
            current_frame += 1

        activity2_start = current_frame
        for _ in range(samples_per_activity):
            frames.append(self.generate_activity_frame())
            current_frame += 1
        ground_truth.append(TimeRange(activity2_start, current_frame))

        for _ in range(static_samples_after):
            frames.append(self.generate_static_frame())
            current_frame += 1

        return frames, ground_truth


class EdgeSegmenter:
    """Portable version of the ESP32 variance-based segmenter."""

    def __init__(
        self,
        window_size: int = 50,
        num_subcarriers: int = 64,
        variance_thresh_start: float = 2.60,
        variance_thresh_stop: float = 2.50,
        hysteresis_samples: int = 10,
        max_segment_length: int = 100,
    ):
        self.window_size = window_size
        self.num_subcarriers = num_subcarriers
        self.variance_thresh_start = variance_thresh_start
        self.variance_thresh_stop = variance_thresh_stop
        self.hysteresis_samples = hysteresis_samples
        self.max_segment_length = max_segment_length

        self.phase_history = [
            [0.0 for _ in range(num_subcarriers)] for _ in range(window_size)
        ]
        self.mean_vector = [0.0 for _ in range(num_subcarriers)]
        self.m2_vector = [0.0 for _ in range(num_subcarriers)]
        self.variance_vector = [0.0 for _ in range(num_subcarriers)]

        self.current_head = 0
        self.samples_collected = 0
        self.buffer_primed = False
        self.current_state = 0  # 0=STATIC, 1=DYNAMIC
        self.hysteresis_counter = 0
        self.frame_counter = 0
        self.detected_segments: List[TimeRange] = []
        self.current_segment_start = 0

    def process_csi_frame(self, iq_frame: Sequence[Tuple[float, float]]) -> None:
        if len(iq_frame) != self.num_subcarriers:
            return

        for k, (real, imag) in enumerate(iq_frame):
            phase = math.atan2(imag, real)
            self._process_welford_variance(k, phase)

        self.current_head = (self.current_head + 1) % self.window_size
        if self.samples_collected < self.window_size:
            self.samples_collected += 1
            if self.samples_collected >= self.window_size:
                self.buffer_primed = True

        env_variance = fmean(self.variance_vector)
        if self.buffer_primed:
            self._update_segmentation_state(env_variance)

        self.frame_counter += 1

    def _process_welford_variance(self, subcarrier: int, new_phase: float) -> None:
        old_phase = self.phase_history[self.current_head][subcarrier]
        self.phase_history[self.current_head][subcarrier] = new_phase

        if not self.buffer_primed:
            count = self.samples_collected + 1
            delta = new_phase - self.mean_vector[subcarrier]
            self.mean_vector[subcarrier] += delta / count
            delta2 = new_phase - self.mean_vector[subcarrier]
            self.m2_vector[subcarrier] += delta * delta2
            self.variance_vector[subcarrier] = (
                self.m2_vector[subcarrier] / (count - 1) if count > 1 else 0.0
            )
            return

        old_mean = self.mean_vector[subcarrier]
        new_mean = old_mean + (new_phase - old_phase) / self.window_size
        self.mean_vector[subcarrier] = new_mean
        self.m2_vector[subcarrier] += (new_phase - old_phase) * (
            new_phase - new_mean + old_phase - old_mean
        )
        self.variance_vector[subcarrier] = self.m2_vector[subcarrier] / (
            self.window_size - 1
        )

    def _update_segmentation_state(self, env_variance: float) -> None:
        if self.current_state == 0:
            if env_variance > self.variance_thresh_start:
                self.current_state = 1
                self.hysteresis_counter = 0
                self.current_segment_start = self.frame_counter
            return

        segment_duration = self.frame_counter - self.current_segment_start + 1
        if segment_duration >= self.max_segment_length:
            self._close_current_segment(self.frame_counter + 1)
            return

        if env_variance < self.variance_thresh_stop:
            self.hysteresis_counter += 1
            if self.hysteresis_counter >= self.hysteresis_samples:
                self._close_current_segment(self.frame_counter + 1)
        else:
            self.hysteresis_counter = 0

    def _close_current_segment(self, end_frame: int) -> None:
        self.current_state = 0
        self.hysteresis_counter = 0
        self.detected_segments.append(TimeRange(self.current_segment_start, end_frame))

    def get_detected_segments(self) -> List[TimeRange]:
        return self.detected_segments


class RangeBasedEvaluator:
    """Point-union implementation of range precision/recall."""

    @staticmethod
    def compute_metrics(
        ground_truth: Sequence[TimeRange],
        predictions: Sequence[TimeRange],
    ) -> RangeMetrics:
        truth_points = RangeBasedEvaluator._points(ground_truth)
        pred_points = RangeBasedEvaluator._points(predictions)

        if not truth_points:
            fp = len(pred_points)
            return RangeMetrics(
                precision=1.0 if not pred_points else 0.0,
                recall=1.0,
                f1_score=1.0 if not pred_points else 0.0,
                tp_range=0,
                fp_range=fp,
                fn_range=0,
            )

        if not pred_points:
            return RangeMetrics(
                precision=1.0,
                recall=0.0,
                f1_score=0.0,
                tp_range=0,
                fp_range=0,
                fn_range=len(truth_points),
            )

        tp = len(truth_points & pred_points)
        fp = len(pred_points - truth_points)
        fn = len(truth_points - pred_points)

        precision = tp / len(pred_points) if pred_points else 0.0
        recall = tp / len(truth_points) if truth_points else 0.0
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall > 0.0
            else 0.0
        )

        return RangeMetrics(precision, recall, f1, tp, fp, fn)

    @staticmethod
    def _points(ranges: Iterable[TimeRange]) -> set[int]:
        points: set[int] = set()
        for time_range in ranges:
            points.update(range(time_range.start_idx, time_range.end_idx))
        return points


SCENARIOS = [
    {
        "scenario_id": 1,
        "name": "Flujo normal",
        "samples_per_activity": 100,
        "static_samples_before": 50,
        "static_samples_between": 75,
        "static_samples_after": 50,
    },
    {
        "scenario_id": 2,
        "name": "Actividades cortas con ruido abundante",
        "samples_per_activity": 60,
        "static_samples_before": 100,
        "static_samples_between": 150,
        "static_samples_after": 100,
    },
    {
        "scenario_id": 3,
        "name": "Actividades sostenidas",
        "samples_per_activity": 200,
        "static_samples_before": 30,
        "static_samples_between": 40,
        "static_samples_after": 30,
    },
]


def run_test_scenario(
    scenario_id: int,
    samples_per_activity: int,
    static_samples_before: int,
    static_samples_between: int,
    static_samples_after: int,
    *,
    variance_thresh_start: float = 2.60,
    variance_thresh_stop: float = 2.50,
    hysteresis_samples: int = 10,
    max_segment_length: int = 100,
) -> dict:
    generator = SyntheticCSIGenerator(seed=42 + scenario_id)
    segmenter = EdgeSegmenter(
        window_size=50,
        num_subcarriers=64,
        variance_thresh_start=variance_thresh_start,
        variance_thresh_stop=variance_thresh_stop,
        hysteresis_samples=hysteresis_samples,
        max_segment_length=max_segment_length,
    )

    frames, ground_truth = generator.generate_continuous_stream(
        samples_per_activity,
        static_samples_before,
        static_samples_between,
        static_samples_after,
    )

    for frame in frames:
        segmenter.process_csi_frame(frame)

    detected = segmenter.get_detected_segments()
    metrics = RangeBasedEvaluator.compute_metrics(ground_truth, detected)

    return {
        "scenario_id": scenario_id,
        "total_frames": len(frames),
        "expected_activities": len(ground_truth),
        "detected_activities": len(detected),
        "ground_truth": [range_to_dict(r) for r in ground_truth],
        "detected_ranges": [range_to_dict(r) for r in detected],
        "metrics": metrics_to_dict(metrics),
    }


def range_to_dict(time_range: TimeRange) -> dict:
    return {
        "start_idx": time_range.start_idx,
        "end_idx": time_range.end_idx,
        "duration": time_range.duration(),
    }


def metrics_to_dict(metrics: RangeMetrics) -> dict:
    return {
        "precision": round(metrics.precision, 4),
        "recall": round(metrics.recall, 4),
        "f1_score": round(metrics.f1_score, 4),
        "tp_range": metrics.tp_range,
        "fp_range": metrics.fp_range,
        "fn_range": metrics.fn_range,
    }


def run_all_scenarios() -> List[dict]:
    results: List[dict] = []
    for scenario in SCENARIOS:
        results.append(
            run_test_scenario(
                scenario["scenario_id"],
                scenario["samples_per_activity"],
                scenario["static_samples_before"],
                scenario["static_samples_between"],
                scenario["static_samples_after"],
            )
        )
    return results


def print_report(results: Sequence[dict]) -> None:
    print("\n" + "=" * 80)
    print("  REPORTE DE EVALUACION - SEMANA 3: M1 SEGMENTADOR VALIDADO")
    print("  Evaluacion con flujos CSI sinteticos continuos")
    print("=" * 80)
    print(f"  Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80 + "\n")

    for result in results:
        metrics = result["metrics"]
        print(f"[Escenario {result['scenario_id']}]")
        print(f"  Frames procesados:      {result['total_frames']}")
        print(f"  Actividades esperadas:  {result['expected_activities']}")
        print(f"  Actividades detectadas: {result['detected_activities']}")
        print("  Metricas:")
        print(f"    Precision: {metrics['precision']:.4f}")
        print(f"    Recall:    {metrics['recall']:.4f}")
        print(f"    F1-Score:  {metrics['f1_score']:.4f}")
        print(f"    TP/FP/FN:  {metrics['tp_range']}/{metrics['fp_range']}/{metrics['fn_range']}")
        print()

    summary = summarize(results)
    print("RESUMEN GLOBAL:")
    print("-" * 40)
    print(f"  Precision promedio: {summary['avg_precision']:.4f}")
    print(f"  Recall promedio:    {summary['avg_recall']:.4f}")
    print(f"  F1-Score promedio:  {summary['avg_f1_score']:.4f}")
    print(f"  Umbral validacion:  {summary['validation_threshold']:.4f}")
    print(f"  Estado:             {summary['overall_status']}")
    print("=" * 80 + "\n")


def summarize(results: Sequence[dict]) -> dict:
    avg_precision = fmean(r["metrics"]["precision"] for r in results)
    avg_recall = fmean(r["metrics"]["recall"] for r in results)
    avg_f1 = fmean(r["metrics"]["f1_score"] for r in results)
    threshold = 0.70
    passed = sum(1 for r in results if r["metrics"]["f1_score"] >= threshold)
    return {
        "avg_f1_score": round(avg_f1, 4),
        "avg_precision": round(avg_precision, 4),
        "avg_recall": round(avg_recall, 4),
        "validation_threshold": threshold,
        "threshold_met": avg_f1 >= threshold,
        "scenarios_passed": passed,
        "scenarios_total": len(results),
        "overall_status": "VALIDACION EXITOSA" if avg_f1 >= threshold else "AJUSTE REQUERIDO",
    }


def main() -> None:
    print("[SEGMENTER_EVAL] Iniciando evaluacion M1 Semana 3")
    results = run_all_scenarios()
    print_report(results)

    report_data = {
        "timestamp": datetime.now().isoformat(),
        "milestone": "M1_SegmentadorValidado_Semana3",
        "project": "CSI Segmentation - Continuous Human Activity Recognition",
        "evaluation_method": "Flujos CSI sinteticos continuos",
        "metrics_reference": "Range-based Precision/Recall por union de rangos",
        "segmenter_parameters": {
            "window_size": 50,
            "num_subcarriers": 64,
            "variance_threshold_start": 2.60,
            "variance_threshold_stop": 2.50,
            "hysteresis_samples": 10,
            "max_segment_length": 100,
        },
        "scenarios": results,
        "summary": summarize(results),
    }

    with open("segmentation_evaluation_results.json", "w", encoding="utf-8") as f:
        json.dump(report_data, f, indent=2)

    print("[SEGMENTER_EVAL] Resultados guardados en segmentation_evaluation_results.json")


if __name__ == "__main__":
    main()
