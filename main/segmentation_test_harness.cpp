/**
 * @file segmentation_test_harness.cpp
 * @brief Test harness for validating the CSI edge segmenter with synthetic flows.
 *
 * Semana 3 / M1:
 * - Generate continuous synthetic CSI streams.
 * - Run the portable adaptive segmenter.
 * - Compute range-based precision, recall, and F1 metrics.
 * - Print a compact validation report.
 */

#include <stdint.h>
#include <stdio.h>

#include <cmath>
#include <utility>
#include <vector>

#include "edge_segmenter.hpp"
#include "segmentation_metrics.hpp"
#include "synthetic_csi_generator.hpp"

const char* TAG_TEST = "SEGMENTER_EVAL";

struct TestScenarioResult {
    const char* name;
    uint32_t num_activities;
    RangeMetrics metrics;
    uint32_t total_frames;
};

std::vector<std::pair<float, float>> convert_frame_to_iq(const CSIFrame& frame) {
    std::vector<std::pair<float, float>> iq_pairs;
    iq_pairs.reserve(frame.iq_data.size());

    for (const auto& csi : frame.iq_data) {
        iq_pairs.push_back({csi.real, csi.imag});
    }

    return iq_pairs;
}

std::vector<TimeRange> convert_segments_to_ranges(
    const std::vector<DetectedSegment>& segments
) {
    std::vector<TimeRange> ranges;
    ranges.reserve(segments.size());

    for (const auto& seg : segments) {
        ranges.push_back({seg.start_frame, seg.end_frame});
    }

    return ranges;
}

TestScenarioResult run_test_scenario(
    uint32_t scenario_id,
    uint32_t samples_per_activity,
    uint32_t static_samples_before,
    uint32_t static_samples_between,
    uint32_t static_samples_after
) {
    TestScenarioResult result = {"Continuous synthetic CSI flow", 0, {0, 0, 0, 0, 0, 0}, 0};

    SyntheticCSIGenerator generator(42 + scenario_id);
    EdgeSegmenter segmenter(50, 64, 2.60f, 2.50f, 10, 100);

    auto stream = generator.generate_continuous_stream(
        64,
        samples_per_activity,
        static_samples_before,
        static_samples_between,
        static_samples_after
    );

    for (const auto& frame : stream.frames) {
        auto iq_pairs = convert_frame_to_iq(frame);
        segmenter.process_csi_frame(iq_pairs);
    }

    const auto detected_segments = segmenter.get_detected_segments();
    const auto detected_ranges = convert_segments_to_ranges(detected_segments);

    std::vector<TimeRange> truth_ranges;
    truth_ranges.reserve(stream.ground_truth.size());
    for (const auto& truth : stream.ground_truth) {
        truth_ranges.push_back({truth.start_idx, truth.end_idx});
    }

    result.metrics = RangeBasedEvaluator::compute_metrics(truth_ranges, detected_ranges);
    result.total_frames = segmenter.get_frame_count();
    result.num_activities = stream.ground_truth.size();

    return result;
}

void print_test_report(const std::vector<TestScenarioResult>& results) {
    printf("\n");
    printf("================================================================================\n");
    printf("  SEMANA 3 / M1 - EDGE SEGMENTER VALIDATION REPORT\n");
    printf("  Continuous synthetic CSI flows\n");
    printf("================================================================================\n\n");

    printf("SCENARIOS:\n");
    printf("----------\n");

    int scenario_num = 1;
    for (const auto& result : results) {
        printf("\n[Scenario %d] %s\n", scenario_num, result.name);
        printf("  Expected activities: %u\n", result.num_activities);
        printf("  Processed frames:    %u\n", result.total_frames);
        printf("\n  Range-based metrics:\n");
        printf("    Precision: %.4f\n", result.metrics.precision);
        printf("    Recall:    %.4f\n", result.metrics.recall);
        printf("    F1-Score:  %.4f\n", result.metrics.f1_score);
        printf("    TP points: %u\n", result.metrics.tp_range);
        printf("    FP points: %u\n", result.metrics.fp_range);
        printf("    FN points: %u\n", result.metrics.fn_range);

        if (result.metrics.f1_score >= 0.85f) {
            printf("  Result: STRONG PASS (F1-Score >= 0.85)\n");
        } else if (result.metrics.f1_score >= 0.70f) {
            printf("  Result: PASS (F1-Score >= 0.70)\n");
        } else {
            printf("  Result: DOES NOT MEET TARGET (F1-Score < 0.70)\n");
        }

        scenario_num++;
    }

    printf("\n\nGLOBAL SUMMARY:\n");
    printf("---------------\n");

    if (!results.empty()) {
        float avg_precision = 0.0f;
        float avg_recall = 0.0f;
        float avg_f1 = 0.0f;

        for (const auto& result : results) {
            avg_precision += result.metrics.precision;
            avg_recall += result.metrics.recall;
            avg_f1 += result.metrics.f1_score;
        }

        avg_precision /= results.size();
        avg_recall /= results.size();
        avg_f1 /= results.size();

        printf("  Average precision: %.4f\n", avg_precision);
        printf("  Average recall:    %.4f\n", avg_recall);
        printf("  Average F1-Score:  %.4f\n", avg_f1);

        if (avg_f1 >= 0.70f) {
            printf("\n  VALIDATION PASSED: M1 segmenter is ready for Semana 3.\n");
        } else {
            printf("\n  REVIEW NEEDED: tune thresholds and segmentation parameters.\n");
        }
    }

    printf("\n================================================================================\n");
    printf("  Method: point-union range precision/recall over expected activity ranges\n");
    printf("================================================================================\n\n");
}

extern "C" void app_main(void) {
    printf("[%s] Starting Semana 3 / M1 segmenter validation\n\n", TAG_TEST);

    std::vector<TestScenarioResult> test_results;

    printf("[Scenario 1] Normal activities with intermediate rest...\n");
    test_results.push_back(run_test_scenario(1, 100, 50, 75, 50));
    printf("  Completed\n\n");

    printf("[Scenario 2] Short activities with extended static periods...\n");
    test_results.push_back(run_test_scenario(2, 60, 100, 150, 100));
    printf("  Completed\n\n");

    printf("[Scenario 3] Long sustained activities...\n");
    test_results.push_back(run_test_scenario(3, 200, 30, 40, 30));
    printf("  Completed\n\n");

    print_test_report(test_results);

    printf("[%s] Validation completed.\n", TAG_TEST);
    printf("[%s] Hardware integration source: csi_baremetal_edge_segmentation.cpp\n\n", TAG_TEST);
}
