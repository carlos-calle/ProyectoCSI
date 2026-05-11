/**
 * @file segmentation_metrics.hpp
 * @brief Range-based Precision/Recall helpers for continuous CSI segmentation.
 */

#pragma once

#include <stdint.h>
#include <algorithm>
#include <vector>

typedef struct {
    uint32_t start_idx;
    uint32_t end_idx;  // Exclusive.
} TimeRange;

typedef struct {
    float precision;
    float recall;
    float f1_score;
    uint32_t tp_range;
    uint32_t fp_range;
    uint32_t fn_range;
} RangeMetrics;

class RangeBasedEvaluator {
public:
    static RangeMetrics compute_metrics(
        const std::vector<TimeRange>& ground_truth,
        const std::vector<TimeRange>& predictions
    ) {
        RangeMetrics metrics = {0.0f, 0.0f, 0.0f, 0, 0, 0};

        if (ground_truth.empty()) {
            metrics.fp_range = count_total_points(predictions);
            metrics.precision = predictions.empty() ? 1.0f : 0.0f;
            metrics.recall = 1.0f;
            metrics.f1_score = predictions.empty() ? 1.0f : 0.0f;
            return metrics;
        }

        if (predictions.empty()) {
            metrics.fn_range = count_total_points(ground_truth);
            metrics.precision = 1.0f;
            metrics.recall = 0.0f;
            metrics.f1_score = 0.0f;
            return metrics;
        }

        const uint32_t max_end = std::max(max_end_idx(ground_truth),
                                          max_end_idx(predictions));
        std::vector<bool> truth_points(max_end, false);
        std::vector<bool> pred_points(max_end, false);
        mark_points(ground_truth, truth_points);
        mark_points(predictions, pred_points);

        uint32_t pred_total = 0;
        uint32_t truth_total = 0;
        for (uint32_t i = 0; i < max_end; ++i) {
            if (pred_points[i]) pred_total++;
            if (truth_points[i]) truth_total++;
            if (pred_points[i] && truth_points[i]) metrics.tp_range++;
            if (pred_points[i] && !truth_points[i]) metrics.fp_range++;
            if (!pred_points[i] && truth_points[i]) metrics.fn_range++;
        }

        metrics.precision = (pred_total > 0)
            ? ((float)metrics.tp_range / (float)pred_total)
            : 0.0f;
        metrics.recall = (truth_total > 0)
            ? ((float)metrics.tp_range / (float)truth_total)
            : 0.0f;

        const float sum = metrics.precision + metrics.recall;
        metrics.f1_score = (sum > 0.0f)
            ? (2.0f * metrics.precision * metrics.recall / sum)
            : 0.0f;

        return metrics;
    }

private:
    static uint32_t count_total_points(const std::vector<TimeRange>& ranges) {
        uint32_t total = 0;
        for (const auto& range : ranges) {
            total += (range.end_idx > range.start_idx)
                ? (range.end_idx - range.start_idx)
                : 0U;
        }
        return total;
    }

    static uint32_t max_end_idx(const std::vector<TimeRange>& ranges) {
        uint32_t max_end = 0;
        for (const auto& range : ranges) {
            if (range.end_idx > max_end) max_end = range.end_idx;
        }
        return max_end;
    }

    static void mark_points(const std::vector<TimeRange>& ranges,
                            std::vector<bool>& points) {
        for (const auto& range : ranges) {
            const uint32_t end = std::min<uint32_t>(range.end_idx, points.size());
            for (uint32_t i = range.start_idx; i < end; ++i) {
                points[i] = true;
            }
        }
    }
};
