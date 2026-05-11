/**
 * @file edge_segmenter.hpp
 * @brief Portable adaptive segmenter based on phase-variance dynamics.
 */

#pragma once

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

typedef enum {
    SEGMENTER_STATE_STATIC = 0,
    SEGMENTER_STATE_DYNAMIC = 1
} SegmenterState;

typedef struct {
    uint32_t start_frame;
    uint32_t end_frame;
    uint32_t duration_frames;
} DetectedSegment;

class EdgeSegmenter {
private:
    uint32_t window_size;
    uint32_t num_subcarriers;
    float variance_threshold_start;
    float variance_threshold_stop;
    uint32_t hysteresis_samples;
    uint32_t max_segment_length;

    std::vector<std::vector<float>> phase_history;
    std::vector<float> mean_vector;
    std::vector<float> m2_vector;
    std::vector<float> variance_vector;

    uint32_t current_head;
    uint32_t samples_collected;
    bool buffer_primed;

    SegmenterState current_state;
    uint32_t hysteresis_counter;
    uint32_t frame_counter;

    std::vector<DetectedSegment> detected_segments;
    uint32_t current_segment_start;

public:
    EdgeSegmenter(
        uint32_t window_size_in = 50,
        uint32_t num_subcarriers_in = 64,
        float thresh_start = 2.60f,
        float thresh_stop = 2.50f,
        uint32_t hysteresis = 10,
        uint32_t max_segment_length_in = 100
    ) :
        window_size(window_size_in),
        num_subcarriers(num_subcarriers_in),
        variance_threshold_start(thresh_start),
        variance_threshold_stop(thresh_stop),
        hysteresis_samples(hysteresis),
        max_segment_length(max_segment_length_in),
        current_head(0),
        samples_collected(0),
        buffer_primed(false),
        current_state(SEGMENTER_STATE_STATIC),
        hysteresis_counter(0),
        frame_counter(0),
        current_segment_start(0)
    {
        phase_history.resize(window_size, std::vector<float>(num_subcarriers, 0.0f));
        mean_vector.resize(num_subcarriers, 0.0f);
        m2_vector.resize(num_subcarriers, 0.0f);
        variance_vector.resize(num_subcarriers, 0.0f);
    }

    void process_csi_frame(const std::vector<std::pair<float, float>>& iq_pairs) {
        if (iq_pairs.size() != num_subcarriers) {
            return;
        }

        for (uint32_t k = 0; k < num_subcarriers; ++k) {
            const float phase = std::atan2(iq_pairs[k].second, iq_pairs[k].first);
            process_welford_variance(k, phase);
        }

        current_head = (current_head + 1U) % window_size;
        if (samples_collected < window_size) {
            samples_collected++;
            if (samples_collected >= window_size) {
                buffer_primed = true;
            }
        }

        const float env_variance = compute_environment_variance();
        if (buffer_primed) {
            update_segmentation_state(env_variance);
        }

        frame_counter++;
    }

    std::vector<DetectedSegment> get_detected_segments() const {
        return detected_segments;
    }

    void reset() {
        current_head = 0;
        samples_collected = 0;
        buffer_primed = false;
        current_state = SEGMENTER_STATE_STATIC;
        hysteresis_counter = 0;
        frame_counter = 0;
        current_segment_start = 0;
        detected_segments.clear();

        std::fill(mean_vector.begin(), mean_vector.end(), 0.0f);
        std::fill(m2_vector.begin(), m2_vector.end(), 0.0f);
        std::fill(variance_vector.begin(), variance_vector.end(), 0.0f);

        for (auto& row : phase_history) {
            std::fill(row.begin(), row.end(), 0.0f);
        }
    }

    SegmenterState get_current_state() const {
        return current_state;
    }

    uint32_t get_frame_count() const {
        return frame_counter;
    }

private:
    void process_welford_variance(uint32_t subcarrier, float new_phase) {
        const float old_phase = phase_history[current_head][subcarrier];
        phase_history[current_head][subcarrier] = new_phase;

        if (!buffer_primed) {
            const float count = (float)(samples_collected + 1U);
            const float delta = new_phase - mean_vector[subcarrier];
            mean_vector[subcarrier] += delta / count;
            const float delta2 = new_phase - mean_vector[subcarrier];
            m2_vector[subcarrier] += delta * delta2;
            variance_vector[subcarrier] = (count > 1.0f)
                ? (m2_vector[subcarrier] / (count - 1.0f))
                : 0.0f;
        } else {
            const float old_mean = mean_vector[subcarrier];
            const float delta_mean = (new_phase - old_phase) / (float)window_size;
            const float new_mean = old_mean + delta_mean;

            mean_vector[subcarrier] = new_mean;
            m2_vector[subcarrier] += (new_phase - old_phase) *
                                     (new_phase - new_mean + old_phase - old_mean);
            variance_vector[subcarrier] = m2_vector[subcarrier] /
                                          (float)(window_size - 1U);
        }
    }

    float compute_environment_variance() const {
        float acc = 0.0f;
        for (uint32_t k = 0; k < num_subcarriers; ++k) {
            acc += variance_vector[k];
        }
        return acc / (float)num_subcarriers;
    }

    void update_segmentation_state(float env_variance) {
        if (current_state == SEGMENTER_STATE_STATIC) {
            if (env_variance > variance_threshold_start) {
                current_state = SEGMENTER_STATE_DYNAMIC;
                hysteresis_counter = 0;
                current_segment_start = frame_counter;
            }
        } else {
            const uint32_t segment_duration = frame_counter - current_segment_start + 1U;
            if (segment_duration >= max_segment_length) {
                close_current_segment(frame_counter + 1U);
            } else if (env_variance < variance_threshold_stop) {
                hysteresis_counter++;
                if (hysteresis_counter >= hysteresis_samples) {
                    close_current_segment(frame_counter + 1U);
                }
            } else {
                hysteresis_counter = 0;
            }
        }
    }

    void close_current_segment(uint32_t end_frame) {
        current_state = SEGMENTER_STATE_STATIC;
        hysteresis_counter = 0;

        DetectedSegment seg;
        seg.start_frame = current_segment_start;
        seg.end_frame = end_frame;
        seg.duration_frames = seg.end_frame - seg.start_frame;
        detected_segments.push_back(seg);
    }
};
