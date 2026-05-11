/**
 * @file synthetic_csi_generator.hpp
 * @brief Synthetic CSI data generator for segmenter evaluation.
 */

#pragma once

#include <stdint.h>

#include <cmath>
#include <random>
#include <vector>

typedef struct {
    float real;
    float imag;
} ComplexCSI;

typedef struct {
    std::vector<ComplexCSI> iq_data;
} CSIFrame;

class SyntheticCSIGenerator {
private:
    static constexpr float kPi = 3.14159265358979323846f;

    std::mt19937 rng;
    std::normal_distribution<float> noise_dist;
    uint32_t frame_counter;

public:
    struct TimeRange {
        uint32_t start_idx;
        uint32_t end_idx;
    };

    struct SyntheticStream {
        std::vector<CSIFrame> frames;
        std::vector<TimeRange> ground_truth;
    };

    SyntheticCSIGenerator(uint32_t seed = 42)
        : rng(seed), noise_dist(0.0f, 0.1f), frame_counter(0) {}

    CSIFrame generate_static_frame(uint32_t num_subcarriers) {
        CSIFrame frame;
        frame.iq_data.resize(num_subcarriers);

        for (uint32_t k = 0; k < num_subcarriers; ++k) {
            const float noise = noise_dist(rng) * 0.5f;
            frame.iq_data[k].real = noise;
            frame.iq_data[k].imag = noise;
        }

        frame_counter++;
        return frame;
    }

    CSIFrame generate_activity_frame(uint32_t num_subcarriers) {
        CSIFrame frame;
        frame.iq_data.resize(num_subcarriers);

        const float activity_amplitude =
            2.0f + 1.0f * std::sin(2.0f * kPi * frame_counter / 50.0f);

        for (uint32_t k = 0; k < num_subcarriers; ++k) {
            const float phase =
                2.0f * kPi * k / (float)num_subcarriers +
                activity_amplitude * std::sin(2.0f * kPi * frame_counter / 20.0f);

            frame.iq_data[k].real =
                activity_amplitude * std::cos(phase) + noise_dist(rng) * 2.5f;
            frame.iq_data[k].imag =
                activity_amplitude * std::sin(phase) + noise_dist(rng) * 2.5f;
        }

        frame_counter++;
        return frame;
    }

    SyntheticStream generate_continuous_stream(
        uint32_t num_subcarriers,
        uint32_t samples_per_activity,
        uint32_t static_samples_before,
        uint32_t static_samples_between,
        uint32_t static_samples_after
    ) {
        SyntheticStream stream;
        uint32_t current_frame = 0;

        for (uint32_t i = 0; i < static_samples_before; ++i) {
            stream.frames.push_back(generate_static_frame(num_subcarriers));
            current_frame++;
        }

        const uint32_t activity1_start = current_frame;
        for (uint32_t i = 0; i < samples_per_activity; ++i) {
            stream.frames.push_back(generate_activity_frame(num_subcarriers));
            current_frame++;
        }
        stream.ground_truth.push_back({activity1_start, current_frame});

        for (uint32_t i = 0; i < static_samples_between; ++i) {
            stream.frames.push_back(generate_static_frame(num_subcarriers));
            current_frame++;
        }

        const uint32_t activity2_start = current_frame;
        for (uint32_t i = 0; i < samples_per_activity; ++i) {
            stream.frames.push_back(generate_activity_frame(num_subcarriers));
            current_frame++;
        }
        stream.ground_truth.push_back({activity2_start, current_frame});

        for (uint32_t i = 0; i < static_samples_after; ++i) {
            stream.frames.push_back(generate_static_frame(num_subcarriers));
            current_frame++;
        }

        return stream;
    }

    void reset() {
        frame_counter = 0;
    }
};
