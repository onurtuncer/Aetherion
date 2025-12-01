// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <fmu4cpp/fmu_base.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

using namespace fmu4cpp;

struct Sample {
    double t;
    double y;
};

class TabulatedSignalFMU : public fmu_base {
public:
    FMU4CPP_CTOR(TabulatedSignalFMU)
    {
        // Output: interpolated value
        register_real("y", &y_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED);

        // Parameter: path to the text file (string parameter is FMI-compliant;
        // if your target tool doesn’t like strings, replace with an integer enum).
        register_string("file_path", &file_path_)
            .setCausality(causality_t::PARAMETER)
            .setVariability(variability_t::FIXED);

        // Optional: time offset if your table doesn’t start at 0
        register_real("t_offset", &t_offset_)
            .setCausality(causality_t::PARAMETER)
            .setVariability(variability_t::FIXED);

        reset();
    }

    // Called when the simulator resets the FMU
    void reset() override
    {
        current_time_ = 0.0;
        y_ = 0.0;
        last_index_ = 0u;

        // Provide sensible defaults if user forgets to set parameters
        if (file_path_.empty()) {
            // You can hard-code a default here or throw
            // For now just leave table empty and output 0
            samples_.clear();
            return;
        }

        load_table(file_path_);
        if (!samples_.empty()) {
            y_ = samples_.front().y;
        }
    }

    // Called each communication step; dt is the step size
    bool do_step(double dt) override
    {
        if (samples_.empty()) {
            // Nothing loaded: keep output as is
            return true;
        }

        current_time_ += dt;
        const double t_query = current_time_ + t_offset_;

        y_ = interpolate(t_query);
        return true;
    }

private:
    // ---- Data ----
    double y_{ 0.0 };                 // output
    std::string file_path_{};       // parameter
    double t_offset_{ 0.0 };          // parameter
    double current_time_{ 0.0 };      // internal model time
    std::vector<Sample> samples_;   // tabulated data
    std::size_t last_index_{ 0 };     // search hint

    // ---- Helpers ----

    void load_table(const std::string& path)
    {
        std::ifstream in(path);
        if (!in.is_open()) {
            throw std::runtime_error("TabulatedSignalFMU: cannot open file: " + path);
        }

        std::vector<Sample> tmp;
        tmp.reserve(256);

        std::string line;
        while (std::getline(in, line)) {
            // Skip comments / empty lines
            if (line.empty() || line[0] == '#')
                continue;

            std::istringstream iss(line);
            double t, y;
            if (!(iss >> t >> y)) {
                // Malformed line; you can choose to skip or throw
                throw std::runtime_error("TabulatedSignalFMU: malformed line in file: " + path);
            }
            tmp.push_back({ t, y });
        }

        if (tmp.size() < 2) {
            throw std::runtime_error("TabulatedSignalFMU: need at least two samples for interpolation");
        }

        // Ensure sorted by time
        std::sort(tmp.begin(), tmp.end(),
            [](const Sample& a, const Sample& b) { return a.t < b.t; });

        samples_ = std::move(tmp);
        last_index_ = 0;
    }

    double interpolate(double t) const
    {
        // Clamp outside the table range
        if (t <= samples_.front().t)
            return samples_.front().y;
        if (t >= samples_.back().t)
            return samples_.back().y;

        // Use last_index_ as a search hint (table is monotone in time, t moves forward)
        std::size_t i = last_index_;
        if (i >= samples_.size() - 1)
            i = samples_.size() - 2;

        // Move forward while t is beyond the next sample
        while (i + 1 < samples_.size() && t > samples_[i + 1].t) {
            ++i;
        }
        // Optionally move backward if user changes t_offset (rare)
        while (i > 0 && t < samples_[i].t) {
            --i;
        }

        // Find neighbors
        const Sample& s0 = samples_[i];
        const Sample& s1 = samples_[i + 1];

        const double alpha = (t - s0.t) / (s1.t - s0.t);
        return (1.0 - alpha) * s0.y + alpha * s1.y;
    }
};

// Model metadata required by fmu4cpp
model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName = "TabulatedSignal";
    info.description = "Outputs a linearly interpolated value from a time–value table";
    return info;
}

// Instantiate macro
FMU4CPP_INSTANTIATE(TabulatedSignalFMU);
