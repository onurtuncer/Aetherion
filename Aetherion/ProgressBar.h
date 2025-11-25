// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <print>
#include <string>
#include <algorithm>

inline void ProgressBar(double progress, int width = 40)
{
    progress = std::clamp(progress, 0.0, 1.0);
    int filled = static_cast<int>(progress * width);

    std::string bar = std::string(filled, '#') +
        std::string(width - filled, '.');

    std::print("\r[{}] {:3.0f}%", bar, progress * 100.0);
}