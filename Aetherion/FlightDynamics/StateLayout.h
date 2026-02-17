// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics
{
	struct StateLayout {
		static constexpr int IDX_P = 0;      // 0..2
		static constexpr int IDX_Q = 3;      // 3..6
		static constexpr int IDX_W = 7;      // 7..9
		static constexpr int IDX_V = 10;     // 10..12
		static constexpr int IDX_M = 13;     // 13
		static constexpr int N = 14;
	};


} // namespace Aetherion::FlightDynamics