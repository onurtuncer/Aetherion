// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::ODE::RKMK::Lie {

	// Instantiate the common scalars
	template struct SE3<double>;

#if __has_include(<cppad/cppad.hpp>)
	template struct SE3<CppAD::AD<double>>;
#endif

} // namespace Aetherion::ODE::RKMK::Lie
