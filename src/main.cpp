// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <iostream>
#include <print>

int main() {

	try {

	}
	catch (const std::exception& e) {
		std::print("Exception: {}\n", e.what());
		return 1;
	}


	return EXIT_SUCCESS;
}
