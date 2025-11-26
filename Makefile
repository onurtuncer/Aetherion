# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

#TODO this makefile is work in progress right now

# Default target
all: help

# Base directory for your project
ROOT_DIR := $(shell pwd)
BUILD_DIR := $(ROOT_DIR)/build

# Targets
.PHONY: all gcc-release  gcc-debug gcc-test clang-debug gcc-asan gcc-msan gcc-tsan gcc-ubsan

all: gcc-release gcc-asan gcc-msan gcc-tsan gcc-ubsan

# Default gcc-release build
gcc-release:
	cmake --preset=gcc-release
	cmake --build --preset=gcc-release

# Default gcc-debug build
gcc-debug:
	cmake --preset=gcc-debug
	cmake --build --preset=gcc-debug

# Default gcc-test build
gcc-test:
	cmake --preset=gcc-test
	cmake --build --preset=gcc-test

# Default clang-debug build
clang-debug:
	cmake --preset=clang-debug
	cmake --build --preset=clang-debug

# Default gcc build with address sanitizer
gcc-asan:
	cmake --preset=gcc-asan
	cmake --build --preset=gcc-asan

# Default gcc build with memory sanitizer
gcc-msan:
	cmake --preset=gcc-msan
	cmake --build --preset=gcc-msan

# Default gcc build with thread sanitizer
gcc-tsan:
	cmake --preset=gcc-tsan
	cmake --build --preset=gcc-tsan

# Dadult gcc build with undefined behaviour sanitizer
gcc-ubsan:
	cmake --preset=gcc-ubsan 
	cmake --build --preset=gcc-ubsan 

# Style
# --------------------------------------------------------------------
.PHONY: check-format format check-newlines

check-format:
	$(call colorecho,'Checking formatting with astyle')
	@"$(ROOT_DIR)"/tools/astyle/check_code_style_all.sh
	@cd "$(ROOT_DIR)" && git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@"$(ROOT_DIR)"/tools/astyle/check_code_style_all.sh --fix

check-newlines:
	$(call colorecho,'Checking for missing or duplicate newlines at the end of files')
	@"$(ROOT_DIR)"/tools/astyle/check_newlines.sh

# Testing
# --------------------------------------------------------------------
.PHONY: tests 

tests: 
	cmake --preset=gcc-debug
	cmake --build --preset=gcc-debug
	ctest --preset=gcc-test

# Paths
COV_BUILD_DIR := build/gcc-debug
COV_INFO := $(COV_BUILD_DIR)/coverage.info
COV_DIR := $(COV_BUILD_DIR)/coverage

# Exclude patterns
LCOV_EXCLUDES := '*/test/*' '*/_deps/*'


.PHONY: coverage

LCOV       := lcov
GCOV_TOOL  := gcov-13
COV_INFO   := build/gcc-debug/coverage.info
COV_FILTERED := build/gcc-debug/coverage_filtered.info
COV_DIR    := $(COV_BUILD_DIR)/coverage_report
GENHTML    := genhtml

coverage:
	cmake --preset=gcc-debug
	cmake --build --preset=gcc-debug
	ctest --preset=gcc-test
	mkdir -p $(COV_DIR)
	@echo "Capturing coverage data..."
	$(LCOV) --capture \
	        --directory build/gcc-debug \
	        --output-file $(COV_INFO) \
	        --gcov-tool=$(GCOV_TOOL) \
	        --base-directory . >/dev/null

	@echo "Excluding unnecessary files..."
	$(LCOV) --remove $(COV_INFO) \
	        '*/test/*' \
	        '*/_deps/*' \
	        '/usr/*' \
	        '*/Catch2/*' \
			--output-file $(COV_FILTERED) \
			--ignore-errors unused >/dev/null

	@echo "Generating HTML report..."
	@mkdir -p $(COV_DIR)
	$(GENHTML) $(COV_FILTERED) --output-directory $(COV_DIR) >/dev/null

	@echo -n "Total Coverage: "
	$(LCOV) -l $(COV_FILTERED) 2>/dev/null | grep Total | sed 's/|//g' | sed 's/Total://g' | awk '{print $$1}' | sed s/%//g > $(COV_DIR)/total
	@cat $(COV_DIR)/total

    
# Static analysis
# --------------------------------------------------------------------
.PHONY: static-analysis

STATIC_ANALYSIS_DIR := $(BUILD_DIR)/static_analysis

static-analysis:
	@echo "Running static analysis"
	@mkdir -p $(STATIC_ANALYSIS_DIR)
	@cd $(STATIC_ANALYSIS_DIR) && \
		cmake -S $(ROOT_DIR) \
		    -B $(STATIC_ANALYSIS_DIR) \
			-G "Unix Makefiles" \
			-DCMAKE_BUILD_TYPE=Release \
			-DENABLE_STATIC_ANALYSIS=ON && \
	    make -C $(STATIC_ANALYSIS_DIR) static_analysis

# Documentation
# --------------------------------------------------------------------
.PHONY: documents

DOC_SRC_DIR := $(ROOT_DIR)
DOC_BUILD_DIR := $(BUILD_DIR)

documents:
	@echo "Building documentation from $(DOC_SRC_DIR) into $(DOC_BUILD_DIR)"
	@mkdir -p "$(DOC_BUILD_DIR)"
	cd "$(DOC_BUILD_DIR)" && \
		cmake -S "$(DOC_SRC_DIR)" -B $(DOC_BUILD_DIR) \
		      -G"Unix Makefiles" \
			  -DCMAKE_CXX_STANDARD=23 \
			  -DBUILD_DOCS=ON && \
		$(MAKE)

# Cleanup
# --------------------------------------------------------------------
.PHONY: clean
clean:
	@echo "Cleaning up build artifacts"
	@rm -rf build/*

# Help 
# --------------------------------------------------------------------
%::
	@echo "Unknown target: $@"
	@echo "Use 'make help' to see available targets."
	@false

# Print a list of non-config targets (based on http://stackoverflow.com/a/26339924/1487069)
help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/(^|\n)# Files(\n|$$)/,/(^|\n)# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(subst $(space),|,$(ALL_CONFIG_TARGETS)))$$' -e '_default$$' -e '^(Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."