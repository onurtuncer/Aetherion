# ------------------------------------------------------------------------------
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

import re

def get_project_name(cmake_file="CMakeLists.txt"):
    try:
        with open(cmake_file, 'r') as f:
            content = f.read()
            match = re.search(r'project\s*\(\s*([^\s\)]+)', content, re.IGNORECASE)
            return match.group(1) if match else "project_name_not_found"
    except FileNotFoundError:
        return "project_name_not_found"