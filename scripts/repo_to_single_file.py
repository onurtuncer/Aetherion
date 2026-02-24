#!/usr/bin/env python3
"""
repo_to_single_txt.py

Export a repository into one text file:
- File tree summary at the top
- Grouped by module
- Headers before sources
- Auto-ignore vendor/3rd-party dirs and common build dirs
- Prints approximate token count to the screen

Usage:
  python repo_to_single_txt.py /path/to/repo output.txt

Notes:
- Token estimate is approximate: ~chars/4 (rough heuristic).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


# -----------------------------
# Ignore rules
# -----------------------------

BASE_IGNORE_DIRS = {
    ".git", ".github",
    "build", "cmake-build-debug", "cmake-build-release", "cmake-build",
    "__pycache__", ".idea", ".vscode",
    "dist", "out", ".venv", "venv",
    ".cache", ".pytest_cache",
    "node_modules",
    "data", 
    "scripts"
}

VENDOR_NAMES = {
    "vendor", "third_party", "thirdparty",
    "external", "extern",
    "deps", "dependencies",
    "submodules",
    "vcpkg_installed",
    "_deps",  # CMake FetchContent often places deps here
}

# Extensions to include (text)
TEXT_EXTENSIONS = {
    # C/C++
    ".h", ".hh", ".hpp", ".hxx", ".inl",
    ".c", ".cc", ".cpp", ".cxx",
    # Python
    ".py",
    # Build / config
    ".cmake", ".txt", ".md", ".rst",
    ".json", ".yaml", ".yml", ".toml",
    ".ini", ".cfg",
    ".sh", ".bat", ".ps1",
    # Docs / misc
    ".tex",
    ".csv"
}

MAX_FILE_SIZE_MB = 5.0


# -----------------------------
# Ordering preferences
# -----------------------------

HEADER_EXTS = {".h", ".hh", ".hpp", ".hxx", ".inl"}
SOURCE_EXTS = {".c", ".cc", ".cpp", ".cxx"}

PREFERRED_ROOTS_ORDER = [
    "include",
    "src",
    "tests",
    "test",
    "apps",
    "examples",
    "tools",
    "cmake",
    "docs",
]

# If module cannot be inferred, put it here
FALLBACK_MODULE = "(root)"


@dataclass(frozen=True)
class FileItem:
    rel_path: Path
    abs_path: Path
    size_bytes: int


# -----------------------------
# Detection helpers
# -----------------------------

def is_text_file(path: Path) -> bool:
    return path.suffix.lower() in TEXT_EXTENSIONS


def safe_read(path: Path) -> str:
    # Try utf-8, then latin-1, otherwise mark unreadable
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        try:
            return path.read_text(encoding="latin-1")
        except Exception:
            return "[Could not decode file]\n"
    except Exception as e:
        return f"[Error reading file: {e}]\n"


def is_git_submodule_dir(path: Path) -> bool:
    # Some submodules have .git file (not a dir) pointing to gitdir:
    return (path / ".git").is_file()


def contains_foreign_cmake_project(path: Path) -> bool:
    """
    Heuristic for embedded third-party projects:
    If directory contains CMakeLists.txt with 'project(' -> likely external dep.
    """
    cmake_file = path / "CMakeLists.txt"
    if not cmake_file.exists():
        return False
    try:
        text = cmake_file.read_text(encoding="utf-8", errors="ignore").lower()
        return "project(" in text
    except Exception:
        return False


def should_ignore_dir(dir_path: Path) -> bool:
    name = dir_path.name.lower()
    if name in BASE_IGNORE_DIRS:
        return True
    if name in VENDOR_NAMES:
        return True
    if is_git_submodule_dir(dir_path):
        return True
    if contains_foreign_cmake_project(dir_path):
        return True
    return False


# -----------------------------
# Module grouping
# -----------------------------

def infer_module(rel_path: Path) -> str:
    """
    Grouping rule (practical for C++ repos):
    - If file is under include/<module>/..., src/<module>/..., tests/<module>/...
      then module = <module>
    - Else module = top-level directory (e.g., docs, cmake, tools)
    - Else module = (root)
    """
    parts = rel_path.parts
    if not parts:
        return FALLBACK_MODULE

    top = parts[0]

    # Recognize common roots where second segment is "module name"
    if top in {"include", "src", "tests", "test", "apps", "examples", "tools"}:
        if len(parts) >= 2:
            return parts[1]
        return top

    # Otherwise: top-level folder is the module
    if len(parts) >= 1 and rel_path.parent != Path("."):
        return top

    return FALLBACK_MODULE


def root_rank(rel_path: Path) -> int:
    """
    Sort modules/sections so include/src/tests appear earlier.
    """
    parts = rel_path.parts
    if not parts:
        return 999
    top = parts[0]
    try:
        return PREFERRED_ROOTS_ORDER.index(top)
    except ValueError:
        return 999


def file_priority(rel_path: Path) -> Tuple[int, str]:
    """
    Within a module: headers first, then sources, then others.
    """
    ext = rel_path.suffix.lower()
    if ext in HEADER_EXTS:
        p = 0
    elif ext in SOURCE_EXTS:
        p = 1
    else:
        p = 2
    # stable secondary sort by path string
    return (p, rel_path.as_posix().lower())


# -----------------------------
# File tree summary
# -----------------------------

def build_tree_summary(files: List[Path], max_depth: int = 4) -> str:
    """
    Produce a compact tree summary like:
      include/
        MyLib/
          Foo.hpp
      src/
        MyLib/
          Foo.cpp
    Depth-limited; deeper paths shown as '...'
    """
    # Build a nested dict tree
    tree: Dict[str, dict] = {}
    for p in files:
        parts = p.parts
        node = tree
        for i, part in enumerate(parts):
            if i >= max_depth:
                node.setdefault("...", {})
                break
            node = node.setdefault(part, {})

    def render(node: Dict[str, dict], indent: str = "") -> List[str]:
        lines: List[str] = []
        keys = sorted(node.keys(), key=lambda k: (k != "...", k.lower()))
        for k in keys:
            lines.append(f"{indent}{k}")
            child = node[k]
            if child:
                lines.extend(render(child, indent + "  "))
        return lines

    lines = render(tree)
    return "\n".join(lines) + ("\n" if lines else "")


# -----------------------------
# Token estimate
# -----------------------------

def approx_tokens_from_chars(n_chars: int) -> int:
    # Very rough heuristic (works OK for English-ish + code):
    # 1 token ~= 4 chars
    return max(1, n_chars // 4)


# -----------------------------
# Main export
# -----------------------------

def collect_ignored_dirs(repo_root: Path) -> List[Path]:
    ignored: List[Path] = []
    for dirpath, dirnames, _filenames in os.walk(repo_root):
        p = Path(dirpath)
        # mutate dirnames in-place so os.walk prunes recursion
        keep = []
        for d in dirnames:
            full = p / d
            if should_ignore_dir(full):
                ignored.append(full.resolve())
            else:
                keep.append(d)
        dirnames[:] = keep
    # sort longest-first so prefix matching is safer
    ignored.sort(key=lambda x: len(str(x)), reverse=True)
    return ignored


def is_under_any_dir(path: Path, dirs: List[Path]) -> bool:
    p = path.resolve()
    ps = str(p)
    for d in dirs:
        ds = str(d)
        if ps.startswith(ds + os.sep) or ps == ds:
            return True
    return False


def export_repo(repo_root: Path, output_file: Path) -> None:
    repo_root = repo_root.resolve()
    ignored_dirs = collect_ignored_dirs(repo_root)

    # Gather file items
    items: List[FileItem] = []
    for path in repo_root.rglob("*"):
        if not path.is_file():
            continue
        if is_under_any_dir(path, ignored_dirs):
            continue
        if not is_text_file(path):
            continue
        size_mb = path.stat().st_size / (1024 * 1024)
        if size_mb > MAX_FILE_SIZE_MB:
            continue
        rel = path.relative_to(repo_root)
        items.append(FileItem(rel_path=rel, abs_path=path, size_bytes=path.stat().st_size))

    # Build tree summary from relative paths
    rel_paths = [it.rel_path for it in items]
    tree_summary = build_tree_summary(rel_paths, max_depth=4)

    # Group by module
    modules: Dict[str, List[FileItem]] = {}
    for it in items:
        mod = infer_module(it.rel_path)
        modules.setdefault(mod, []).append(it)

    # Sort modules: by the best (min) root rank among their files, then name
    def module_sort_key(mod_name: str) -> Tuple[int, str]:
        rank = min((root_rank(it.rel_path) for it in modules[mod_name]), default=999)
        return (rank, mod_name.lower())

    module_names = sorted(modules.keys(), key=module_sort_key)

    # Sort files within each module
    for m in module_names:
        modules[m].sort(key=lambda it: file_priority(it.rel_path))

    # Write output and compute char count for token estimate
    total_chars = 0

    with output_file.open("w", encoding="utf-8") as out:
        header = "=" * 80 + "\n"
        header += f"Repository Export: {repo_root}\n"
        header += "=" * 80 + "\n\n"
        header += "FILE TREE (depth-limited):\n"
        header += "-" * 80 + "\n"
        header += (tree_summary if tree_summary.strip() else "(no files matched)\n")
        header += "-" * 80 + "\n\n"
        out.write(header)
        total_chars += len(header)

        # Modules
        for mod in module_names:
            mod_hdr = "\n" + "#" * 80 + "\n"
            mod_hdr += f"MODULE: {mod}\n"
            mod_hdr += "#" * 80 + "\n\n"
            out.write(mod_hdr)
            total_chars += len(mod_hdr)

            for it in modules[mod]:
                size_mb = it.size_bytes / (1024 * 1024)
                file_block_hdr = "\n" + "-" * 80 + "\n"
                file_block_hdr += f"FILE: {it.rel_path.name}\n"
                file_block_hdr += f"PATH: {it.rel_path.as_posix()}\n"
                file_block_hdr += f"SIZE: {size_mb:.2f} MB\n"
                file_block_hdr += "-" * 80 + "\n\n"
                out.write(file_block_hdr)
                total_chars += len(file_block_hdr)

                content = safe_read(it.abs_path)
                out.write(content)
                out.write("\n\n")
                total_chars += len(content) + 2

    approx_tokens = approx_tokens_from_chars(total_chars)

    # Print summary to screen
    print("\n✓ Repository exported.")
    print(f"  Repo:   {repo_root}")
    print(f"  Output: {output_file.resolve()}")
    print(f"  Files included: {len(items)}")
    print(f"  Ignored dirs detected: {len(ignored_dirs)}")
    print(f"  Approx chars written: {total_chars:,}")
    print(f"  Approx tokens: {approx_tokens:,}  (heuristic: ~chars/4)\n")


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage:\n  python repo_to_single_txt.py /path/to/repo output.txt")
        return 1

    repo_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])

    if not repo_path.exists():
        print("Error: repo path does not exist.")
        return 1
    if not repo_path.is_dir():
        print("Error: repo path is not a directory.")
        return 1

    export_repo(repo_path, out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())