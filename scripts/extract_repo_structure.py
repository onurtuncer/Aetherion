#!/usr/bin/env python3
"""
Extract and display the directory and file structure of a repository.
"""

import os
import argparse
from pathlib import Path

# Common directories/files to ignore by default
DEFAULT_IGNORE = {
    ".git", ".svn", ".hg",
    "__pycache__", ".pytest_cache", ".mypy_cache",
    "node_modules", ".next", "dist", "build", ".venv", "venv", "env",
    ".DS_Store", "Thumbs.db",
    ".idea", ".vscode",
}


def build_tree(
    root: Path,
    prefix: str = "",
    ignore: set = None,
    show_hidden: bool = False,
    max_depth: int = None,
    current_depth: int = 0,
    output_lines: list = None,
    stats: dict = None,
) -> list:
    if output_lines is None:
        output_lines = []
    if stats is None:
        stats = {"files": 0, "dirs": 0}
    if ignore is None:
        ignore = DEFAULT_IGNORE

    if max_depth is not None and current_depth >= max_depth:
        return output_lines

    try:
        entries = sorted(root.iterdir(), key=lambda e: (e.is_file(), e.name.lower()))
    except PermissionError:
        output_lines.append(f"{prefix}  [Permission Denied]")
        return output_lines

    entries = [
        e for e in entries
        if e.name not in ignore and (show_hidden or not e.name.startswith("."))
    ]

    for i, entry in enumerate(entries):
        is_last = i == len(entries) - 1
        connector = "└── " if is_last else "├── "
        extension = "    " if is_last else "│   "

        if entry.is_dir():
            stats["dirs"] += 1
            output_lines.append(f"{prefix}{connector}{entry.name}/")
            build_tree(
                entry,
                prefix=prefix + extension,
                ignore=ignore,
                show_hidden=show_hidden,
                max_depth=max_depth,
                current_depth=current_depth + 1,
                output_lines=output_lines,
                stats=stats,
            )
        else:
            stats["files"] += 1
            size = _format_size(entry.stat().st_size)
            output_lines.append(f"{prefix}{connector}{entry.name}  ({size})")

    return output_lines


def _format_size(size_bytes: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size_bytes < 1024:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.1f} TB"


def extract_structure(
    path: str = ".",
    ignore_extra: list = None,
    show_hidden: bool = False,
    max_depth: int = None,
    output_file: str = None,
) -> str:
    root = Path(path).resolve()
    if not root.exists():
        raise ValueError(f"Path does not exist: {root}")

    ignore = DEFAULT_IGNORE.copy()
    if ignore_extra:
        ignore.update(ignore_extra)

    stats = {"files": 0, "dirs": 0}
    lines = [f"{root.name}/"]
    build_tree(
        root,
        ignore=ignore,
        show_hidden=show_hidden,
        max_depth=max_depth,
        output_lines=lines,
        stats=stats,
    )

    summary = f"\n{stats['dirs']} directories, {stats['files']} files"
    lines.append(summary)
    result = "\n".join(lines)

    if output_file:
        with open(output_file, "w", encoding="utf-8") as f:
            f.write(result)
        print(f"Structure saved to: {output_file}")

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Extract and display the directory/file structure of a repository."
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=".",
        help="Path to the repository root (default: current directory)",
    )
    parser.add_argument(
        "--ignore", "-i",
        nargs="*",
        default=[],
        metavar="DIR",
        help="Additional directories/files to ignore",
    )
    parser.add_argument(
        "--hidden", "-H",
        action="store_true",
        help="Show hidden files and directories (starting with '.')",
    )
    parser.add_argument(
        "--depth", "-d",
        type=int,
        default=None,
        metavar="N",
        help="Maximum depth to traverse (default: unlimited)",
    )
    parser.add_argument(
        "--output", "-o",
        default=None,
        metavar="FILE",
        help="Save output to a file",
    )

    args = parser.parse_args()

    structure = extract_structure(
        path=args.path,
        ignore_extra=args.ignore,
        show_hidden=args.hidden,
        max_depth=args.depth,
        output_file=args.output,
    )
    print(structure)


if __name__ == "__main__":
    main()