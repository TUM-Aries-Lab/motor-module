#!/usr/bin/env python3
"""Generate a Markdown tree of the project directory.

Usage:
    python repo_tree.py
    python repo_tree.py --update-readme
"""

from __future__ import annotations

import argparse
import fnmatch
from pathlib import Path

from loguru import logger

START_MARKER = "<!-- TREE-START -->"
END_MARKER = "<!-- TREE-END -->"
ENCODING = "utf-8"

# Manual ignores are still allowed
MANUAL_IGNORE = {
    ".git",
    ".venv",
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".DS_Store",
    "data",
    "simulink",
    ".github",
}


def load_gitignore_patterns(path: Path) -> list[str]:
    """Load patterns from .gitignore."""
    gitignore = path / ".gitignore"
    if not gitignore.exists():
        return []

    patterns: list[str] = []
    for line in gitignore.read_text().splitlines():
        line_str = line.strip()
        if not line_str or line_str.startswith("#"):
            continue
        patterns.append(line_str)
    return patterns


def is_ignored(path: Path, patterns: list[str]) -> bool:
    """Return True if path matches any ignore pattern.

    Supports:
    - literal names
    - wildcard patterns
    - directory patterns (e.g. logs/)
    """
    name = path.name

    # Manual ignores first
    if name in MANUAL_IGNORE:
        return True

    for pattern in patterns:
        # Directory ignore
        if pattern.endswith("/") and path.is_dir():
            if fnmatch.fnmatch(name + "/", pattern):
                return True

        # File/directory wildcard
        if fnmatch.fnmatch(name, pattern):
            return True

    return False


def build_tree(path: Path, patterns: list[str], prefix: str = "") -> list[str]:
    """Recursively build a tree representation."""
    entries = sorted(
        [p for p in path.iterdir() if not is_ignored(p, patterns)],
        key=lambda x: (x.is_file(), x.name),
    )

    lines: list[str] = []

    for i, entry in enumerate(entries):
        connector = "└── " if i == len(entries) - 1 else "├── "

        lines.append(f"{prefix}{connector}{entry.name}")

        if entry.is_dir():
            extension = "    " if i == len(entries) - 1 else "│   "
            lines.extend(build_tree(entry, patterns, prefix + extension))

    return lines


def generate_markdown_tree() -> str:
    """Return the full markdown tree as a formatted code block."""
    root = Path(".").resolve()

    gitignore_patterns = load_gitignore_patterns(root)
    tree_lines = build_tree(root, gitignore_patterns)
    tree_text = "\n".join(tree_lines)

    return f"```\n{tree_text}\n```"


def update_readme_block(readme_path: Path) -> None:
    """Replace the section between markers with the generated tree."""
    readme = readme_path.read_text(encoding=ENCODING).splitlines()

    if START_MARKER not in readme or END_MARKER not in readme:
        raise RuntimeError(
            f"README.md must contain '{START_MARKER}' and '{END_MARKER}' markers."
        )

    start = readme.index(START_MARKER) + 1
    end = readme.index(END_MARKER)

    tree = generate_markdown_tree().splitlines()

    new_readme = readme[:start] + tree + readme[end:]
    readme_path.write_text("\n".join(new_readme), encoding=ENCODING)

    logger.success("README updated with latest repo tree (gitignored files excluded).")


def main() -> None:
    """Create a Markdown tree of the project directory."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--update-readme", action="store_true")
    args = parser.parse_args()

    if args.update_readme:
        update_readme_block(Path("README.md"))
    else:
        logger.info(generate_markdown_tree())


if __name__ == "__main__":
    main()
