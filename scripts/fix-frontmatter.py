#!/usr/bin/env python3
"""
Fix malformed frontmatter in generated chapters.
Removes ```yaml and trailing ``` from frontmatter blocks.
"""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"


def fix_frontmatter(file_path: Path) -> bool:
    """
    Fix frontmatter in a markdown file.

    Args:
        file_path: Path to the markdown file

    Returns:
        True if file was modified, False otherwise
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        if not lines:
            return False

        # Check if first line is ```yaml
        if lines[0].strip() == '```yaml':
            # Remove first line
            lines = lines[1:]

            # Find and fix the closing ---
            for i, line in enumerate(lines):
                if line.strip() == '```':
                    # Remove this line
                    lines.pop(i)
                    break

            # Write back to file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.writelines(lines)

            print(f"[OK] Fixed: {file_path.relative_to(REPO_ROOT)}")
            return True
        else:
            print(f"[SKIP] Already correct: {file_path.relative_to(REPO_ROOT)}")
            return False

    except Exception as e:
        print(f"[ERROR] Error processing {file_path}: {str(e)}")
        return False


def main():
    """Fix frontmatter in all generated chapters"""
    print("Fixing frontmatter in all chapters...\n")

    # Find all markdown files in module directories
    chapter_files = sorted(DOCS_DIR.glob("module-*/*.md"))

    if not chapter_files:
        print("[ERROR] No chapter files found!")
        sys.exit(1)

    print(f"Found {len(chapter_files)} chapter files\n")

    fixed_count = 0
    for file_path in chapter_files:
        if fix_frontmatter(file_path):
            fixed_count += 1

    print(f"\n[SUCCESS] Fixed {fixed_count} of {len(chapter_files)} files")


if __name__ == "__main__":
    main()
