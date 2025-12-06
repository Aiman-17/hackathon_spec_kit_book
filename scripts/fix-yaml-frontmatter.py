#!/usr/bin/env python3
"""
Fix YAML frontmatter issues in generated chapters.
Properly quotes values that contain special YAML characters.
"""

from pathlib import Path
import re
import sys

REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"


def fix_yaml_value(line: str) -> str:
    """
    Fix a YAML key-value line by quoting the value if needed.

    Args:
        line: A YAML line like "title: Some Title: With Colon"

    Returns:
        Fixed line like 'title: "Some Title: With Colon"'
    """
    # Match YAML key: value pattern
    match = re.match(r'^(\s*)([\w_]+):\s+(.+)$', line)
    if not match:
        return line

    indent, key, value = match.groups()
    value = value.rstrip()

    # Check if value needs quoting
    needs_quoting = False

    # Already quoted?
    if (value.startswith('"') and value.endswith('"')) or \
       (value.startswith("'") and value.endswith("'")):
        return line

    # Arrays and objects don't need quoting
    if (value.startswith('[') and value.endswith(']')) or \
       (value.startswith('{') and value.endswith('}')):
        return line

    # Contains special characters that need quoting (excluding array/object brackets)
    special_chars = [':', '#', '&', '*', '!', '|', '>', '@', '`']
    if any(char in value for char in special_chars):
        needs_quoting = True

    # Quote if needed
    if needs_quoting:
        # Escape any existing quotes
        value = value.replace('"', '\\"')
        return f'{indent}{key}: "{value}"\n'

    return line


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

        modified = False
        in_frontmatter = False
        frontmatter_start = -1
        frontmatter_end = -1

        # Find frontmatter boundaries
        for i, line in enumerate(lines):
            if line.strip() == '---':
                if frontmatter_start == -1:
                    frontmatter_start = i
                    in_frontmatter = True
                elif in_frontmatter:
                    frontmatter_end = i
                    break

        if frontmatter_start == -1 or frontmatter_end == -1:
            print(f"[SKIP] No frontmatter found: {file_path.relative_to(REPO_ROOT)}")
            return False

        # Fix frontmatter lines
        for i in range(frontmatter_start + 1, frontmatter_end):
            original_line = lines[i]
            fixed_line = fix_yaml_value(original_line)

            if fixed_line != original_line:
                lines[i] = fixed_line
                modified = True

        if modified:
            # Write back to file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.writelines(lines)
            print(f"[OK] Fixed YAML: {file_path.relative_to(REPO_ROOT)}")
            return True
        else:
            print(f"[SKIP] Already correct: {file_path.relative_to(REPO_ROOT)}")
            return False

    except Exception as e:
        print(f"[ERROR] Error processing {file_path}: {str(e)}")
        return False


def main():
    """Fix frontmatter in all generated chapters"""
    print("Fixing YAML frontmatter in all chapters...\n")

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
