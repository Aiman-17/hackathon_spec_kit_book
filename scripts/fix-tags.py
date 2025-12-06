#!/usr/bin/env python3
"""
Fix tags field in frontmatter to be proper YAML arrays.
"""

from pathlib import Path
import re
import sys

REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"


def fix_tags_field(content: str) -> tuple[str, bool]:
    """
    Fix tags field in frontmatter content.

    Args:
        content: Full file content

    Returns:
        Tuple of (fixed_content, was_modified)
    """
    # Pattern to match tags field with various formats
    # Match: tags: "value" or tags: 'value' or tags: value (but not tags: [value])
    pattern = r'^(tags:\s+)(["\']?)([^\[\n]+?)(["\']?)\s*$'

    lines = content.split('\n')
    modified = False
    in_frontmatter = False
    frontmatter_count = 0

    for i, line in enumerate(lines):
        # Track frontmatter boundaries
        if line.strip() == '---':
            frontmatter_count += 1
            if frontmatter_count == 1:
                in_frontmatter = True
            elif frontmatter_count == 2:
                in_frontmatter = False
            continue

        # Only process lines within frontmatter
        if not in_frontmatter:
            continue

        # Check if this is a tags field that needs fixing
        match = re.match(pattern, line)
        if match and not line.strip().startswith('tags: ['):
            prefix = match.group(1)
            tags_value = match.group(3).strip()

            # Remove quotes if present
            tags_value = tags_value.strip('"\'')

            # Split by comma and clean up
            tags = [tag.strip().rstrip(':') for tag in tags_value.split(',')]

            # Create proper YAML array
            tags_array = '[' + ', '.join(tags) + ']'

            lines[i] = prefix + tags_array
            modified = True

    return '\n'.join(lines), modified


def process_file(file_path: Path) -> bool:
    """
    Process a single markdown file.

    Args:
        file_path: Path to the file

    Returns:
        True if modified, False otherwise
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        fixed_content, was_modified = fix_tags_field(content)

        if was_modified:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(fixed_content)
            print(f"[OK] Fixed tags in: {file_path.relative_to(REPO_ROOT)}")
            return True
        else:
            print(f"[SKIP] No changes needed: {file_path.relative_to(REPO_ROOT)}")
            return False

    except Exception as e:
        print(f"[ERROR] Error processing {file_path}: {str(e)}")
        return False


def main():
    """Fix tags in all chapter files"""
    print("Fixing tags fields in all chapters...\n")

    chapter_files = sorted(DOCS_DIR.glob("module-*/*.md"))

    if not chapter_files:
        print("[ERROR] No chapter files found!")
        sys.exit(1)

    print(f"Found {len(chapter_files)} chapter files\n")

    fixed_count = 0
    for file_path in chapter_files:
        if process_file(file_path):
            fixed_count += 1

    print(f"\n[SUCCESS] Fixed {fixed_count} of {len(chapter_files)} files")


if __name__ == "__main__":
    main()
