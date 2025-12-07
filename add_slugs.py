#!/usr/bin/env python3
"""Add custom slug frontmatter to all chapter files."""

import re
from pathlib import Path

def add_slug_to_file(file_path):
    """Add slug frontmatter to a chapter file."""
    # Read the file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract filename without extension and number prefix
    filename = file_path.stem  # e.g., "01-introduction-to-physical-ai"

    # Remove number prefix (01-, 02-, etc.)
    slug = re.sub(r'^\d+-', '', filename)

    # Check if file already has slug in frontmatter
    if 'slug:' in content:
        print(f'Skipping {file_path.name} (already has slug)')
        return False

    # Find frontmatter block
    frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)

    if not frontmatter_match:
        print(f'ERROR: No frontmatter found in {file_path.name}')
        return False

    # Add slug after title
    frontmatter = frontmatter_match.group(1)

    # Insert slug after title line
    lines = frontmatter.split('\n')
    new_lines = []
    for line in lines:
        new_lines.append(line)
        if line.startswith('title:'):
            new_lines.append(f'slug: {slug}')

    new_frontmatter = '\n'.join(new_lines)
    new_content = content.replace(frontmatter, new_frontmatter)

    # Write back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f'Added slug "{slug}" to {file_path.name}')
    return True

def main():
    docs_dir = Path('frontend/docs')

    # Process all module directories
    module_dirs = sorted([d for d in docs_dir.iterdir() if d.is_dir() and d.name.startswith('module-')])

    total_updated = 0

    for module_dir in module_dirs:
        print(f'\nProcessing {module_dir.name}...')

        # Get all markdown files in this module
        md_files = sorted(module_dir.glob('*.md'))

        for md_file in md_files:
            # Skip .broken files
            if '.broken' in md_file.name:
                print(f'Skipping {md_file.name} (broken file)')
                continue

            if add_slug_to_file(md_file):
                total_updated += 1

    print(f'\nUpdated {total_updated} files with custom slugs!')
    return 0

if __name__ == '__main__':
    exit(main())
