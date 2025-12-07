#!/usr/bin/env python3
"""Check markdown files for syntax errors."""

import re
from pathlib import Path

def check_code_blocks(file_path):
    """Check if code blocks are properly closed."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Count triple backticks
    backticks = content.count('```')

    # Should be even (each opening has a closing)
    if backticks % 2 != 0:
        return True, f'Odd number of backticks: {backticks}'

    return False, 'OK'

def main():
    docs_dir = Path('frontend/docs')
    files = sorted(docs_dir.glob('module-*/*.md'))

    errors = []

    for file_path in files:
        has_error, msg = check_code_blocks(file_path)
        if has_error:
            errors.append((str(file_path.relative_to(docs_dir)), msg))

    if errors:
        print('Files with potential code block errors:')
        for file, msg in errors:
            print(f'  {file}: {msg}')
        return 1
    else:
        print('✓ All files have matching code block markers!')
        return 0

if __name__ == '__main__':
    exit(main())
