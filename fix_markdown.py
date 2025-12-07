#!/usr/bin/env python3
"""Fix unclosed code blocks in markdown files."""

import re
from pathlib import Path

def fix_code_blocks(file_path):
    """Fix unclosed code blocks in a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    fixed_lines = []
    in_code_block = False
    code_block_lang = None

    for i, line in enumerate(lines):
        # Check for code block markers
        code_match = re.match(r'^```(\w*)', line)

        if code_match:
            in_code_block = not in_code_block
            if in_code_block:
                code_block_lang = code_match.group(1)
            else:
                code_block_lang = None

        # Check if we're at ::: and we have an unclosed code block
        if line.startswith(':::') and in_code_block:
            # Insert closing ``` before the :::
            fixed_lines.append('```\n')
            in_code_block = False
            code_block_lang = None

        # Check if we're at a heading (## or ###) with unclosed code block
        elif re.match(r'^#{2,3}\s', line) and in_code_block:
            # Insert closing ``` before the heading
            fixed_lines.append('```\n')
            fixed_lines.append('\n')
            in_code_block = False
            code_block_lang = None

        fixed_lines.append(line)

    # If still in code block at end, close it
    if in_code_block:
        fixed_lines.append('```\n')

    return ''.join(fixed_lines)

def main():
    docs_dir = Path('frontend/docs')
    files_to_fix = [
        'module-1/02-humanoid-robotics-overview.md',
        'module-1/04-mathematics-for-robotics.md',
        'module-1/06-linear-algebra-calculus.md',
        'module-2/07-ros2-in-depth.md',
        'module-2/09-urdf-xacro-modeling.md',
        'module-2/10-gazebo-comparison.md',
        'module-2/11-humanoid-model-gazebo.md',
        'module-2/12-unity-robotics.md',
        'module-3/13-computer-vision.md',
        'module-3/14-isaac-sim-fundamentals.md',
        'module-3/15-isaac-ros-perception.md',
        'module-3/16-nav2-navigation.md',
        'module-3/17-mapping-localization.md',
        'module-3/18-motion-planning-humanoids.md',
        'module-3/19-vision-language-action.md',
        'module-4/20-integrating-perception-action-control.md',
        'module-4/21-ai-agents-robotics.md',
        'module-4/22-end-to-end-humanoid-pipeline.md',
        'module-4/23-multi-agent-coordination.md',
        'module-4/24-project-autonomous-humanoid.md',
        'module-4/25-final-capstone.md',
    ]

    fixed_count = 0
    for rel_path in files_to_fix:
        file_path = docs_dir / rel_path
        print(f'Fixing {rel_path}...')

        fixed_content = fix_code_blocks(file_path)

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(fixed_content)

        fixed_count += 1

    print(f'\n✓ Fixed {fixed_count} files!')
    return 0

if __name__ == '__main__':
    exit(main())
