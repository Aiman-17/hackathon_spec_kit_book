#!/usr/bin/env python3
"""
AI-Powered Chapter Generation Script

Generates high-quality textbook chapters using OpenAI GPT-4 for the
Physical AI & Humanoid Robotics textbook.

Usage:
    python scripts/generate-chapters.py --module 1 --chapter 1
    python scripts/generate-chapters.py --all
    python scripts/generate-chapters.py --chapter-id module-1/01-introduction-to-physical-ai
"""

import argparse
import asyncio
import json
import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from openai import AsyncOpenAI
import yaml
from dotenv import load_dotenv

# Load environment variables from backend/.env
SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent
ENV_PATH = REPO_ROOT / "backend" / ".env"
load_dotenv(ENV_PATH)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ==========================================
# Configuration
# ==========================================

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    logger.error("OPENAI_API_KEY not found in environment variables")
    sys.exit(1)

# Initialize OpenAI client (with OpenRouter support)
# OpenRouter uses the same client interface but different base URL
client = AsyncOpenAI(
    api_key=OPENAI_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)

# Project paths
DOCS_DIR = REPO_ROOT / "frontend" / "docs"
OUTPUT_DIR = DOCS_DIR
COURSE_OUTLINE_PATH = REPO_ROOT / "docs-source" / "course-outline.yaml"


# ==========================================
# Load Course Outline from YAML
# ==========================================

def load_course_outline() -> Dict:
    """Load course outline from YAML file"""
    try:
        with open(COURSE_OUTLINE_PATH, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        logger.error(f"Failed to load course outline: {str(e)}")
        sys.exit(1)


def build_chapter_metadata() -> Dict:
    """Build chapter metadata from course outline"""
    outline = load_course_outline()
    metadata = {}

    for module in outline['modules']:
        module_id = module['id']
        metadata[module_id] = {
            "title": f"Module {module['number']}: {module['title']}",
            "chapters": []
        }

        for chapter in module['chapters']:
            # Generate keywords from chapter title
            title_words = chapter['title'].lower().split()
            keywords = [chapter['title']] + title_words[:5]  # Use title + first 5 words as keywords

            metadata[module_id]["chapters"].append({
                "id": chapter['slug'],
                "title": chapter['title'],
                "number": chapter['number'],
                "keywords": keywords
            })

    return metadata


# Load chapter metadata
CHAPTER_METADATA = build_chapter_metadata()


# ==========================================
# Chapter Generation System Prompt
# ==========================================

SYSTEM_PROMPT = """You are an expert robotics and AI educator creating a comprehensive, AI-native textbook on Physical AI and Humanoid Robotics.

**CRITICAL REQUIREMENTS - Chapter Structure (MUST INCLUDE ALL):**

1. **Docusaurus Frontmatter** (at very top):
```yaml
---
title: "[Chapter Title]"
sidebar_position: [number]
description: "[Brief 1-sentence description]"
tags: [relevant, keywords, here]
---
```

2. **Summary** (≥3 sentences) - Overview of what the chapter covers

3. **Learning Objectives** (≥3 objectives):
   - Use Bloom's taxonomy verbs: explain, implement, analyze, evaluate, create
   - Example: "Implement a ROS 2 publisher node that sends sensor data at 10 Hz"

4. **Prerequisites** (explicit list or state "None")

5. **Main Content** (3-5 major H2 sections with H3 subsections):
   - Clear, engaging explanations
   - Code examples in Python/ROS 2 with proper syntax highlighting
   - Mermaid diagrams for architectures
   - Real-world examples and applications

6. **Key Takeaways** (≥3 bullet points summarizing main concepts)

7. **Glossary** (≥5 technical terms with definitions)

8. **Review Questions** (≥3 questions aligned with learning objectives)

**Formatting Requirements:**
- Use consistent heading hierarchy (H1 → H2 → H3, no level skipping)
- Code blocks MUST specify language: ```python, ```bash, ```yaml
- Define technical terms on first use
- NO placeholder text (TODO, TBD, [INSERT_HERE], etc.)
- Use Docusaurus admonitions: :::tip, :::note, :::warning, :::info
- Include Mermaid diagrams where helpful

**Target**: 2500-3500 words, technically accurate, pedagogically rigorous."""


# ==========================================
# Chapter Generation Functions
# ==========================================

async def generate_chapter_content(
    module_id: str,
    chapter: Dict[str, str],
    module_title: str
) -> str:
    """
    Generate chapter content using OpenAI GPT-4

    Args:
        module_id: Module identifier (e.g., "module-1")
        chapter: Chapter metadata dict with id, title, keywords, number
        module_title: Full module title

    Returns:
        Generated markdown content
    """
    chapter_id = chapter["id"]
    chapter_title = chapter["title"]
    chapter_number = chapter["number"]
    keywords = ", ".join(chapter["keywords"])

    user_prompt = f"""Generate a comprehensive textbook chapter for:

**Module**: {module_title}
**Chapter Number**: {chapter_number}
**Chapter Title**: {chapter_title}
**Focus Keywords**: {keywords}

IMPORTANT:
- Set sidebar_position to {chapter_number} in the frontmatter
- This chapter is for advanced undergraduate students studying robotics and AI
- Include practical code examples in Python/ROS 2
- Add Mermaid diagrams for system architectures
- Follow ALL requirements from the system prompt (Summary, Learning Objectives, Prerequisites, Key Takeaways, Glossary, Review Questions)

Generate the complete chapter content now with proper Docusaurus frontmatter."""

    logger.info(f"Generating chapter: {module_id}/{chapter_id}")

    try:
        response = await client.chat.completions.create(
            model="anthropic/claude-3-haiku",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=3500,  # Reduced for cost-effectiveness while maintaining quality
        )

        content = response.choices[0].message.content
        logger.info(f"✅ Generated {len(content)} characters for {chapter_id}")
        return content

    except Exception as e:
        logger.error(f"❌ Failed to generate chapter {chapter_id}: {str(e)}")
        raise


def save_chapter(module_id: str, chapter_id: str, content: str) -> Path:
    """
    Save generated chapter to file

    Args:
        module_id: Module identifier
        chapter_id: Chapter identifier
        content: Markdown content

    Returns:
        Path to saved file
    """
    module_dir = OUTPUT_DIR / module_id
    module_dir.mkdir(parents=True, exist_ok=True)

    output_file = module_dir / f"{chapter_id}.md"

    with open(output_file, "w", encoding="utf-8") as f:
        f.write(content)

    logger.info(f"💾 Saved chapter to: {output_file}")
    return output_file


async def generate_single_chapter(module_id: str, chapter_number: int):
    """Generate a single chapter"""
    if module_id not in CHAPTER_METADATA:
        logger.error(f"Invalid module ID: {module_id}")
        return

    module_data = CHAPTER_METADATA[module_id]
    chapters = module_data["chapters"]

    if chapter_number < 1 or chapter_number > len(chapters):
        logger.error(f"Invalid chapter number: {chapter_number}")
        return

    chapter = chapters[chapter_number - 1]
    content = await generate_chapter_content(module_id, chapter, module_data["title"])
    save_chapter(module_id, chapter["id"], content)


async def generate_all_chapters():
    """Generate all chapters for all modules"""
    logger.info("🚀 Starting generation of all chapters...")

    total_chapters = sum(len(module["chapters"]) for module in CHAPTER_METADATA.values())
    logger.info(f"Total chapters to generate: {total_chapters}")

    for module_id, module_data in CHAPTER_METADATA.items():
        logger.info(f"\n📚 Generating {module_data['title']}")

        for chapter in module_data["chapters"]:
            try:
                content = await generate_chapter_content(
                    module_id,
                    chapter,
                    module_data["title"]
                )
                save_chapter(module_id, chapter["id"], content)

                # Rate limiting - wait between API calls
                await asyncio.sleep(2)

            except Exception as e:
                logger.error(f"Failed to generate {chapter['id']}: {str(e)}")
                continue

    logger.info("\n✅ Chapter generation complete!")


# ==========================================
# CLI Interface
# ==========================================

def main():
    """Main entry point for chapter generation script"""
    parser = argparse.ArgumentParser(
        description="Generate AI-powered textbook chapters"
    )
    parser.add_argument(
        "--module",
        type=int,
        help="Module number (1-4)"
    )
    parser.add_argument(
        "--chapter",
        type=int,
        help="Chapter number within module"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Generate all chapters"
    )

    args = parser.parse_args()

    if args.all:
        asyncio.run(generate_all_chapters())
    elif args.module and args.chapter:
        module_id = f"module-{args.module}"
        asyncio.run(generate_single_chapter(module_id, args.chapter))
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
