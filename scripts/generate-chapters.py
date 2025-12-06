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

import openai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

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

openai.api_key = OPENAI_API_KEY

# Project paths
REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"
OUTPUT_DIR = DOCS_DIR


# ==========================================
# Chapter Metadata
# ==========================================

CHAPTER_METADATA = {
    "module-1": {
        "title": "Module 1: Foundations of Physical AI & Robotics",
        "chapters": [
            {"id": "01-introduction-to-physical-ai", "title": "Introduction to Physical AI", "keywords": ["physical AI", "embodied intelligence", "robotics overview"]},
            {"id": "02-history-evolution-robotics", "title": "History and Evolution of Robotics", "keywords": ["robotics history", "industrial robots", "humanoid development"]},
            {"id": "03-key-concepts-terminology", "title": "Key Concepts and Terminology", "keywords": ["degrees of freedom", "kinematics", "dynamics", "end effector"]},
            {"id": "04-hardware-components-sensors", "title": "Hardware Components and Sensors", "keywords": ["sensors", "lidar", "cameras", "IMU", "force sensors"]},
            {"id": "05-actuators-control-systems", "title": "Actuators and Control Systems", "keywords": ["motors", "servos", "hydraulics", "PID control"]},
            {"id": "06-ethical-considerations", "title": "Ethical Considerations in Robotics", "keywords": ["robot ethics", "safety", "privacy", "accountability"]},
        ]
    },
    "module-2": {
        "title": "Module 2: Simulation Environments & Robotics Software",
        "chapters": [
            {"id": "07-introduction-simulation", "title": "Introduction to Robotics Simulation", "keywords": ["simulation", "virtual testing", "physics engines"]},
            {"id": "08-nvidia-isaac-sim", "title": "NVIDIA Isaac Sim", "keywords": ["Isaac Sim", "Omniverse", "GPU acceleration", "photorealistic simulation"]},
            {"id": "09-gazebo-simulator", "title": "Gazebo Simulator", "keywords": ["Gazebo", "ROS integration", "sensor simulation"]},
            {"id": "10-ros2-fundamentals", "title": "ROS 2 Fundamentals", "keywords": ["ROS 2", "nodes", "topics", "services", "actions"]},
            {"id": "11-ros2-advanced-topics", "title": "ROS 2 Advanced Topics", "keywords": ["DDS", "quality of service", "lifecycle nodes", "composition"]},
            {"id": "12-integration-simulation-ros2", "title": "Integration of Simulation with ROS 2", "keywords": ["sim-to-real", "ROS 2 bridge", "testing pipelines"]},
        ]
    },
    "module-3": {
        "title": "Module 3: Advanced Perception, Navigation & Control",
        "chapters": [
            {"id": "13-computer-vision-robotics", "title": "Computer Vision for Robotics", "keywords": ["computer vision", "object detection", "semantic segmentation"]},
            {"id": "14-sensor-fusion-techniques", "title": "Sensor Fusion Techniques", "keywords": ["sensor fusion", "Kalman filter", "particle filter", "multi-modal"]},
            {"id": "15-slam-mapping", "title": "SLAM and Mapping", "keywords": ["SLAM", "localization", "mapping", "graph optimization"]},
            {"id": "16-path-planning-algorithms", "title": "Path Planning Algorithms", "keywords": ["A*", "RRT", "Dijkstra", "motion planning"]},
            {"id": "17-motion-control-systems", "title": "Motion Control Systems", "keywords": ["trajectory control", "inverse kinematics", "dynamics control"]},
            {"id": "18-machine-learning-perception", "title": "Machine Learning for Perception", "keywords": ["deep learning", "neural networks", "CNN", "object recognition"]},
            {"id": "19-real-time-decision-making", "title": "Real-Time Decision Making", "keywords": ["decision making", "behavior trees", "finite state machines"]},
        ]
    },
    "module-4": {
        "title": "Module 4: Humanoid AI Systems & Capstone Development",
        "chapters": [
            {"id": "20-humanoid-robotics-overview", "title": "Humanoid Robotics Overview", "keywords": ["humanoid robots", "bipedal systems", "anthropomorphic design"]},
            {"id": "21-bipedal-locomotion", "title": "Bipedal Locomotion", "keywords": ["walking", "balance", "ZMP", "gait generation"]},
            {"id": "22-human-robot-interaction", "title": "Human-Robot Interaction", "keywords": ["HRI", "natural language", "gestures", "social robotics"]},
            {"id": "23-manipulation-grasping", "title": "Manipulation and Grasping", "keywords": ["grasping", "manipulation", "dexterous hands", "force control"]},
            {"id": "24-integration-ai-systems", "title": "Integration of AI Systems", "keywords": ["AI integration", "multi-modal AI", "reinforcement learning"]},
            {"id": "25-capstone-project-guide", "title": "Capstone Project Guide", "keywords": ["capstone", "project planning", "system integration", "deployment"]},
        ]
    }
}


# ==========================================
# Chapter Generation System Prompt
# ==========================================

SYSTEM_PROMPT = """You are an expert robotics and AI educator creating a comprehensive, AI-native textbook on Physical AI and Humanoid Robotics.

Your chapters must be:
1. **Educational Excellence**: Clear, engaging, technically accurate with real-world examples
2. **Practical Focus**: Include code examples, simulation setups, and hands-on exercises
3. **Progressive Complexity**: Build on previous concepts, suitable for advanced undergraduates
4. **Industry-Relevant**: Reference current technologies (ROS 2, NVIDIA Isaac Sim, Gazebo)
5. **Well-Structured**: Use clear headings, bullet points, diagrams (as Mermaid), and summaries

Structure each chapter with:
- Introduction (motivation and learning objectives)
- Core Concepts (3-5 main sections with subsections)
- Practical Examples (code snippets in Python/ROS 2)
- Hands-On Exercises (simulation tasks, coding challenges)
- Summary and Key Takeaways
- Further Reading and Resources

Use Docusaurus-compatible Markdown with:
- Mermaid diagrams for flowcharts and architectures
- Code blocks with language tags (```python, ```bash, ```yaml)
- Admonitions (:::tip, :::note, :::warning, :::info)
- Math equations in LaTeX where appropriate

Target length: 2500-3500 words per chapter."""


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
        chapter: Chapter metadata dict with id, title, keywords
        module_title: Full module title

    Returns:
        Generated markdown content
    """
    chapter_id = chapter["id"]
    chapter_title = chapter["title"]
    keywords = ", ".join(chapter["keywords"])

    user_prompt = f"""Generate a comprehensive textbook chapter for:

**Module**: {module_title}
**Chapter**: {chapter_title}
**Focus Keywords**: {keywords}

This chapter should be suitable for advanced undergraduate students studying robotics and AI.
Include practical examples, code snippets, and exercises.
Use Docusaurus Markdown format with Mermaid diagrams where appropriate.

Generate the full chapter content now."""

    logger.info(f"Generating chapter: {module_id}/{chapter_id}")

    try:
        response = await openai.ChatCompletion.acreate(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=4096,
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
