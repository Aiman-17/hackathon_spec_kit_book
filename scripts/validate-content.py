#!/usr/bin/env python3
"""
Content Quality Validation Script

Validates textbook chapter quality against 15 criteria:
1. Proper markdown structure
2. Front matter present
3. Minimum word count (2000 words)
4. Headings hierarchy (H2-H4)
5. Code blocks with language tags
6. Practical examples present
7. Exercises/activities included
8. Summary section exists
9. Learning objectives stated
10. Technical accuracy (keyword validation)
11. Links validity
12. Image references valid
13. Admonitions used appropriately
14. Mermaid diagrams for complex concepts
15. Consistent terminology

Usage:
    python scripts/validate-content.py --file frontend/docs/module-1/01-intro.md
    python scripts/validate-content.py --all
    python scripts/validate-content.py --module 1
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ==========================================
# Configuration
# ==========================================

REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"

# Validation thresholds
MIN_WORD_COUNT = 2000
MAX_WORD_COUNT = 5000
MIN_CODE_BLOCKS = 2
MIN_HEADINGS = 5
MIN_EXERCISES = 1


# Required keywords by module (basic technical accuracy check)
MODULE_KEYWORDS = {
    "module-1": ["robot", "ai", "sensor", "actuator", "control"],
    "module-2": ["ros", "simulation", "gazebo", "isaac", "node"],
    "module-3": ["perception", "navigation", "slam", "vision", "planning"],
    "module-4": ["humanoid", "locomotion", "manipulation", "interaction", "grasping"]
}


# ==========================================
# Validation Criteria Functions
# ==========================================

class ContentValidator:
    """Validates textbook chapter content against quality criteria"""

    def __init__(self, file_path: Path):
        self.file_path = file_path
        self.content = ""
        self.lines = []
        self.results = {}
        self.score = 0
        self.max_score = 15

        # Read file content
        if file_path.exists():
            with open(file_path, "r", encoding="utf-8") as f:
                self.content = f.read()
                self.lines = self.content.split("\n")
        else:
            logger.error(f"File not found: {file_path}")

    def criterion_1_markdown_structure(self) -> Tuple[bool, str]:
        """1. Proper markdown structure"""
        if not self.content:
            return False, "File is empty or not found"

        # Check for basic markdown elements
        has_headings = bool(re.search(r'^#{1,6}\s+.+', self.content, re.MULTILINE))
        has_paragraphs = len(self.content.split("\n\n")) > 3

        if has_headings and has_paragraphs:
            return True, "Valid markdown structure"
        return False, "Missing headings or paragraphs"

    def criterion_2_front_matter(self) -> Tuple[bool, str]:
        """2. Front matter present (Docusaurus format)"""
        # Front matter is optional for Docusaurus, but good to have
        has_front_matter = self.content.startswith("---")

        if has_front_matter:
            return True, "Front matter present"
        return False, "No front matter (optional but recommended)"

    def criterion_3_word_count(self) -> Tuple[bool, str]:
        """3. Minimum word count (2000 words)"""
        # Remove code blocks for accurate word count
        text_only = re.sub(r'```.*?```', '', self.content, flags=re.DOTALL)
        words = text_only.split()
        word_count = len(words)

        if MIN_WORD_COUNT <= word_count <= MAX_WORD_COUNT:
            return True, f"Word count: {word_count} (optimal range)"
        elif word_count < MIN_WORD_COUNT:
            return False, f"Word count: {word_count} (below minimum {MIN_WORD_COUNT})"
        else:
            return False, f"Word count: {word_count} (above maximum {MAX_WORD_COUNT})"

    def criterion_4_headings_hierarchy(self) -> Tuple[bool, str]:
        """4. Headings hierarchy (H2-H4)"""
        headings = re.findall(r'^(#{2,4})\s+(.+)$', self.content, re.MULTILINE)

        if len(headings) >= MIN_HEADINGS:
            h2_count = sum(1 for h in headings if h[0] == "##")
            h3_count = sum(1 for h in headings if h[0] == "###")

            if h2_count >= 3:  # At least 3 main sections
                return True, f"Good heading structure: {len(headings)} headings"
            return False, f"Too few H2 headings: {h2_count}"

        return False, f"Insufficient headings: {len(headings)} (need {MIN_HEADINGS})"

    def criterion_5_code_blocks(self) -> Tuple[bool, str]:
        """5. Code blocks with language tags"""
        code_blocks = re.findall(r'```(\w+)\n', self.content)

        if len(code_blocks) >= MIN_CODE_BLOCKS:
            languages = set(code_blocks)
            return True, f"Code blocks: {len(code_blocks)} ({', '.join(languages)})"
        return False, f"Too few code blocks: {len(code_blocks)} (need {MIN_CODE_BLOCKS})"

    def criterion_6_practical_examples(self) -> Tuple[bool, str]:
        """6. Practical examples present"""
        example_keywords = [
            "example", "demo", "tutorial", "walkthrough",
            "implementation", "practice", "hands-on"
        ]

        example_count = sum(
            self.content.lower().count(keyword)
            for keyword in example_keywords
        )

        if example_count >= 3:
            return True, f"Practical examples found: {example_count} references"
        return False, f"Insufficient examples: {example_count}"

    def criterion_7_exercises(self) -> Tuple[bool, str]:
        """7. Exercises/activities included"""
        exercise_patterns = [
            r'##+ (Exercise|Activity|Challenge|Task)',
            r':::tip.*?(exercise|try|practice)',
            r'\*\*Exercise\*\*',
        ]

        exercise_count = sum(
            len(re.findall(pattern, self.content, re.IGNORECASE | re.DOTALL))
            for pattern in exercise_patterns
        )

        if exercise_count >= MIN_EXERCISES:
            return True, f"Exercises found: {exercise_count}"
        return False, f"No exercises found (need {MIN_EXERCISES})"

    def criterion_8_summary_section(self) -> Tuple[bool, str]:
        """8. Summary section exists"""
        has_summary = bool(re.search(
            r'##+ (Summary|Conclusion|Key Takeaways|Recap)',
            self.content,
            re.IGNORECASE
        ))

        if has_summary:
            return True, "Summary section found"
        return False, "No summary section"

    def criterion_9_learning_objectives(self) -> Tuple[bool, str]:
        """9. Learning objectives stated"""
        has_objectives = bool(re.search(
            r'(Learning Objectives|Learning Goals|In this chapter|You will learn)',
            self.content,
            re.IGNORECASE
        ))

        if has_objectives:
            return True, "Learning objectives stated"
        return False, "No learning objectives"

    def criterion_10_technical_accuracy(self) -> Tuple[bool, str]:
        """10. Technical accuracy (keyword validation)"""
        # Determine module from file path
        module_match = re.search(r'module-(\d+)', str(self.file_path))
        if not module_match:
            return False, "Cannot determine module number"

        module_id = f"module-{module_match.group(1)}"
        required_keywords = MODULE_KEYWORDS.get(module_id, [])

        content_lower = self.content.lower()
        found_keywords = [kw for kw in required_keywords if kw in content_lower]

        coverage = len(found_keywords) / len(required_keywords) if required_keywords else 0

        if coverage >= 0.6:  # 60% keyword coverage
            return True, f"Keyword coverage: {coverage:.0%} ({len(found_keywords)}/{len(required_keywords)})"
        return False, f"Low keyword coverage: {coverage:.0%}"

    def criterion_11_links_validity(self) -> Tuple[bool, str]:
        """11. Links validity (basic format check)"""
        # Check for markdown links
        links = re.findall(r'\[([^\]]+)\]\(([^)]+)\)', self.content)

        if not links:
            return False, "No links found"

        # Check for broken internal links (basic check)
        broken_links = [url for text, url in links if url.startswith("#") and url.count("#") > 1]

        if broken_links:
            return False, f"Potentially broken links: {len(broken_links)}"

        return True, f"Links found: {len(links)}"

    def criterion_12_image_references(self) -> Tuple[bool, str]:
        """12. Image references valid"""
        # Check for image references
        images = re.findall(r'!\[([^\]]*)\]\(([^)]+)\)', self.content)

        if images:
            # Basic validation - images should have alt text and valid paths
            no_alt_text = sum(1 for alt, url in images if not alt.strip())
            if no_alt_text > 0:
                return False, f"Images without alt text: {no_alt_text}/{len(images)}"
            return True, f"Images found: {len(images)} (all with alt text)"

        return True, "No images (acceptable)"

    def criterion_13_admonitions(self) -> Tuple[bool, str]:
        """13. Admonitions used appropriately"""
        admonitions = re.findall(r':::(tip|note|warning|info|caution)', self.content)

        if len(admonitions) >= 2:
            types = set(admonitions)
            return True, f"Admonitions: {len(admonitions)} ({', '.join(types)})"
        return False, f"Too few admonitions: {len(admonitions)} (recommend 2+)"

    def criterion_14_mermaid_diagrams(self) -> Tuple[bool, str]:
        """14. Mermaid diagrams for complex concepts"""
        mermaid_blocks = re.findall(r'```mermaid\n', self.content)

        if mermaid_blocks:
            return True, f"Mermaid diagrams: {len(mermaid_blocks)}"
        return False, "No Mermaid diagrams (recommended for complex concepts)"

    def criterion_15_consistent_terminology(self) -> Tuple[bool, str]:
        """15. Consistent terminology"""
        # Check for common inconsistencies
        inconsistencies = []

        # ROS vs ROS2 vs ROS 2
        has_ros = "ros" in self.content.lower()
        if has_ros:
            ros_variants = [self.content.count(v) for v in ["ROS", "ROS2", "ROS 2"]]
            if len([v for v in ros_variants if v > 0]) > 1:
                # Multiple variants used - check if one is dominant
                max_variant = max(ros_variants)
                if max_variant < sum(ros_variants) * 0.8:
                    inconsistencies.append("ROS/ROS2/ROS 2")

        # AI vs A.I. vs ai
        ai_variants = [self.content.count(v) for v in ["AI", "A.I."]]
        if len([v for v in ai_variants if v > 0]) > 1:
            inconsistencies.append("AI/A.I.")

        if inconsistencies:
            return False, f"Terminology inconsistencies: {', '.join(inconsistencies)}"
        return True, "Consistent terminology"

    def validate_all(self) -> Dict:
        """Run all validation criteria and return results"""
        criteria = [
            ("Markdown Structure", self.criterion_1_markdown_structure),
            ("Front Matter", self.criterion_2_front_matter),
            ("Word Count", self.criterion_3_word_count),
            ("Headings Hierarchy", self.criterion_4_headings_hierarchy),
            ("Code Blocks", self.criterion_5_code_blocks),
            ("Practical Examples", self.criterion_6_practical_examples),
            ("Exercises", self.criterion_7_exercises),
            ("Summary Section", self.criterion_8_summary_section),
            ("Learning Objectives", self.criterion_9_learning_objectives),
            ("Technical Accuracy", self.criterion_10_technical_accuracy),
            ("Links Validity", self.criterion_11_links_validity),
            ("Image References", self.criterion_12_image_references),
            ("Admonitions", self.criterion_13_admonitions),
            ("Mermaid Diagrams", self.criterion_14_mermaid_diagrams),
            ("Consistent Terminology", self.criterion_15_consistent_terminology),
        ]

        results = {}
        passed = 0

        logger.info(f"\n{'='*60}")
        logger.info(f"Validating: {self.file_path.name}")
        logger.info(f"{'='*60}\n")

        for name, criterion_func in criteria:
            try:
                success, message = criterion_func()
                results[name] = {"passed": success, "message": message}

                status = "✅ PASS" if success else "❌ FAIL"
                logger.info(f"{status} - {name}: {message}")

                if success:
                    passed += 1

            except Exception as e:
                logger.error(f"Error in {name}: {str(e)}")
                results[name] = {"passed": False, "message": f"Error: {str(e)}"}

        self.results = results
        self.score = passed

        # Final score
        percentage = (passed / self.max_score) * 100
        logger.info(f"\n{'='*60}")
        logger.info(f"Score: {passed}/{self.max_score} ({percentage:.1f}%)")

        if percentage >= 80:
            logger.info("Grade: EXCELLENT ✨")
        elif percentage >= 60:
            logger.info("Grade: GOOD ✓")
        elif percentage >= 40:
            logger.info("Grade: NEEDS IMPROVEMENT ⚠️")
        else:
            logger.info("Grade: POOR ❌")

        logger.info(f"{'='*60}\n")

        return results


# ==========================================
# Batch Validation
# ==========================================

def validate_module(module_number: int):
    """Validate all chapters in a module"""
    module_dir = DOCS_DIR / f"module-{module_number}"

    if not module_dir.exists():
        logger.error(f"Module directory not found: {module_dir}")
        return

    chapter_files = sorted(module_dir.glob("*.md"))

    if not chapter_files:
        logger.warning(f"No markdown files found in {module_dir}")
        return

    logger.info(f"\n📚 Validating Module {module_number} ({len(chapter_files)} chapters)\n")

    total_score = 0
    max_total_score = 0

    for chapter_file in chapter_files:
        validator = ContentValidator(chapter_file)
        validator.validate_all()

        total_score += validator.score
        max_total_score += validator.max_score

    # Module summary
    avg_percentage = (total_score / max_total_score * 100) if max_total_score > 0 else 0
    logger.info(f"\n{'='*60}")
    logger.info(f"Module {module_number} Summary")
    logger.info(f"Total Score: {total_score}/{max_total_score} ({avg_percentage:.1f}%)")
    logger.info(f"{'='*60}\n")


def validate_all():
    """Validate all chapters in all modules"""
    logger.info("\n🚀 Validating ALL chapters\n")

    for module_number in range(1, 5):  # Modules 1-4
        validate_module(module_number)


# ==========================================
# CLI Interface
# ==========================================

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Validate textbook chapter quality"
    )
    parser.add_argument(
        "--file",
        type=Path,
        help="Path to specific markdown file"
    )
    parser.add_argument(
        "--module",
        type=int,
        choices=[1, 2, 3, 4],
        help="Validate all chapters in module"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Validate all chapters"
    )

    args = parser.parse_args()

    if args.file:
        validator = ContentValidator(args.file)
        validator.validate_all()
    elif args.module:
        validate_module(args.module)
    elif args.all:
        validate_all()
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
