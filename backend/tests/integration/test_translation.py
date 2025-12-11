"""
Integration Tests for Translation Service

Tests based on Success Criteria SC-012 from spec.md:
- Urdu translation produces valid output for ≥95% of chapters
- Markdown structure is preserved
- Translation completes within 10 seconds
"""

import pytest
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.services.translation_service import translation_service


# Sample test content with varying complexity
TEST_CONTENT = {
    "simple": """## Introduction

This is a simple chapter about robotics.

Robotics is the study of robots.""",

    "with_code": """## ROS 2 Nodes

ROS 2 nodes are processes that perform computation.

```python
# Create a node
import rclpy
node = rclpy.create_node('my_node')
```

Each node can communicate with other nodes.""",

    "with_lists": """## Hardware Components

Key hardware components include:

- **Sensors**: Cameras, LIDAR, IMU
- **Actuators**: Motors, servos
- **Controllers**: Microcontrollers, embedded systems

1. First, install the sensors
2. Then, connect the actuators
3. Finally, configure the controller""",

    "with_links": """## Resources

Learn more about robotics:

- [ROS 2 Documentation](https://docs.ros.org)
- [Gazebo Tutorials](https://gazebosim.org)

Visit the official website for more information.""",
}


@pytest.mark.asyncio
@pytest.mark.parametrize("content_type", ["simple", "with_code", "with_lists", "with_links"])
async def test_translation_preserves_markdown(content_type):
    """
    Test that Urdu translation preserves Markdown structure.

    Success Criteria:
    - Valid UTF-8 Urdu text
    - Markdown structure preserved (headers, code blocks, lists, links)
    - Translation completes within 10 seconds
    """
    content = TEST_CONTENT[content_type]

    # Measure translation time
    start_time = time.time()
    translated_text = await translation_service.translate_to_urdu(content)
    elapsed_time = time.time() - start_time

    # Assertion: Translation should complete within 10 seconds
    assert elapsed_time < 10.0, f"Translation took {elapsed_time:.2f}s > 10s limit"

    # Assertion: Translation should not be empty
    assert translated_text is not None and len(translated_text) > 0, \
        "Translation is empty"

    # Assertion: Should contain Urdu characters (Unicode range)
    has_urdu = any('\u0600' <= char <= '\u06FF' for char in translated_text)
    assert has_urdu, "Translation does not contain Urdu script"

    # Check structure preservation based on content type
    if content_type == "with_code":
        # Assertion: Code blocks should be preserved
        assert "```python" in translated_text, "Python code block marker missing"
        assert "import rclpy" in translated_text, "Code content not preserved"

    if content_type == "with_lists":
        # Assertion: Lists should be preserved
        assert "- " in translated_text or "• " in translated_text, "Unordered list markers missing"
        assert "1. " in translated_text or "1." in translated_text, "Ordered list markers missing"

    if content_type == "with_links":
        # Assertion: Links should be preserved
        assert "[" in translated_text and "](" in translated_text, "Link syntax missing"
        assert "https://docs.ros.org" in translated_text, "URLs not preserved"

    # Assertion: Headers should be preserved
    assert "##" in translated_text, "Header markers (##) missing"

    print(f"\n[{content_type}] Translation Test:")
    print(f"  Time: {elapsed_time:.2f}s")
    print(f"  Length: {len(translated_text)} chars")
    print(f"  Has Urdu: {'✅' if has_urdu else '❌'}")
    print(f"  Structure: ✅")


@pytest.mark.asyncio
async def test_translation_caching():
    """
    Test that translation caching works correctly.

    Second request for same content should be faster (cache hit).
    """
    content = TEST_CONTENT["simple"]

    # First translation (cache miss)
    start_time = time.time()
    result1 = await translation_service.translate_to_urdu(content)
    time1 = time.time() - start_time

    # Second translation (should be cache hit)
    start_time = time.time()
    result2 = await translation_service.translate_to_urdu(content)
    time2 = time.time() - start_time

    # Assertions
    assert result1 == result2, "Cached result differs from original"
    assert time2 < time1, f"Cache hit ({time2:.2f}s) not faster than miss ({time1:.2f}s)"

    print(f"\nCaching Test:")
    print(f"  First call (miss): {time1:.2f}s")
    print(f"  Second call (hit): {time2:.2f}s")
    print(f"  Speedup: {time1/time2:.1f}x")


@pytest.mark.asyncio
async def test_translation_chapter():
    """
    Test translating a full chapter with multiple sections.
    """
    chapter_content = """## Module 1: Foundations

### Introduction

Physical AI combines physical systems with AI.

### Key Concepts

Important concepts include:
- Robotics fundamentals
- Sensor integration
- Control systems

```python
# Example code
def control_robot():
    pass
```

### Summary

This chapter introduced the foundations of Physical AI."""

    chapter_id = "01-foundations"

    result = await translation_service.translate_chapter(chapter_content, chapter_id)

    # Assertions
    assert result["chapter_id"] == chapter_id
    assert len(result["translated_content"]) > 0
    assert result["sections_count"] > 0
    assert "##" in result["translated_content"], "Section headers missing"

    print(f"\nChapter Translation Test:")
    print(f"  Chapter: {chapter_id}")
    print(f"  Sections: {result['sections_count']}")
    print(f"  Length: {result['original_length']} → {result['translated_length']}")
    print(f"  Cache hits: {result['cache_hits']}")
    print(f"  Cache misses: {result['cache_misses']}")


@pytest.mark.asyncio
async def test_translation_stats():
    """
    Test that translation service tracks statistics correctly.
    """
    # Perform some translations
    await translation_service.translate_to_urdu(TEST_CONTENT["simple"])

    # Get stats
    stats = translation_service.get_stats()

    # Assertions
    assert "total_translations" in stats
    assert "cache_hits" in stats
    assert "cache_misses" in stats
    assert "cache_hit_rate" in stats
    assert stats["total_requests"] >= 0

    print(f"\nTranslation Stats:")
    print(f"  Total translations: {stats['total_translations']}")
    print(f"  Cache hits: {stats['cache_hits']}")
    print(f"  Cache misses: {stats['cache_misses']}")
    print(f"  Hit rate: {stats['cache_hit_rate']:.1f}%")


if __name__ == "__main__":
    # Run with pytest
    pytest.main([__file__, "-v", "-s"])
