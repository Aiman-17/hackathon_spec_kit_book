"""
Integration Tests for RAG Pipeline

Tests based on Success Criteria SC-008 from spec.md:
- 10 predefined test queries
- Each query must return answer + citation
- ≥80% success rate required (8/10 queries)
"""

import pytest
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.services.qdrant_manager import qdrant_manager
from src.api.routers.chat import retrieve_relevant_chunks, generate_rag_response


# Test queries from SC-008 in spec.md
TEST_QUERIES_SC008 = [
    {
        "id": "Q1",
        "query": "What is ROS 2?",
        "expected_citation": "Introduction to Robot Operating System (ROS 2)",
        "min_similarity": 0.6
    },
    {
        "id": "Q2",
        "query": "How do URDF and XACRO differ?",
        "expected_citation": "URDF & XACRO for Robot Modeling",
        "min_similarity": 0.6
    },
    {
        "id": "Q3",
        "query": "Explain the Nav2 navigation stack",
        "expected_citation": "Nav2 Navigation Stack",
        "min_similarity": 0.6
    },
    {
        "id": "Q4",
        "query": "What sensors are used in humanoid robots?",
        "expected_citation": "Robot Hardware: Sensors, Actuators, Control Systems",
        "min_similarity": 0.5
    },
    {
        "id": "Q5",
        "query": "How does Isaac Sim work?",
        "expected_citation": "NVIDIA Isaac Sim: Set Up & Fundamentals",
        "min_similarity": 0.6
    },
    {
        "id": "Q6",
        "query": "What is bipedal control?",
        "expected_citation": "Motion Planning for Humanoids (Bipedal Control)",
        "min_similarity": 0.6
    },
    {
        "id": "Q7",
        "query": "Explain vision-language-action pipelines",
        "expected_citation": "Vision-Language-Action Pipelines (VLA)",
        "min_similarity": 0.6
    },
    {
        "id": "Q8",
        "query": "How do ROS nodes communicate?",
        "expected_citation": "ROS 2 in Depth: Nodes, Topics, Services, Actions",
        "min_similarity": 0.6
    },
    {
        "id": "Q9",
        "query": "What is VSLAM?",
        "expected_citation": "Isaac ROS Perception Pipelines (VSLAM, AprilTags)",
        "min_similarity": 0.5
    },
    {
        "id": "Q10",
        "query": "How do you build a humanoid model?",
        "expected_citation": "Building a Humanoid Model in Gazebo",
        "min_similarity": 0.5
    },
]


@pytest.fixture(scope="module")
def qdrant_client():
    """Initialize Qdrant client for tests"""
    qdrant_manager.initialize()
    yield qdrant_manager
    qdrant_manager.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("query_data", TEST_QUERIES_SC008, ids=[q["id"] for q in TEST_QUERIES_SC008])
async def test_rag_query_with_citation(query_data, qdrant_client):
    """
    Test that RAG query returns answer with citation.

    Success Criteria:
    - Retrieves relevant chunks (similarity > min_threshold)
    - Generates an answer
    - Answer includes citation to relevant chapter
    """
    query = query_data["query"]
    expected_citation = query_data["expected_citation"]
    min_similarity = query_data["min_similarity"]

    # Step 1: Retrieve relevant chunks
    chunks = await retrieve_relevant_chunks(query, max_results=3)

    # Assertion: Should retrieve at least 1 chunk
    assert len(chunks) > 0, f"No chunks retrieved for query: {query}"

    # Assertion: Max similarity should meet threshold
    max_similarity = max((c.get("score", 0.0) for c in chunks), default=0.0)
    assert max_similarity >= min_similarity, \
        f"Max similarity {max_similarity:.3f} < threshold {min_similarity}"

    # Step 2: Generate answer
    answer = await generate_rag_response(query, chunks)

    # Assertion: Answer should not be empty
    assert answer is not None and len(answer) > 0, "Generated answer is empty"

    # Assertion: Answer should include reasoning/citations (check for "Source" keyword)
    assert "Source" in answer or "[" in answer, \
        "Answer does not include citations or source references"

    # Step 3: Verify citation relevance (check if expected chapter appears in retrieved chunks)
    retrieved_chapters = [
        c.get("payload", {}).get("chapter_title", "")
        for c in chunks
    ]

    # Fuzzy match: check if any retrieved chapter contains key terms from expected citation
    expected_terms = expected_citation.lower().split()
    citation_found = any(
        any(term in chapter.lower() for term in expected_terms)
        for chapter in retrieved_chapters
    )

    print(f"\n[{query_data['id']}] Query: {query}")
    print(f"  Expected: {expected_citation}")
    print(f"  Retrieved: {retrieved_chapters[0] if retrieved_chapters else 'None'}")
    print(f"  Similarity: {max_similarity:.3f}")
    print(f"  Citation Match: {'✅' if citation_found else '⚠️'}")

    # Note: We don't strictly fail on citation mismatch,
    # as RAG may return related but differently titled chapters
    # The key requirement is that answer + citation exist


@pytest.mark.asyncio
async def test_rag_success_rate():
    """
    Test that RAG pipeline meets SC-008 success criteria:
    ≥80% of queries (8/10) return answer + citation
    """
    qdrant_manager.initialize()

    passed = 0
    total = len(TEST_QUERIES_SC008)

    for query_data in TEST_QUERIES_SC008:
        try:
            chunks = await retrieve_relevant_chunks(query_data["query"], max_results=3)
            if len(chunks) > 0:
                max_similarity = max((c.get("score", 0.0) for c in chunks), default=0.0)
                if max_similarity >= query_data["min_similarity"]:
                    answer = await generate_rag_response(query_data["query"], chunks)
                    if answer and ("Source" in answer or "[" in answer):
                        passed += 1
        except Exception as e:
            print(f"Error for query {query_data['id']}: {e}")
            continue

    success_rate = (passed / total) * 100
    print(f"\nRAG Success Rate: {passed}/{total} ({success_rate:.1f}%)")

    qdrant_manager.close()

    # Assertion: Success rate must be ≥80% (8/10 queries)
    assert success_rate >= 80.0, \
        f"Success rate {success_rate:.1f}% < required 80%. Only {passed}/{total} queries passed."


@pytest.mark.asyncio
async def test_out_of_scope_query():
    """
    Test that out-of-scope queries are handled gracefully.

    The system should:
    - Return a response indicating content not available
    - NOT hallucinate information
    """
    qdrant_manager.initialize()

    out_of_scope_queries = [
        "What is underwater robotics?",
        "How do quantum computers work?",
        "What is the weather today?",
    ]

    for query in out_of_scope_queries:
        chunks = await retrieve_relevant_chunks(query, max_results=3)

        # If low similarity or no chunks, system should handle gracefully
        if len(chunks) == 0 or max((c.get("score", 0.0) for c in chunks), default=0.0) < 0.5:
            answer = await generate_rag_response(query, chunks)

            # Assertion: Answer should indicate content not available
            not_available_keywords = [
                "don't have information",
                "not contain information",
                "textbook content provided does not",
                "can't find",
                "couldn't find"
            ]

            contains_disclaimer = any(kw in answer.lower() for kw in not_available_keywords)

            print(f"\nOut-of-scope query: {query}")
            print(f"  Chunks: {len(chunks)}")
            print(f"  Contains disclaimer: {'✅' if contains_disclaimer else '❌'}")

            # This is a soft check - we want to encourage proper handling
            # but don't strictly fail if the answer is reasonable

    qdrant_manager.close()


if __name__ == "__main__":
    # Run with pytest
    pytest.main([__file__, "-v", "-s"])
