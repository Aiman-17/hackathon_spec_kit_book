"""
Reproducible RAG Query Tests

Tests the chatbot with known queries to verify:
1. Grounded answers with citations
2. Proper handling of out-of-scope questions
3. Similarity scores and retrieval quality
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.qdrant_manager import qdrant_manager
from src.services.embedding_pipeline import EmbeddingPipelineService
from src.api.routers.chat import retrieve_relevant_chunks, generate_rag_response


# Test queries - mix of in-scope and out-of-scope
TEST_QUERIES = [
    {
        "query": "What is ROS 2?",
        "expected_scope": "in_scope",
        "expected_modules": ["Module 1", "Module 2"],
        "min_similarity": 0.6
    },
    {
        "query": "Explain SLAM in robotics",
        "expected_scope": "in_scope",
        "expected_modules": ["Module 3"],
        "min_similarity": 0.5
    },
    {
        "query": "What is URDF?",
        "expected_scope": "in_scope",
        "expected_modules": ["Module 2"],
        "min_similarity": 0.6
    },
    {
        "query": "How does computer vision work in robotics?",
        "expected_scope": "in_scope",
        "expected_modules": ["Module 3"],
        "min_similarity": 0.5
    },
    {
        "query": "What is humanoid locomotion?",
        "expected_scope": "in_scope",
        "expected_modules": ["Module 3", "Module 4"],
        "min_similarity": 0.5
    },
    {
        "query": "Explain underwater robotics",
        "expected_scope": "out_of_scope",
        "expected_modules": [],
        "min_similarity": 0.0
    },
    {
        "query": "What is the weather today?",
        "expected_scope": "out_of_scope",
        "expected_modules": [],
        "min_similarity": 0.0
    }
]


async def run_test_query(query_data: dict, verbose: bool = True):
    """
    Run a single test query and return results.

    Args:
        query_data: Dictionary with query and expected results
        verbose: Whether to print detailed output

    Returns:
        Dictionary with test results
    """
    query = query_data["query"]
    expected_scope = query_data["expected_scope"]
    min_similarity = query_data["min_similarity"]

    if verbose:
        print(f"\n{'='*80}")
        print(f"Query: {query}")
        print(f"Expected: {expected_scope}")
        print(f"Min Similarity: {min_similarity}")
        print(f"{'='*80}")

    try:
        # Retrieve relevant chunks
        chunks = await retrieve_relevant_chunks(query, max_results=3)

        # Analyze results
        num_results = len(chunks)
        avg_similarity = sum(c.get("score", 0.0) for c in chunks) / num_results if num_results > 0 else 0.0
        max_similarity = max((c.get("score", 0.0) for c in chunks), default=0.0)

        # Check if results match expectations
        is_in_scope = num_results > 0 and max_similarity >= min_similarity
        scope_match = (is_in_scope and expected_scope == "in_scope") or (not is_in_scope and expected_scope == "out_of_scope")

        if verbose:
            print(f"\nResults:")
            print(f"  - Retrieved chunks: {num_results}")
            print(f"  - Max similarity: {max_similarity:.3f}")
            print(f"  - Avg similarity: {avg_similarity:.3f}")
            print(f"  - Scope match: {'✅' if scope_match else '❌'}")

            if num_results > 0:
                print(f"\nTop results:")
                for i, chunk in enumerate(chunks[:3], 1):
                    payload = chunk.get("payload", {})
                    score = chunk.get("score", 0.0)
                    chapter = payload.get("chapter_title", "Unknown")
                    section = payload.get("section_title", "Unknown Section")
                    content_preview = payload.get("content", "")[:100] + "..."

                    print(f"\n  [{i}] Similarity: {score:.3f}")
                    print(f"      Chapter: {chapter}")
                    print(f"      Section: {section}")
                    print(f"      Preview: {content_preview}")

                # Generate answer
                print(f"\n{'='*80}")
                print("Generated Answer:")
                print(f"{'='*80}")
                answer = await generate_rag_response(query, chunks)
                print(answer)
            else:
                print("\n  No relevant chunks found (expected for out-of-scope queries)")

        return {
            "query": query,
            "expected_scope": expected_scope,
            "num_results": num_results,
            "max_similarity": max_similarity,
            "avg_similarity": avg_similarity,
            "scope_match": scope_match,
            "passed": scope_match
        }

    except Exception as e:
        print(f"\n❌ Error: {str(e)}")
        return {
            "query": query,
            "expected_scope": expected_scope,
            "passed": False,
            "error": str(e)
        }


async def run_all_tests():
    """
    Run all test queries and generate a summary report.
    """
    print("\n" + "="*80)
    print("RAG CHATBOT TEST SUITE")
    print("="*80)

    # Initialize services
    print("\nInitializing services...")
    try:
        qdrant_manager.initialize()
        print("✅ Qdrant initialized")
    except Exception as e:
        print(f"❌ Failed to initialize Qdrant: {e}")
        return

    # Run all tests
    results = []
    for query_data in TEST_QUERIES:
        result = await run_test_query(query_data, verbose=True)
        results.append(result)
        await asyncio.sleep(0.5)  # Small delay between requests

    # Generate summary
    print("\n" + "="*80)
    print("TEST SUMMARY")
    print("="*80)

    total_tests = len(results)
    passed_tests = sum(1 for r in results if r.get("passed", False))
    failed_tests = total_tests - passed_tests

    print(f"\nTotal Tests: {total_tests}")
    print(f"Passed: {passed_tests} ✅")
    print(f"Failed: {failed_tests} ❌")
    print(f"Success Rate: {(passed_tests/total_tests)*100:.1f}%")

    # Detailed results
    print("\nDetailed Results:")
    for i, result in enumerate(results, 1):
        status = "✅" if result.get("passed", False) else "❌"
        query = result["query"]
        expected = result["expected_scope"]
        similarity = result.get("max_similarity", 0.0)

        print(f"{i}. {status} {query[:50]}")
        print(f"   Expected: {expected}, Similarity: {similarity:.3f}")

    # Cleanup
    qdrant_manager.close()

    print("\n" + "="*80)
    print("Tests completed!")
    print("="*80 + "\n")


if __name__ == "__main__":
    # Run tests
    asyncio.run(run_all_tests())
