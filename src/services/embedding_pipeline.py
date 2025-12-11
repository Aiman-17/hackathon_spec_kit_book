"""
Embedding Pipeline Service

Generates embeddings using OpenAI text-embedding-3-small model.
Handles batching, rate limiting, and caching for optimal performance.
"""

import asyncio
import logging
from typing import List, Dict, Any
from openai import AsyncOpenAI

from src.config import settings

logger = logging.getLogger(__name__)


# ==========================================
# Embedding Pipeline
# ==========================================

class EmbeddingPipeline:
    """
    Manages embedding generation using OpenAI API.

    Features:
    - Batch processing for efficiency
    - Rate limiting to respect API limits
    - Error handling and retries
    - Cost tracking
    """

    # Model configuration
    MODEL = "text-embedding-3-small"
    EMBEDDING_SIZE = 1536
    MAX_BATCH_SIZE = 100  # OpenAI allows up to 2048, but we use smaller batches
    MAX_TOKENS_PER_REQUEST = 8000  # Conservative limit

    def __init__(self):
        # Use OpenRouter's API endpoint (compatible with OpenAI SDK)
        self.client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        self.total_tokens_used = 0
        self.total_requests = 0

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text to embed

        Returns:
            1536-dimensional embedding vector
        """
        try:
            response = await self.client.embeddings.create(
                model=self.MODEL,
                input=text,
                dimensions=self.EMBEDDING_SIZE
            )

            embedding = response.data[0].embedding
            self.total_tokens_used += response.usage.total_tokens
            self.total_requests += 1

            logger.debug(f"Generated embedding ({len(embedding)} dims, {response.usage.total_tokens} tokens)")

            return embedding

        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}")
            raise

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        batch_size: int = MAX_BATCH_SIZE
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batches.

        Args:
            texts: List of input texts
            batch_size: Number of texts per API request

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        all_embeddings = []
        total_batches = (len(texts) + batch_size - 1) // batch_size

        logger.info(f"Generating embeddings for {len(texts)} texts in {total_batches} batches")

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_num = i // batch_size + 1

            logger.debug(f"Processing batch {batch_num}/{total_batches} ({len(batch)} texts)")

            try:
                response = await self.client.embeddings.create(
                    model=self.MODEL,
                    input=batch,
                    dimensions=self.EMBEDDING_SIZE
                )

                # Extract embeddings in order
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

                # Track usage
                self.total_tokens_used += response.usage.total_tokens
                self.total_requests += 1

                logger.debug(f"Batch {batch_num} complete: {response.usage.total_tokens} tokens")

                # Rate limiting - wait between batches
                if i + batch_size < len(texts):
                    await asyncio.sleep(0.5)  # 500ms between batches

            except Exception as e:
                logger.error(f"Failed to process batch {batch_num}: {str(e)}")
                raise

        logger.info(f"✅ Generated {len(all_embeddings)} embeddings ({self.total_tokens_used} tokens)")

        return all_embeddings

    async def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a search query.

        This is a convenience method optimized for single queries.

        Args:
            query: Search query text

        Returns:
            1536-dimensional embedding vector
        """
        return await self.generate_embedding(query)

    def get_usage_stats(self) -> Dict[str, Any]:
        """
        Get usage statistics for cost tracking.

        Returns:
            dict with usage stats
        """
        # Estimate cost (text-embedding-3-small pricing)
        # $0.020 per 1M tokens
        cost_per_1k_tokens = 0.00002
        estimated_cost = (self.total_tokens_used / 1000) * cost_per_1k_tokens

        return {
            "total_requests": self.total_requests,
            "total_tokens": self.total_tokens_used,
            "estimated_cost_usd": round(estimated_cost, 4),
            "model": self.MODEL,
            "embedding_size": self.EMBEDDING_SIZE,
        }

    def reset_stats(self):
        """Reset usage statistics"""
        self.total_tokens_used = 0
        self.total_requests = 0

    async def embed_and_store_chunks(self, chunks: List[Dict[str, Any]]) -> int:
        """
        Generate embeddings for chunks and store them in Qdrant.

        Args:
            chunks: List of chunk dictionaries with content and metadata

        Returns:
            Number of vectors stored
        """
        from qdrant_client.models import PointStruct
        from src.services.qdrant_manager import qdrant_manager

        if not chunks:
            return 0

        try:
            # Extract text content from chunks (support both dict and Pydantic objects)
            texts = []
            for chunk in chunks:
                if isinstance(chunk, dict):
                    texts.append(chunk["content"])
                else:
                    texts.append(chunk.content)

            # Generate embeddings in batch
            embeddings = await self.generate_embeddings_batch(texts)

            # Create PointStruct objects for Qdrant
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                # Extract chunk data (support both dict and Pydantic objects)
                if isinstance(chunk, dict):
                    content = chunk["content"]
                    chunk_id = chunk.get("chunk_id", f"chunk_{i}")
                    chapter_id = chunk.get("chapter_id", "unknown")
                    chapter_title = chunk.get("chapter_title", "Unknown")
                    section_title = chunk.get("section_title", "Unknown Section")
                    module_name = chunk.get("module_name", "Unknown Module")
                    module_id = chunk.get("module_id", "unknown")
                else:
                    content = chunk.content
                    chunk_id = f"{chunk.chapter_metadata.chapter_id}_chunk_{i}"
                    chapter_id = chunk.chapter_metadata.chapter_id
                    chapter_title = chunk.chapter_metadata.chapter_title
                    section_title = chunk.chunk_metadata.heading
                    module_name = chunk.chapter_metadata.module_title
                    module_id = chunk.chapter_metadata.module_id

                # Use chunk_id as point ID (hash it to integer)
                point_id = hash(chunk_id) % (2**63)

                # Create point with embedding and metadata
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "chunk_id": chunk_id,
                        "chapter_id": chapter_id,
                        "chapter_title": chapter_title,
                        "section_title": section_title,
                        "module_name": module_name,
                        "module_id": module_id,
                        "chunk_type": "content",
                        "position": i,
                    }
                )
                points.append(point)

            # Store in Qdrant
            qdrant_manager.upsert_points(points)

            logger.info(f"✅ Stored {len(points)} vectors in Qdrant")

            return len(points)

        except Exception as e:
            logger.error(f"Failed to embed and store chunks: {str(e)}")
            raise


# ==========================================
# Global Instance
# ==========================================

embedding_pipeline = EmbeddingPipeline()

# Alias for consistency with other services
EmbeddingPipelineService = EmbeddingPipeline


# ==========================================
# Helper Functions
# ==========================================

async def embed_text(text: str) -> List[float]:
    """
    Convenience function to embed a single text.

    Args:
        text: Input text

    Returns:
        Embedding vector
    """
    return await embedding_pipeline.generate_embedding(text)


async def embed_texts(texts: List[str]) -> List[List[float]]:
    """
    Convenience function to embed multiple texts.

    Args:
        texts: List of input texts

    Returns:
        List of embedding vectors
    """
    return await embedding_pipeline.generate_embeddings_batch(texts)


async def embed_query(query: str) -> List[float]:
    """
    Convenience function to embed a search query.

    Args:
        query: Search query

    Returns:
        Embedding vector
    """
    return await embedding_pipeline.generate_query_embedding(query)


# ==========================================
# CLI Interface
# ==========================================

async def main():
    """CLI for testing embedding pipeline"""
    import argparse

    parser = argparse.ArgumentParser(description="Embedding Pipeline Test")
    parser.add_argument(
        "--text",
        type=str,
        help="Text to embed"
    )
    parser.add_argument(
        "--file",
        type=str,
        help="File with texts (one per line)"
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Show usage statistics"
    )

    args = parser.parse_args()

    if args.text:
        # Embed single text
        logger.info(f"Embedding text: {args.text[:100]}...")
        embedding = await embed_text(args.text)
        logger.info(f"Embedding size: {len(embedding)} dimensions")
        logger.info(f"First 5 values: {embedding[:5]}")

    elif args.file:
        # Embed texts from file
        with open(args.file, "r", encoding="utf-8") as f:
            texts = [line.strip() for line in f if line.strip()]

        logger.info(f"Embedding {len(texts)} texts from {args.file}")
        embeddings = await embed_texts(texts)
        logger.info(f"Generated {len(embeddings)} embeddings")

    if args.stats or args.text or args.file:
        stats = embedding_pipeline.get_usage_stats()
        logger.info(f"\nUsage Statistics:")
        logger.info(f"  Requests: {stats['total_requests']}")
        logger.info(f"  Tokens: {stats['total_tokens']}")
        logger.info(f"  Estimated Cost: ${stats['estimated_cost_usd']}")

    if not any([args.text, args.file, args.stats]):
        parser.print_help()


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
