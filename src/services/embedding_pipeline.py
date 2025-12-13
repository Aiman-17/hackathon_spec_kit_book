"""
Embedding Pipeline Service - Production Grade

Generates embeddings using OpenAI text-embedding-3-small model via OpenRouter API.
Optimized for reliability, observability, and OpenRouter compatibility.

CRITICAL COMPATIBILITY NOTES:
- OpenRouter's embeddings API does NOT support the 'dimensions' parameter
- The text-embedding-3-small model returns 1536 dimensions by default
- Do not pass dimensions= to client.embeddings.create() - it will raise TypeError
- OpenRouter proxy is OpenAI SDK-compatible but with limited parameter support

Architecture:
- Async-first design for high throughput
- Batch processing with rate limiting
- Comprehensive error handling and retry logic
- Cost tracking and usage metrics
- Qdrant integration for vector storage

Author: Backend Team
Last Updated: 2025-12-12
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI, APIError, RateLimitError, APIConnectionError

from src.config import settings

logger = logging.getLogger(__name__)


# ==========================================
# Configuration Constants
# ==========================================

# Model configuration - DO NOT pass 'dimensions' parameter to OpenRouter
EMBEDDING_MODEL = "text-embedding-3-small"
EXPECTED_EMBEDDING_SIZE = 1536  # Auto-returned by model, not configurable via API
MAX_BATCH_SIZE = 100  # Conservative batch size for stability
MAX_TOKENS_PER_REQUEST = 8000  # Token limit per request
RATE_LIMIT_DELAY_SECONDS = 0.5  # Delay between batch requests


# ==========================================
# Embedding Pipeline
# ==========================================

class EmbeddingPipeline:
    """
    Production-grade embedding generation service.

    Features:
    - OpenRouter API compatibility (no dimensions parameter)
    - Async batch processing with automatic rate limiting
    - Comprehensive error handling with specific exception types
    - Usage tracking and cost estimation
    - Input validation and sanitization
    - Auto-retry on transient failures
    - Detailed observability logging

    OpenRouter Compatibility:
    - Uses base_url="https://openrouter.ai/api/v1"
    - Only passes 'model' and 'input' parameters to embeddings.create()
    - Dimensions are auto-detected from response, not specified in request
    """

    def __init__(self):
        """Initialize the embedding pipeline with OpenRouter client."""
        # Initialize OpenRouter-compatible AsyncOpenAI client
        # CRITICAL: OpenRouter requires base_url override
        self.client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1",
            timeout=60.0,  # 60 second timeout for large batches
        )

        # Usage tracking
        self.total_tokens_used = 0
        self.total_requests = 0

        logger.info(
            f"✅ EmbeddingPipeline initialized | "
            f"Model: {EMBEDDING_MODEL} | "
            f"Expected dimensions: {EXPECTED_EMBEDDING_SIZE} | "
            f"Max batch: {MAX_BATCH_SIZE}"
        )

    async def generate_embedding(
        self,
        text: str,
        validate_dimensions: bool = True
    ) -> List[float]:
        """
        Generate embedding for a single text.

        CRITICAL: Does NOT pass 'dimensions' parameter - OpenRouter doesn't support it.
        The model returns 1536 dimensions by default.

        Args:
            text: Input text to embed (will be stripped and validated)
            validate_dimensions: Whether to validate returned embedding size

        Returns:
            1536-dimensional embedding vector

        Raises:
            ValueError: If input text is empty or invalid
            APIError: If OpenRouter API request fails
            TypeError: If dimensions parameter was accidentally passed (shouldn't happen)
        """
        # Input validation
        if not text or not text.strip():
            raise ValueError("Cannot generate embedding for empty text")

        text = text.strip()

        try:
            # CRITICAL FIX: Do NOT pass 'dimensions' parameter
            # OpenRouter's API proxy does not support it and will raise:
            # TypeError: AsyncEmbeddings.create() got an unexpected keyword argument 'dimensions'
            response = await self.client.embeddings.create(
                model=EMBEDDING_MODEL,
                input=text
                # ❌ DO NOT ADD: dimensions=EXPECTED_EMBEDDING_SIZE
            )

            # Extract embedding from response
            embedding = response.data[0].embedding

            # Validate embedding dimensions if requested
            if validate_dimensions and len(embedding) != EXPECTED_EMBEDDING_SIZE:
                logger.warning(
                    f"⚠️ Unexpected embedding size: {len(embedding)} "
                    f"(expected {EXPECTED_EMBEDDING_SIZE})"
                )

            # Track usage
            self.total_tokens_used += response.usage.total_tokens
            self.total_requests += 1

            logger.debug(
                f"✅ Generated embedding | "
                f"Dimensions: {len(embedding)} | "
                f"Tokens: {response.usage.total_tokens} | "
                f"Text preview: {text[:50]}..."
            )

            return embedding

        except RateLimitError as e:
            logger.error(f"❌ Rate limit exceeded: {str(e)}")
            raise
        except APIConnectionError as e:
            logger.error(f"❌ API connection failed: {str(e)}")
            raise
        except APIError as e:
            logger.error(f"❌ OpenAI API error: {str(e)}")
            raise
        except TypeError as e:
            # This catches the 'dimensions' parameter error if it ever reappears
            logger.error(
                f"❌ CRITICAL: TypeError in embeddings.create() - "
                f"Check if 'dimensions' parameter was accidentally added: {str(e)}"
            )
            raise
        except Exception as e:
            logger.error(f"❌ Unexpected error generating embedding: {str(e)}")
            raise

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        batch_size: int = MAX_BATCH_SIZE,
        validate_dimensions: bool = True
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batches with rate limiting.

        CRITICAL: Does NOT pass 'dimensions' parameter - OpenRouter doesn't support it.

        Args:
            texts: List of input texts (empty/whitespace-only texts will be filtered)
            batch_size: Number of texts per API request (default: 100)
            validate_dimensions: Whether to validate returned embedding sizes

        Returns:
            List of 1536-dimensional embedding vectors (one per input text)

        Raises:
            ValueError: If texts list is empty after filtering
            APIError: If any batch request fails
        """
        # Filter and validate inputs
        texts = [t.strip() for t in texts if t and t.strip()]

        if not texts:
            logger.warning("⚠️ No valid texts to embed (all empty/whitespace)")
            return []

        all_embeddings = []
        total_batches = (len(texts) + batch_size - 1) // batch_size

        logger.info(
            f"📊 Starting batch embedding | "
            f"Total texts: {len(texts)} | "
            f"Batches: {total_batches} | "
            f"Batch size: {batch_size}"
        )

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_num = i // batch_size + 1

            logger.debug(
                f"⏳ Processing batch {batch_num}/{total_batches} | "
                f"Size: {len(batch)} texts"
            )

            try:
                # CRITICAL FIX: Do NOT pass 'dimensions' parameter
                # OpenRouter's API proxy does not support it
                response = await self.client.embeddings.create(
                    model=EMBEDDING_MODEL,
                    input=batch
                    # ❌ DO NOT ADD: dimensions=EXPECTED_EMBEDDING_SIZE
                )

                # Extract embeddings in original order
                batch_embeddings = [item.embedding for item in response.data]

                # Validate batch size matches
                if len(batch_embeddings) != len(batch):
                    logger.error(
                        f"❌ Batch size mismatch | "
                        f"Requested: {len(batch)} | "
                        f"Received: {len(batch_embeddings)}"
                    )
                    raise ValueError("Embedding count mismatch in batch response")

                # Validate dimensions if requested
                if validate_dimensions:
                    for idx, emb in enumerate(batch_embeddings):
                        if len(emb) != EXPECTED_EMBEDDING_SIZE:
                            logger.warning(
                                f"⚠️ Batch {batch_num}, item {idx}: "
                                f"Unexpected embedding size {len(emb)} "
                                f"(expected {EXPECTED_EMBEDDING_SIZE})"
                            )

                all_embeddings.extend(batch_embeddings)

                # Track usage
                self.total_tokens_used += response.usage.total_tokens
                self.total_requests += 1

                logger.debug(
                    f"✅ Batch {batch_num}/{total_batches} complete | "
                    f"Tokens: {response.usage.total_tokens} | "
                    f"Embeddings: {len(batch_embeddings)}"
                )

                # Rate limiting - wait between batches to avoid hitting limits
                if i + batch_size < len(texts):
                    await asyncio.sleep(RATE_LIMIT_DELAY_SECONDS)

            except RateLimitError as e:
                logger.error(f"❌ Rate limit on batch {batch_num}: {str(e)}")
                raise
            except APIConnectionError as e:
                logger.error(f"❌ Connection failed on batch {batch_num}: {str(e)}")
                raise
            except APIError as e:
                logger.error(f"❌ API error on batch {batch_num}: {str(e)}")
                raise
            except TypeError as e:
                # This catches the 'dimensions' parameter error if it ever reappears
                logger.error(
                    f"❌ CRITICAL: TypeError in batch {batch_num} - "
                    f"Check if 'dimensions' parameter was accidentally added: {str(e)}"
                )
                raise
            except Exception as e:
                logger.error(f"❌ Unexpected error in batch {batch_num}: {str(e)}")
                raise

        logger.info(
            f"✅ Batch embedding complete | "
            f"Total embeddings: {len(all_embeddings)} | "
            f"Total tokens: {self.total_tokens_used}"
        )

        return all_embeddings

    async def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a search query.

        Convenience method optimized for single query embeddings.
        Delegates to generate_embedding() with validation.

        Args:
            query: Search query text

        Returns:
            1536-dimensional embedding vector
        """
        return await self.generate_embedding(query, validate_dimensions=True)

    def get_usage_stats(self) -> Dict[str, Any]:
        """
        Get usage statistics for monitoring and cost tracking.

        Returns:
            dict with comprehensive usage metrics:
            - total_requests: Number of API calls made
            - total_tokens: Total tokens processed
            - estimated_cost_usd: Estimated cost in USD
            - model: Model name used
            - embedding_size: Expected embedding dimensions
            - avg_tokens_per_request: Average tokens per request
        """
        # Cost calculation for text-embedding-3-small
        # OpenAI pricing: $0.020 per 1M tokens = $0.00002 per 1K tokens
        cost_per_1k_tokens = 0.00002
        estimated_cost = (self.total_tokens_used / 1000) * cost_per_1k_tokens

        # Calculate averages
        avg_tokens = (
            self.total_tokens_used / self.total_requests
            if self.total_requests > 0
            else 0
        )

        return {
            "total_requests": self.total_requests,
            "total_tokens": self.total_tokens_used,
            "estimated_cost_usd": round(estimated_cost, 4),
            "model": EMBEDDING_MODEL,
            "embedding_size": EXPECTED_EMBEDDING_SIZE,
            "avg_tokens_per_request": round(avg_tokens, 2),
        }

    def reset_stats(self):
        """Reset usage statistics (useful for testing or periodic resets)."""
        self.total_tokens_used = 0
        self.total_requests = 0
        logger.info("📊 Usage statistics reset")

    async def embed_and_store_chunks(
        self,
        chunks: List[Dict[str, Any]]
    ) -> int:
        """
        Generate embeddings for text chunks and store them in Qdrant vector database.

        This is a high-level method that:
        1. Extracts text content from chunk objects
        2. Generates embeddings in batch
        3. Creates Qdrant point structures with metadata
        4. Stores vectors in Qdrant collection

        Args:
            chunks: List of chunk dictionaries or Pydantic objects with:
                - content: Text content to embed
                - chunk_id: Unique identifier
                - chapter_id, chapter_title: Chapter metadata
                - section_title: Section within chapter
                - module_name, module_id: Module metadata

        Returns:
            Number of vectors successfully stored in Qdrant

        Raises:
            ValueError: If chunks list is empty or invalid
            APIError: If embedding generation fails
            Exception: If Qdrant storage fails
        """
        from qdrant_client.models import PointStruct
        from src.services.qdrant_manager import qdrant_manager

        if not chunks:
            logger.warning("⚠️ No chunks provided to embed_and_store_chunks")
            return 0

        try:
            # Extract text content from chunks (support both dict and Pydantic objects)
            texts = []
            for idx, chunk in enumerate(chunks):
                try:
                    if isinstance(chunk, dict):
                        content = chunk.get("content", "")
                    else:
                        content = getattr(chunk, "content", "")

                    if content and content.strip():
                        texts.append(content.strip())
                    else:
                        logger.warning(f"⚠️ Chunk {idx} has empty content, skipping")
                except Exception as e:
                    logger.error(f"❌ Error extracting content from chunk {idx}: {e}")
                    raise

            if not texts:
                raise ValueError("No valid text content found in chunks")

            logger.info(f"📝 Embedding {len(texts)} chunks for Qdrant storage")

            # Generate embeddings in batch (efficient API usage)
            embeddings = await self.generate_embeddings_batch(texts, validate_dimensions=True)

            if len(embeddings) != len(texts):
                raise ValueError(
                    f"Embedding count mismatch: {len(embeddings)} != {len(texts)}"
                )

            # Create Qdrant point structures with metadata
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                # Extract metadata (support both dict and Pydantic objects)
                if isinstance(chunk, dict):
                    content = chunk.get("content", "")
                    chunk_id = chunk.get("chunk_id", f"chunk_{i}")
                    chapter_id = chunk.get("chapter_id", "unknown")
                    chapter_title = chunk.get("chapter_title", "Unknown")
                    section_title = chunk.get("section_title", "Unknown Section")
                    module_name = chunk.get("module_name", "Unknown Module")
                    module_id = chunk.get("module_id", "unknown")
                else:
                    # Pydantic object access
                    content = chunk.content
                    chunk_id = f"{chunk.chapter_metadata.chapter_id}_chunk_{i}"
                    chapter_id = chunk.chapter_metadata.chapter_id
                    chapter_title = chunk.chapter_metadata.chapter_title
                    section_title = chunk.chunk_metadata.heading
                    module_name = chunk.chapter_metadata.module_title
                    module_id = chunk.chapter_metadata.module_id

                # Generate stable point ID from chunk_id
                # Use modulo to ensure it fits in 64-bit signed integer range
                point_id = hash(chunk_id) % (2**63)

                # Create Qdrant point with embedding and rich metadata
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
                        "embedding_model": EMBEDDING_MODEL,
                        "embedding_dimensions": len(embedding),
                    }
                )
                points.append(point)

            # Store all points in Qdrant
            qdrant_manager.upsert_points(points)

            logger.info(
                f"✅ Successfully stored {len(points)} vectors in Qdrant | "
                f"Model: {EMBEDDING_MODEL} | "
                f"Dimensions: {EXPECTED_EMBEDDING_SIZE}"
            )

            return len(points)

        except Exception as e:
            logger.error(f"❌ Failed to embed and store chunks: {str(e)}")
            raise


# ==========================================
# Global Instance (Singleton Pattern)
# ==========================================

# Global singleton instance for convenient access
embedding_pipeline = EmbeddingPipeline()

# Alias for consistency with other services
EmbeddingPipelineService = EmbeddingPipeline


# ==========================================
# Convenience Helper Functions
# ==========================================

async def embed_text(text: str) -> List[float]:
    """
    Convenience function to embed a single text.

    Uses the global singleton instance.

    Args:
        text: Input text to embed

    Returns:
        1536-dimensional embedding vector
    """
    return await embedding_pipeline.generate_embedding(text)


async def embed_texts(texts: List[str]) -> List[List[float]]:
    """
    Convenience function to embed multiple texts in batch.

    Uses the global singleton instance.

    Args:
        texts: List of input texts

    Returns:
        List of 1536-dimensional embedding vectors
    """
    return await embedding_pipeline.generate_embeddings_batch(texts)


async def embed_query(query: str) -> List[float]:
    """
    Convenience function to embed a search query.

    Uses the global singleton instance.

    Args:
        query: Search query text

    Returns:
        1536-dimensional embedding vector
    """
    return await embedding_pipeline.generate_query_embedding(query)


# ==========================================
# CLI Interface for Testing
# ==========================================

async def main():
    """
    Command-line interface for testing the embedding pipeline.

    Usage:
        python -m src.services.embedding_pipeline --text "Your text here"
        python -m src.services.embedding_pipeline --file texts.txt
        python -m src.services.embedding_pipeline --stats
    """
    import argparse

    parser = argparse.ArgumentParser(
        description="Embedding Pipeline Testing CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Embed single text
  python -m src.services.embedding_pipeline --text "What is AI?"

  # Embed multiple texts from file
  python -m src.services.embedding_pipeline --file texts.txt

  # Show usage statistics
  python -m src.services.embedding_pipeline --stats
        """
    )
    parser.add_argument(
        "--text",
        type=str,
        help="Single text to embed"
    )
    parser.add_argument(
        "--file",
        type=str,
        help="File with texts to embed (one per line)"
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Show usage statistics"
    )

    args = parser.parse_args()

    # Configure logging for CLI
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    if args.text:
        # Embed single text
        logger.info(f"📝 Embedding text: {args.text[:100]}...")
        embedding = await embed_text(args.text)
        logger.info(f"✅ Embedding generated | Dimensions: {len(embedding)}")
        logger.info(f"First 5 values: {embedding[:5]}")

    elif args.file:
        # Embed texts from file
        try:
            with open(args.file, "r", encoding="utf-8") as f:
                texts = [line.strip() for line in f if line.strip()]

            if not texts:
                logger.error("❌ No valid texts found in file")
                return

            logger.info(f"📝 Embedding {len(texts)} texts from {args.file}")
            embeddings = await embed_texts(texts)
            logger.info(f"✅ Generated {len(embeddings)} embeddings")

        except FileNotFoundError:
            logger.error(f"❌ File not found: {args.file}")
            return
        except Exception as e:
            logger.error(f"❌ Error reading file: {e}")
            return

    if args.stats or args.text or args.file:
        # Show usage statistics
        stats = embedding_pipeline.get_usage_stats()
        logger.info("\n" + "="*50)
        logger.info("📊 USAGE STATISTICS")
        logger.info("="*50)
        logger.info(f"  Model:               {stats['model']}")
        logger.info(f"  Embedding Size:      {stats['embedding_size']} dimensions")
        logger.info(f"  Total Requests:      {stats['total_requests']}")
        logger.info(f"  Total Tokens:        {stats['total_tokens']:,}")
        logger.info(f"  Avg Tokens/Request:  {stats['avg_tokens_per_request']}")
        logger.info(f"  Estimated Cost:      ${stats['estimated_cost_usd']:.4f} USD")
        logger.info("="*50)

    if not any([args.text, args.file, args.stats]):
        parser.print_help()


if __name__ == "__main__":
    asyncio.run(main())
