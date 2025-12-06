"""
Qdrant Vector Database Manager

Manages Qdrant Cloud connections, collection creation, and vector operations.
"""

import logging
from typing import List, Optional, Dict, Any

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest,
    QueryResponse,
)

from src.config import settings
from src.models.chapter import ChapterChunk, SimilaritySearchResult

logger = logging.getLogger(__name__)


# ==========================================
# Qdrant Manager
# ==========================================

class QdrantManager:
    """
    Manages Qdrant vector database operations.

    Handles collection management, vector search, and document indexing.
    """

    # Collection configuration
    COLLECTION_NAME = "book_chapters"
    EMBEDDING_SIZE = 1536  # OpenAI text-embedding-3-small
    DISTANCE_METRIC = Distance.COSINE

    def __init__(self):
        self.client: Optional[QdrantClient] = None

    def initialize(self):
        """Initialize Qdrant client connection"""
        if self.client:
            logger.warning("Qdrant client already initialized")
            return

        try:
            logger.info("Initializing Qdrant client...")

            self.client = QdrantClient(
                url=str(settings.qdrant_url),
                api_key=settings.qdrant_api_key,
                timeout=60,
            )

            logger.info("✅ Qdrant client initialized")

            # Verify connection
            collections = self.client.get_collections()
            logger.info(f"Connected to Qdrant. Collections: {len(collections.collections)}")

        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise

    def close(self):
        """Close Qdrant client connection"""
        if self.client:
            self.client.close()
            self.client = None
            logger.info("Qdrant client closed")

    def is_connected(self) -> bool:
        """Check if Qdrant client is initialized and connected"""
        if not self.client:
            return False

        try:
            # Try to get collections to verify connection
            self.client.get_collections()
            return True
        except Exception:
            return False

    # ==========================================
    # Collection Management
    # ==========================================

    def create_collection(self, recreate: bool = False):
        """
        Create the book_chapters collection.

        Args:
            recreate: If True, delete existing collection first
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call initialize() first.")

        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            exists = any(c.name == self.COLLECTION_NAME for c in collections)

            if exists:
                if recreate:
                    logger.warning(f"Deleting existing collection: {self.COLLECTION_NAME}")
                    self.client.delete_collection(self.COLLECTION_NAME)
                else:
                    logger.info(f"Collection '{self.COLLECTION_NAME}' already exists")
                    return

            # Create collection
            logger.info(f"Creating collection: {self.COLLECTION_NAME}")

            self.client.create_collection(
                collection_name=self.COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=self.EMBEDDING_SIZE,
                    distance=self.DISTANCE_METRIC
                ),
                # Optimize for search performance
                hnsw_config={
                    "m": 16,
                    "ef_construct": 100,
                },
                optimizers_config={
                    "default_segment_number": 2,
                },
            )

            logger.info(f"✅ Collection '{self.COLLECTION_NAME}' created successfully")

        except Exception as e:
            logger.error(f"Failed to create collection: {str(e)}")
            raise

    def delete_collection(self):
        """Delete the book_chapters collection"""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            self.client.delete_collection(self.COLLECTION_NAME)
            logger.info(f"✅ Collection '{self.COLLECTION_NAME}' deleted")

        except Exception as e:
            logger.error(f"Failed to delete collection: {str(e)}")
            raise

    def collection_exists(self) -> bool:
        """Check if the collection exists"""
        if not self.client:
            return False

        try:
            collections = self.client.get_collections().collections
            return any(c.name == self.COLLECTION_NAME for c in collections)

        except Exception as e:
            logger.error(f"Failed to check collection existence: {str(e)}")
            return False

    def get_collection_info(self) -> Dict[str, Any]:
        """Get collection information and statistics"""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            info = self.client.get_collection(self.COLLECTION_NAME)

            return {
                "name": info.config.params.vectors.size if hasattr(info.config.params, 'vectors') else None,
                "vector_size": self.EMBEDDING_SIZE,
                "distance": self.DISTANCE_METRIC,
                "points_count": info.points_count,
                "indexed_vectors_count": info.indexed_vectors_count,
                "status": info.status,
            }

        except Exception as e:
            logger.error(f"Failed to get collection info: {str(e)}")
            raise

    # ==========================================
    # Vector Operations
    # ==========================================

    def upsert_points(self, points: List[PointStruct]) -> None:
        """
        Insert or update points in the collection.

        Args:
            points: List of PointStruct objects with id, vector, and payload
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            self.client.upsert(
                collection_name=self.COLLECTION_NAME,
                points=points
            )

            logger.info(f"✅ Upserted {len(points)} points to {self.COLLECTION_NAME}")

        except Exception as e:
            logger.error(f"Failed to upsert points: {str(e)}")
            raise

    def delete_points(self, point_ids: List[int]) -> None:
        """Delete points by IDs"""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            self.client.delete(
                collection_name=self.COLLECTION_NAME,
                points_selector=point_ids
            )

            logger.info(f"✅ Deleted {len(point_ids)} points")

        except Exception as e:
            logger.error(f"Failed to delete points: {str(e)}")
            raise

    # ==========================================
    # Similarity Search
    # ==========================================

    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        filter_conditions: Optional[Filter] = None,
        score_threshold: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform similarity search.

        Args:
            query_vector: Query embedding vector (1536 dims)
            limit: Number of results to return
            filter_conditions: Optional Qdrant filter
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of search results with payload and score
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized")

        try:
            results = self.client.search(
                collection_name=self.COLLECTION_NAME,
                query_vector=query_vector,
                limit=limit,
                query_filter=filter_conditions,
                score_threshold=score_threshold,
                with_payload=True,
                with_vectors=False,  # Don't return vectors to save bandwidth
            )

            return [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                }
                for result in results
            ]

        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            raise

    def search_with_filter(
        self,
        query_vector: List[float],
        module_id: Optional[str] = None,
        chapter_id: Optional[str] = None,
        limit: int = 5,
        score_threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """
        Search with module/chapter filters.

        Args:
            query_vector: Query embedding
            module_id: Filter by module (e.g., "module-1")
            chapter_id: Filter by chapter (e.g., "01-introduction")
            limit: Number of results
            score_threshold: Minimum score

        Returns:
            List of filtered search results
        """
        filter_conditions = None

        # Build filter conditions
        conditions = []
        if module_id:
            conditions.append(
                FieldCondition(
                    key="module_id",
                    match=MatchValue(value=module_id)
                )
            )

        if chapter_id:
            conditions.append(
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )
            )

        if conditions:
            filter_conditions = Filter(must=conditions)

        return self.search(
            query_vector=query_vector,
            limit=limit,
            filter_conditions=filter_conditions,
            score_threshold=score_threshold
        )

    # ==========================================
    # Health Check
    # ==========================================

    def health_check(self) -> Dict[str, Any]:
        """
        Check Qdrant health and collection status.

        Returns:
            dict with health status and collection info
        """
        if not self.client:
            return {
                "status": "unhealthy",
                "message": "Client not initialized"
            }

        try:
            # Check collection
            if not self.collection_exists():
                return {
                    "status": "unhealthy",
                    "message": f"Collection '{self.COLLECTION_NAME}' does not exist"
                }

            # Get collection info
            info = self.get_collection_info()

            return {
                "status": "healthy",
                "collection": self.COLLECTION_NAME,
                "points_count": info["points_count"],
                "indexed_vectors": info["indexed_vectors_count"],
                "vector_size": info["vector_size"],
            }

        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }


# ==========================================
# Global Instance
# ==========================================

qdrant_manager = QdrantManager()


# ==========================================
# CLI Interface
# ==========================================

async def main():
    """CLI for Qdrant management"""
    import argparse

    parser = argparse.ArgumentParser(description="Qdrant Vector Database Manager")
    parser.add_argument(
        "--create-collection",
        action="store_true",
        help="Create collection"
    )
    parser.add_argument(
        "--recreate",
        action="store_true",
        help="Recreate collection (delete existing)"
    )
    parser.add_argument(
        "--delete-collection",
        action="store_true",
        help="Delete collection"
    )
    parser.add_argument(
        "--health",
        action="store_true",
        help="Check health"
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Show collection info"
    )

    args = parser.parse_args()

    # Initialize client
    qdrant_manager.initialize()

    try:
        if args.create_collection:
            qdrant_manager.create_collection(recreate=args.recreate)

        elif args.delete_collection:
            qdrant_manager.delete_collection()

        elif args.health:
            health = qdrant_manager.health_check()
            logger.info(f"Health check: {health}")

        elif args.info:
            info = qdrant_manager.get_collection_info()
            logger.info(f"Collection info: {info}")

        else:
            parser.print_help()

    finally:
        qdrant_manager.close()


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
