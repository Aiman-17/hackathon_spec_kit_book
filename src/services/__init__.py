"""
Services Package

Core backend services for the AI-Native Textbook platform.
"""

from .postgres_manager import postgres_manager, PostgresManager
from .qdrant_manager import qdrant_manager, QdrantManager
from .embedding_pipeline import embedding_pipeline, EmbeddingPipeline, embed_text, embed_texts, embed_query
from .document_ingestion import document_ingestion_service, DocumentIngestionService

__all__ = [
    "postgres_manager",
    "PostgresManager",
    "qdrant_manager",
    "QdrantManager",
    "embedding_pipeline",
    "EmbeddingPipeline",
    "embed_text",
    "embed_texts",
    "embed_query",
    "document_ingestion_service",
    "DocumentIngestionService",
]
