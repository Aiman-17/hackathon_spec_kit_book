"""
Chapter and ChapterChunk Pydantic Models

Data models for textbook content and vector embeddings.
"""

from datetime import datetime
from typing import List, Optional
from uuid import UUID, uuid4
from pydantic import BaseModel, Field


# ==========================================
# Chapter Metadata Models
# ==========================================

class ChapterMetadata(BaseModel):
    """Metadata for a textbook chapter"""
    module_id: str = Field(..., pattern="^module-[1-4]$")
    module_title: str
    chapter_id: str
    chapter_title: str
    chapter_number: int = Field(..., ge=1, le=25)
    tags: List[str] = []
    description: Optional[str] = None
    sidebar_position: Optional[int] = None

    model_config = {
        "from_attributes": True
    }


class ModuleMetadata(BaseModel):
    """Metadata for a module (group of chapters)"""
    module_id: str = Field(..., pattern="^module-[1-4]$")
    module_title: str
    module_number: int = Field(..., ge=1, le=4)
    chapter_count: int = Field(..., ge=1, le=10)
    description: Optional[str] = None

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Chunk Models
# ==========================================

class ChunkMetadata(BaseModel):
    """Metadata for a semantic chunk"""
    heading: str
    heading_level: int = Field(..., ge=2, le=4)  # H2, H3, or H4
    section_path: List[str] = []  # Hierarchical path: ["Section", "Subsection"]
    token_count: int
    chunk_index: int = 0  # Index within the chapter
    has_code_blocks: bool = False
    has_diagrams: bool = False
    keywords: List[str] = []

    model_config = {
        "from_attributes": True
    }


class ChapterChunkBase(BaseModel):
    """Base model for a chapter chunk"""
    content: str = Field(..., min_length=100, max_length=5000)
    chunk_metadata: ChunkMetadata
    chapter_metadata: ChapterMetadata


class ChapterChunkCreate(ChapterChunkBase):
    """Model for creating a chapter chunk (for ingestion)"""
    embedding: Optional[List[float]] = None  # 1536-dim vector from OpenAI


class ChapterChunk(ChapterChunkBase):
    """Complete chapter chunk model"""
    id: UUID = Field(default_factory=uuid4)
    embedding: Optional[List[float]] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Search and Retrieval Models
# ==========================================

class SimilaritySearchRequest(BaseModel):
    """Request model for similarity search"""
    query: str = Field(..., min_length=1, max_length=2000)
    top_k: int = Field(default=5, ge=1, le=20)
    filter_module: Optional[str] = Field(None, pattern="^module-[1-4]$")
    filter_chapter: Optional[str] = None
    min_score: float = Field(default=0.5, ge=0.0, le=1.0)


class SimilaritySearchResult(BaseModel):
    """Individual search result from vector similarity"""
    chunk: ChapterChunk
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    distance: float  # Cosine distance

    model_config = {
        "from_attributes": True
    }


class SimilaritySearchResponse(BaseModel):
    """Response model for similarity search"""
    results: List[SimilaritySearchResult]
    total_found: int
    query: str
    processing_time_ms: int


# ==========================================
# Document Ingestion Models
# ==========================================

class IngestionStatus(str):
    """Enum-like class for ingestion status"""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


class DocumentIngestionRequest(BaseModel):
    """Request model for document ingestion"""
    file_path: str
    module_id: str = Field(..., pattern="^module-[1-4]$")
    chapter_id: str
    force_reindex: bool = False


class DocumentIngestionResult(BaseModel):
    """Result from document ingestion"""
    module_id: str
    chapter_id: str
    chunks_created: int
    total_tokens: int
    status: str
    processing_time_ms: int
    errors: List[str] = []


class IngestionBatchResult(BaseModel):
    """Result from batch ingestion"""
    total_documents: int
    successful: int
    failed: int
    total_chunks: int
    total_tokens: int
    results: List[DocumentIngestionResult]
    processing_time_ms: int


# ==========================================
# Chunk Validation Models
# ==========================================

class ChunkValidation(BaseModel):
    """Validation result for a chunk"""
    chunk_id: UUID
    is_valid: bool
    errors: List[str] = []
    warnings: List[str] = []
    token_count: int
    has_embedding: bool


class ChunkQualityReport(BaseModel):
    """Quality report for ingested chunks"""
    total_chunks: int
    valid_chunks: int
    invalid_chunks: int
    avg_token_count: float
    min_token_count: int
    max_token_count: int
    chunks_with_embeddings: int
    quality_score: float = Field(..., ge=0.0, le=1.0)
    validation_results: List[ChunkValidation]


# ==========================================
# Content Statistics Models
# ==========================================

class ContentStatistics(BaseModel):
    """Statistics for textbook content"""
    total_modules: int
    total_chapters: int
    total_chunks: int
    total_tokens: int
    avg_chunks_per_chapter: float
    chunks_by_module: dict  # {module_id: count}
    embedding_coverage: float = Field(..., ge=0.0, le=1.0)  # % of chunks with embeddings

    model_config = {
        "from_attributes": True
    }
