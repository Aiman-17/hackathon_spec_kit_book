"""
Data Models Package

Pydantic models for data validation and serialization.
"""

from .user import (
    User,
    UserCreate,
    UserLogin,
    UserPublic,
    UserProfile,
    UserProfileCreate,
    UserProfileUpdate,
    UserWithProfile,
    Token,
    TokenPayload,
    SkillLevel,
    PreferredLanguage,
)

from .chat import (
    ChatMessage,
    ChatMessageCreate,
    ChatQueryRequest,
    ChatQueryResponse,
    ChatHistoryRequest,
    ChatHistoryResponse,
    ChatSession,
    Citation,
    RAGContext,
    RAGResult,
    MessageType,
)

from .chapter import (
    ChapterMetadata,
    ModuleMetadata,
    ChunkMetadata,
    ChapterChunk,
    ChapterChunkCreate,
    SimilaritySearchRequest,
    SimilaritySearchResult,
    SimilaritySearchResponse,
    DocumentIngestionRequest,
    DocumentIngestionResult,
    IngestionBatchResult,
    ChunkValidation,
    ChunkQualityReport,
    ContentStatistics,
)

__all__ = [
    # User models
    "User",
    "UserCreate",
    "UserLogin",
    "UserPublic",
    "UserProfile",
    "UserProfileCreate",
    "UserProfileUpdate",
    "UserWithProfile",
    "Token",
    "TokenPayload",
    "SkillLevel",
    "PreferredLanguage",
    # Chat models
    "ChatMessage",
    "ChatMessageCreate",
    "ChatQueryRequest",
    "ChatQueryResponse",
    "ChatHistoryRequest",
    "ChatHistoryResponse",
    "ChatSession",
    "Citation",
    "RAGContext",
    "RAGResult",
    "MessageType",
    # Chapter models
    "ChapterMetadata",
    "ModuleMetadata",
    "ChunkMetadata",
    "ChapterChunk",
    "ChapterChunkCreate",
    "SimilaritySearchRequest",
    "SimilaritySearchResult",
    "SimilaritySearchResponse",
    "DocumentIngestionRequest",
    "DocumentIngestionResult",
    "IngestionBatchResult",
    "ChunkValidation",
    "ChunkQualityReport",
    "ContentStatistics",
]
