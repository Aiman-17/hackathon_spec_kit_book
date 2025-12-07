"""
Chat Message Pydantic Models

Data models for RAG chatbot conversation history.
"""

from datetime import datetime
from typing import List, Optional
from uuid import UUID, uuid4
from pydantic import BaseModel, Field


# ==========================================
# Citation Models
# ==========================================

class Citation(BaseModel):
    """Individual citation from RAG retrieval"""
    chapter_id: str
    chapter_title: str
    section_heading: Optional[str] = None
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    chunk_content: Optional[str] = None  # Preview of the retrieved chunk

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Chat Message Models
# ==========================================

class MessageType(str):
    """Enum-like class for message types"""
    USER = "user"
    ASSISTANT = "assistant"


class ChatMessageBase(BaseModel):
    """Base chat message model"""
    message_type: str = Field(..., pattern="^(user|assistant)$")
    query_text: Optional[str] = None
    response_text: Optional[str] = None
    selected_text: Optional[str] = None


class ChatMessageCreate(ChatMessageBase):
    """Model for creating a chat message"""
    session_id: UUID = Field(default_factory=uuid4)
    citations: Optional[List[Citation]] = None
    retrieved_chunks_count: Optional[int] = 0
    model_used: Optional[str] = "gpt-4-turbo-preview"
    tokens_used: Optional[int] = None
    response_time_ms: Optional[int] = None


class ChatMessage(ChatMessageBase):
    """Complete chat message model from database"""
    id: UUID = Field(default_factory=uuid4)
    user_id: Optional[UUID] = None
    session_id: UUID
    citations: Optional[List[Citation]] = None
    retrieved_chunks_count: Optional[int] = 0
    model_used: Optional[str] = None
    tokens_used: Optional[int] = None
    response_time_ms: Optional[int] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    model_config = {
        "from_attributes": True
    }


# ==========================================
# Chat Query Models
# ==========================================

class ChatQueryRequest(BaseModel):
    """Request model for chat query endpoint"""
    query: str = Field(..., min_length=1, max_length=2000)
    selected_text: Optional[str] = Field(None, max_length=5000)
    session_id: Optional[UUID] = Field(default_factory=uuid4)
    user_id: Optional[UUID] = None  # Optional - supports anonymous queries


class ChatQueryResponse(BaseModel):
    """Response model for chat query endpoint"""
    response: str
    citations: List[Citation] = []
    session_id: UUID
    retrieved_chunks_count: int = 0
    model_used: str = "gpt-4-turbo-preview"
    tokens_used: Optional[int] = None
    response_time_ms: Optional[int] = None


# ==========================================
# Chat History Models
# ==========================================

class ChatHistoryRequest(BaseModel):
    """Request model for retrieving chat history"""
    user_id: UUID
    session_id: Optional[UUID] = None
    limit: int = Field(default=50, ge=1, le=200)
    offset: int = Field(default=0, ge=0)


class ChatHistoryResponse(BaseModel):
    """Response model for chat history"""
    messages: List[ChatMessage]
    total_count: int
    has_more: bool


class ChatSession(BaseModel):
    """Model for a chat session summary"""
    session_id: UUID
    user_id: Optional[UUID]
    message_count: int
    first_message_at: datetime
    last_message_at: datetime
    topics: Optional[List[str]] = []  # Extracted topics/keywords

    model_config = {
        "from_attributes": True
    }


# ==========================================
# RAG Context Models
# ==========================================

class RAGContext(BaseModel):
    """Context information for RAG query"""
    query: str
    selected_text: Optional[str] = None
    conversation_history: List[ChatMessage] = []
    user_skill_level: Optional[str] = None  # For personalization
    preferred_language: str = "en"


class RAGResult(BaseModel):
    """Result from RAG pipeline"""
    response: str
    citations: List[Citation]
    retrieved_chunks: List[str] = []
    total_chunks_found: int
    model_used: str
    tokens_used: int
    processing_time_ms: int
