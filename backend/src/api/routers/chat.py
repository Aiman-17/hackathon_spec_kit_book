"""
Chat/RAG API Router

Endpoints for RAG-powered question answering using the textbook content.
"""

import logging
from typing import List, Optional, Dict, Any
from datetime import datetime
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from src.services.qdrant_manager import qdrant_manager
from src.services.embedding_pipeline import EmbeddingPipelineService
from src.config import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/chat", tags=["Chat"])


# ==========================================
# Request/Response Models
# ==========================================

class ChatMessage(BaseModel):
    """Chat message model"""
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")
    timestamp: Optional[datetime] = None


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    query: str = Field(..., description="User's question", min_length=1, max_length=2000)
    context: Optional[str] = Field(None, description="Selected text for context-aware questions")
    conversation_id: Optional[str] = Field(None, description="Conversation ID for history tracking")
    max_results: int = Field(3, description="Maximum number of relevant chunks to retrieve", ge=1, le=10)
    include_sources: bool = Field(True, description="Whether to include source citations")


class SourceCitation(BaseModel):
    """Source citation for RAG response"""
    chapter: str
    section: str
    module: str
    content_snippet: str
    similarity_score: float


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    answer: str
    sources: List[SourceCitation] = []
    conversation_id: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    tokens_used: Optional[int] = None


class ConversationHistory(BaseModel):
    """Conversation history model"""
    conversation_id: str
    messages: List[ChatMessage]
    created_at: datetime
    updated_at: datetime


# ==========================================
# RAG Helper Functions
# ==========================================

async def retrieve_relevant_chunks(query: str, max_results: int = 3, context: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Retrieve relevant chunks from Qdrant using semantic search.

    Args:
        query: User's question
        max_results: Maximum number of chunks to retrieve
        context: Optional selected text for context-aware search

    Returns:
        List of relevant chunks with metadata
    """
    try:
        # Initialize embedding service
        embedding_service = EmbeddingPipelineService()

        # If context is provided, combine it with the query for better retrieval
        search_query = f"{context}\n\n{query}" if context else query

        # Generate embedding for the query
        query_embedding = await embedding_service.generate_embedding(search_query)

        # Search in Qdrant
        search_results = qdrant_manager.search(
            query_vector=query_embedding,
            limit=max_results,
            score_threshold=0.5  # Lowered from 0.7 - more lenient matching
        )

        logger.info(f"Found {len(search_results)} results for query")
        return search_results

    except Exception as e:
        logger.error(f"Failed to retrieve chunks: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve relevant content: {str(e)}"
        )


async def generate_rag_response(query: str, relevant_chunks: List[Dict[str, Any]]) -> str:
    """
    Generate a response using OpenAI with RAG context.

    Args:
        query: User's question
        relevant_chunks: Retrieved chunks from vector database

    Returns:
        Generated answer
    """
    try:
        from openai import AsyncOpenAI

        client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Construct context from relevant chunks
        context_parts = []
        for i, chunk in enumerate(relevant_chunks, 1):
            payload = chunk.get("payload", {})
            content = payload.get("content", "")
            chapter = payload.get("chapter_title", "Unknown")
            section = payload.get("section_title", "Unknown Section")

            context_parts.append(
                f"[Source {i} - {chapter} / {section}]\n{content}\n"
            )

        context = "\n\n".join(context_parts)

        # System prompt for RAG
        system_prompt = """You are an expert tutor for the Physical AI & Humanoid Robotics textbook.
Your role is to answer students' questions using ONLY the provided textbook content.

Instructions:
- Answer clearly and concisely using the provided sources
- If the sources don't contain enough information, acknowledge the limitation
- Include specific references to chapters/sections when relevant
- Use technical terminology appropriately for undergraduate-level students
- Provide examples or clarifications when helpful
- NEVER make up information not in the sources"""

        # User prompt with context
        user_prompt = f"""Based on the following textbook content, answer this question:

**Question:** {query}

**Relevant Textbook Content:**
{context}

**Answer:**"""

        # Call OpenAI API
        response = await client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=800,
        )

        answer = response.choices[0].message.content
        return answer

    except Exception as e:
        logger.error(f"Failed to generate RAG response: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate response: {str(e)}"
        )


# ==========================================
# API Endpoints
# ==========================================

@router.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Answer a question using RAG (Retrieval-Augmented Generation).

    This endpoint:
    1. Embeds the user's question
    2. Retrieves relevant textbook chunks from Qdrant
    3. Generates an answer using OpenAI with the retrieved context
    4. Returns the answer with source citations

    Args:
        request: Chat request with query and optional context

    Returns:
        ChatResponse with answer and source citations
    """
    try:
        # Step 1: Retrieve relevant chunks
        relevant_chunks = await retrieve_relevant_chunks(
            query=request.query,
            max_results=request.max_results,
            context=request.context
        )

        if not relevant_chunks:
            return ChatResponse(
                answer="I couldn't find relevant information in the textbook to answer your question. Please try rephrasing your question or ask about topics covered in the Physical AI & Humanoid Robotics course.",
                sources=[],
                conversation_id=request.conversation_id
            )

        # Step 2: Generate answer using RAG
        answer = await generate_rag_response(request.query, relevant_chunks)

        # Step 3: Prepare source citations
        sources = []
        if request.include_sources:
            for chunk in relevant_chunks:
                payload = chunk.get("payload", {})
                sources.append(SourceCitation(
                    chapter=payload.get("chapter_title", "Unknown Chapter"),
                    section=payload.get("section_title", "Unknown Section"),
                    module=payload.get("module_name", "Unknown Module"),
                    content_snippet=payload.get("content", "")[:200] + "...",
                    similarity_score=chunk.get("score", 0.0)
                ))

        # TODO: Store conversation in Postgres (Phase 4)

        return ChatResponse(
            answer=answer,
            sources=sources,
            conversation_id=request.conversation_id,
            tokens_used=None  # TODO: Track token usage
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat query failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}"
        )


@router.get("/health")
async def chat_health():
    """
    Health check for chat service.

    Returns:
        Service status and configuration
    """
    return {
        "status": "healthy",
        "service": "rag-chat",
        "qdrant_connected": qdrant_manager.is_connected(),
        "features": {
            "context_aware_search": True,
            "source_citations": True,
            "conversation_history": False,  # TODO: Phase 4
        }
    }
