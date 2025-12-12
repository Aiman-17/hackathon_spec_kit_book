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

        # System prompt for RAG with explicit reasoning
        system_prompt = """You are an expert tutor for the Physical AI & Humanoid Robotics textbook.
Your role is to answer students' questions using ONLY the provided textbook content.

CRITICAL RULES:
1. ONLY use information from the provided textbook sources
2. If the sources don't contain relevant information, explicitly state: "The textbook content provided does not contain information about [topic]. Please ask about topics covered in the Physical AI & Humanoid Robotics course."
3. NEVER make up, infer, or hallucinate information not present in the sources
4. Always cite specific chapters/sections when using information

ANSWER FORMAT:
1. Start with a brief answer (1-2 sentences)
2. Provide reasoning steps explaining how you arrived at the answer from the sources
3. Include specific citations in the format: [Source X: Chapter Name / Section Name]
4. If uncertain or information is incomplete, explicitly state the limitation

Examples:
- Good: "ROS 2 uses a distributed architecture [Source 1: Introduction to ROS 2 / Architecture]. This means..."
- Bad: "ROS 2 is commonly used..." (no citation)
- Good: "The textbook content provided does not contain information about underwater robotics."
- Bad: Making up an answer about underwater robotics"""

        # User prompt with context
        user_prompt = f"""Based on the following textbook content, answer this question:

**Question:** {query}

**Relevant Textbook Content:**
{context}

**Instructions:**
1. First, determine if the sources contain relevant information to answer the question
2. If yes, provide a clear answer with reasoning steps and citations
3. If no, state that the information is not in the textbook
4. Use the format: Answer → Reasoning → Citations

**Your Response:**"""

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
                answer="""I couldn't find relevant information in the textbook to answer your question.

This textbook covers:
- Module 1: Foundations of Physical AI & Robotics (ROS 2, hardware, mathematics)
- Module 2: Simulation Environments (Gazebo, Unity, URDF modeling)
- Module 3: Advanced Perception & Navigation (Computer vision, SLAM, motion planning)
- Module 4: Humanoid AI Systems (Autonomous agents, multi-agent coordination)

Please try:
1. Rephrasing your question
2. Asking about topics covered in these modules
3. Being more specific about which aspect of robotics you're interested in""",
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
