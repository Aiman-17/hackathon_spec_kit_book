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
    # Personalization parameters (no auth required - from localStorage)
    user_level: Optional[str] = Field("student", description="User level: student, beginner, or advanced")
    language: Optional[str] = Field("en", description="Response language: en or ur")


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


async def generate_rag_response(
    query: str,
    relevant_chunks: List[Dict[str, Any]],
    user_level: str = "student"
) -> str:
    """
    Generate a response using OpenRouter with RAG context.

    Args:
        query: User's question
        relevant_chunks: Retrieved chunks from vector database

    Returns:
        Generated answer or fallback message if credits exhausted
    """
    try:
        from openai import AsyncOpenAI
        from openai import RateLimitError, APIError

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

        # Adjust tone/depth based on user level
        level_instructions = {
            "student": "Provide clear, accessible explanations suitable for students new to robotics.",
            "beginner": "Provide straightforward explanations with some technical detail.",
            "advanced": "Provide concise, technical explanations assuming robotics knowledge."
        }
        level_instruction = level_instructions.get(user_level, level_instructions["student"])

        # System prompt for RAG - textbook as single source of truth
        system_prompt = f"""You are an expert tutor for the Physical AI & Humanoid Robotics textbook.
Your role is to answer questions using ONLY the provided textbook content.

CRITICAL RULES:
1. The textbook is the SINGLE SOURCE OF TRUTH - use ONLY information from the provided sources
2. If sources exist for the query topic, ALWAYS provide an answer - NEVER reject valid textbook queries
3. NEVER make up, infer, or hallucinate information not present in the sources
4. NEVER use external facts or knowledge beyond the retrieved chunks
5. Keep answers concise:
   - Definitions: 6-7 lines maximum
   - Explanations: brief and focused, no verbosity
6. NO over-quoting or long citations - summarize key points

ANSWER FORMAT:
1. Brief answer (1-2 sentences) anchored to sources
2. Concise reasoning from the retrieved chunks only
3. Minimal citations: [Source X: Chapter/Section]

PERSONALIZATION: {level_instruction}

ALWAYS ANSWER if chunks exist - trust that retrieval matched the query to textbook content."""

        # User prompt with context
        user_prompt = f"""Based on the following textbook content, answer this question:

**Question:** {query}

**Relevant Textbook Content:**
{context}

**Instructions:**
1. The sources above were retrieved as relevant to the question
2. Provide a concise answer using ONLY the information in these sources
3. Keep response brief (6-7 lines for definitions, concise for explanations)
4. Never add external information or make inferences beyond the text

**Your Response:**"""

        # Call OpenRouter API with best free model
        response = await client.chat.completions.create(
            model="meta-llama/llama-3.2-3b-instruct:free",  # Free tier model
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,  # Lower temperature for more focused answers
            max_tokens=400,  # Reduced to enforce brevity
        )

        answer = response.choices[0].message.content
        return answer

    except RateLimitError as e:
        # Credit exhaustion - return graceful fallback with retrieved content summary
        logger.warning(f"OpenRouter credit limit reached: {str(e)}")

        # Provide fallback using retrieved chunks
        fallback_answer = "⚠️ The AI service is temporarily unavailable due to usage limits.\n\n"
        fallback_answer += "The system retrieved relevant textbook sections, but cannot generate a full response right now.\n\n"
        fallback_answer += "**Retrieved content:**\n"

        for i, chunk in enumerate(relevant_chunks[:2], 1):  # Show first 2 chunks
            payload = chunk.get("payload", {})
            content = payload.get("content", "")[:300]  # First 300 chars
            chapter = payload.get("chapter_title", "Unknown")
            fallback_answer += f"\n[Source {i} - {chapter}]\n{content}...\n"

        fallback_answer += "\nPlease try again later."
        return fallback_answer

    except APIError as e:
        # Handle 402 Payment Required and other API errors gracefully
        if "402" in str(e) or "payment" in str(e).lower() or "credit" in str(e).lower():
            logger.warning(f"OpenRouter payment/credit error: {str(e)}")

            fallback_answer = "⚠️ The AI service is temporarily unavailable due to usage limits.\n\n"
            fallback_answer += "The system retrieved relevant textbook sections, but cannot generate a full response right now.\n\n"
            fallback_answer += "Please try again later."
            return fallback_answer
        else:
            # Other API errors - log and raise with user-friendly message
            logger.error(f"OpenRouter API error: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="The AI service encountered an error. Please try again."
            )

    except Exception as e:
        logger.error(f"Unexpected error in RAG response: {str(e)}", exc_info=True)
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

        # Step 2: Generate answer using RAG (with personalization)
        answer = await generate_rag_response(
            query=request.query,
            relevant_chunks=relevant_chunks,
            user_level=request.user_level
        )

        # Step 2.5: Translate to Urdu if requested
        if request.language == "ur":
            try:
                from src.services.translation_service import translation_service
                answer = await translation_service.translate_to_urdu(
                    text=answer,
                    preserve_formatting=True
                )
            except Exception as e:
                logger.warning(f"Translation failed, returning English: {str(e)}")
                # Continue with English answer if translation fails

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


@router.get("/credit-status")
async def check_credit_status():
    """
    Check OpenRouter API credit status.

    Returns:
        Credit availability status
    """
    try:
        from openai import AsyncOpenAI

        client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Attempt a minimal API call to check credit availability
        response = await client.chat.completions.create(
            model="meta-llama/llama-3.2-3b-instruct:free",
            messages=[{"role": "user", "content": "test"}],
            max_tokens=1
        )

        return {
            "status": "available",
            "model": "meta-llama/llama-3.2-3b-instruct:free",
            "message": "API credits available"
        }

    except Exception as e:
        error_str = str(e).lower()
        if "402" in error_str or "payment" in error_str or "credit" in error_str or "rate" in error_str:
            return {
                "status": "exhausted",
                "model": "meta-llama/llama-3.2-3b-instruct:free",
                "message": "API credits exhausted or rate limited"
            }
        else:
            logger.error(f"Credit status check failed: {str(e)}")
            return {
                "status": "unknown",
                "model": "meta-llama/llama-3.2-3b-instruct:free",
                "message": f"Unable to check status: {str(e)}"
            }
