"""
Translation API Router

Endpoints for translating textbook content to Urdu.
"""

import logging
from typing import Optional
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from src.services.translation_service import translation_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/translate", tags=["Translation"])


# ==========================================
# Request/Response Models
# ==========================================

class TranslateRequest(BaseModel):
    """Request model for translation endpoint"""
    text: str = Field(..., description="Text to translate", min_length=1, max_length=50000)
    preserve_formatting: bool = Field(True, description="Preserve markdown formatting")
    preserve_code: bool = Field(True, description="Don't translate code blocks")


class TranslateResponse(BaseModel):
    """Response model for translation endpoint"""
    original_text: str
    translated_text: str
    original_length: int
    translated_length: int
    cache_hit: bool
    language_pair: str = "en->ur"


class TranslateChapterRequest(BaseModel):
    """Request model for chapter translation"""
    chapter_content: str = Field(..., description="Full chapter markdown content")
    chapter_id: str = Field(..., description="Chapter identifier")


class TranslateChapterResponse(BaseModel):
    """Response model for chapter translation"""
    chapter_id: str
    translated_content: str
    original_length: int
    translated_length: int
    sections_count: int
    cache_hits: int
    cache_misses: int


# ==========================================
# API Endpoints
# ==========================================

@router.post("/urdu", response_model=TranslateResponse)
async def translate_to_urdu(request: TranslateRequest):
    """
    Translate English text to Urdu.

    This endpoint:
    1. Checks translation cache for existing translation
    2. If not cached, translates using OpenAI GPT-4
    3. Preserves markdown formatting and code blocks
    4. Stores result in cache for future requests

    Args:
        request: Translation request with text and options

    Returns:
        TranslateResponse with translated text and metadata
    """
    try:
        logger.info(f"Translating {len(request.text)} characters to Urdu")

        # Check cache before translation
        original_cache_hits = translation_service.cache_hits

        # Translate
        translated_text = await translation_service.translate_to_urdu(
            text=request.text,
            preserve_formatting=request.preserve_formatting,
            preserve_code=request.preserve_code
        )

        # Determine if this was a cache hit
        cache_hit = translation_service.cache_hits > original_cache_hits

        return TranslateResponse(
            original_text=request.text,
            translated_text=translated_text,
            original_length=len(request.text),
            translated_length=len(translated_text),
            cache_hit=cache_hit,
            language_pair="en->ur"
        )

    except Exception as e:
        logger.error(f"Translation failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )


@router.post("/chapter", response_model=TranslateChapterResponse)
async def translate_chapter(request: TranslateChapterRequest):
    """
    Translate an entire chapter to Urdu.

    This endpoint:
    1. Splits chapter into sections by H2 headers
    2. Translates each section separately for better quality
    3. Caches each section translation
    4. Reassembles translated chapter

    Args:
        request: Chapter translation request

    Returns:
        TranslateChapterResponse with translated chapter and stats
    """
    try:
        logger.info(f"Translating chapter: {request.chapter_id}")

        result = await translation_service.translate_chapter(
            chapter_content=request.chapter_content,
            chapter_id=request.chapter_id
        )

        return TranslateChapterResponse(
            chapter_id=result["chapter_id"],
            translated_content=result["translated_content"],
            original_length=result["original_length"],
            translated_length=result["translated_length"],
            sections_count=result["sections_count"],
            cache_hits=result["cache_hits"],
            cache_misses=result["cache_misses"]
        )

    except Exception as e:
        logger.error(f"Chapter translation failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Chapter translation failed: {str(e)}"
        )


@router.get("/stats")
async def translation_stats():
    """
    Get translation service statistics.

    Returns:
        Dictionary with usage stats including cache hit rate
    """
    return translation_service.get_stats()


@router.get("/health")
async def translation_health():
    """
    Health check for translation service.

    Returns:
        Service status
    """
    return {
        "status": "healthy",
        "service": "translation",
        "supported_languages": ["en->ur"],
        "features": {
            "caching": True,
            "markdown_preservation": True,
            "code_preservation": True,
        }
    }
