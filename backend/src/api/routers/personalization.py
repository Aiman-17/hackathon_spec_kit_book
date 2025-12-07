"""
Personalization API Router

Endpoints for Phase 4: Personalized learning experience, translation, and recommendations.
"""

import logging
from typing import List, Optional, Dict, Any
from datetime import datetime
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from src.services.personalization_service import personalization_service
from src.services.translation_service import translation_service
from src.services.postgres_manager import postgres_manager

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/personalization", tags=["Personalization"])


# ==========================================
# Request/Response Models
# ==========================================

class UserProfileUpdate(BaseModel):
    """Update user profile"""
    skill_level: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced|expert)$")
    technical_background: Optional[str] = None
    learning_goals: Optional[str] = None
    preferred_language: Optional[str] = Field(None, pattern="^(en|ur)$")


class LearningProgressUpdate(BaseModel):
    """Update learning progress for a chapter"""
    chapter_id: str
    module_id: str
    completion_percentage: int = Field(0, ge=0, le=100)
    time_spent_seconds: int = Field(0, ge=0)
    last_position: Optional[str] = None
    status: str = Field("in_progress", pattern="^(not_started|in_progress|completed|skipped)$")


class TranslationRequest(BaseModel):
    """Request translation"""
    text: str = Field(..., min_length=1, max_length=5000)
    target_language: str = Field("ur", pattern="^(ur)$")
    preserve_formatting: bool = True


class RecommendationRequest(BaseModel):
    """Request personalized recommendations"""
    count: int = Field(5, ge=1, le=10)


class RecommendationResponse(BaseModel):
    """Recommendation response"""
    chapter_id: str
    module_id: str
    reason: str
    recommendation_type: str
    relevance_score: float
    priority: int


class SkillGapAnalysis(BaseModel):
    """Skill gap analysis response"""
    skill_gaps: List[str]
    strengths: List[str]
    recommended_focus_areas: List[str]
    learning_pace: str
    engagement_level: str


# ==========================================
# User Profile Endpoints
# ==========================================

@router.get("/profile/{user_id}")
async def get_user_profile(user_id: str):
    """
    Get user profile with preferences.

    Args:
        user_id: User UUID

    Returns:
        User profile data
    """
    try:
        query = """
            SELECT up.*, u.email, u.created_at as user_created_at
            FROM user_profiles up
            JOIN users u ON up.user_id = u.id
            WHERE up.user_id = $1
        """

        profile = await postgres_manager.fetch_one(query, user_id)

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )

        return dict(profile)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to get user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve profile: {str(e)}"
        )


@router.put("/profile/{user_id}")
async def update_user_profile(user_id: str, profile: UserProfileUpdate):
    """
    Update user profile.

    Args:
        user_id: User UUID
        profile: Profile update data

    Returns:
        Updated profile
    """
    try:
        # Build dynamic update query
        updates = []
        values = []
        param_count = 1

        for field, value in profile.model_dump(exclude_unset=True).items():
            if value is not None:
                updates.append(f"{field} = ${param_count}")
                values.append(value)
                param_count += 1

        if not updates:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="No fields to update"
            )

        values.append(user_id)
        query = f"""
            UPDATE user_profiles
            SET {', '.join(updates)}, updated_at = CURRENT_TIMESTAMP
            WHERE user_id = ${param_count}
            RETURNING *
        """

        updated = await postgres_manager.fetch_one(query, *values)

        if not updated:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )

        return dict(updated)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to update profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update profile: {str(e)}"
        )


# ==========================================
# Learning Progress Endpoints
# ==========================================

@router.get("/progress/{user_id}")
async def get_learning_progress(user_id: str, limit: int = 20):
    """
    Get user's learning progress across all chapters.

    Args:
        user_id: User UUID
        limit: Maximum number of records to return

    Returns:
        List of progress records
    """
    try:
        query = """
            SELECT *
            FROM learning_progress
            WHERE user_id = $1
            ORDER BY updated_at DESC
            LIMIT $2
        """

        progress = await postgres_manager.fetch_all(query, user_id, limit)

        return [dict(p) for p in progress]

    except Exception as e:
        logger.error(f"Failed to get learning progress: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve progress: {str(e)}"
        )


@router.post("/progress/{user_id}")
async def update_learning_progress(user_id: str, progress: LearningProgressUpdate):
    """
    Update learning progress for a chapter.

    Args:
        user_id: User UUID
        progress: Progress update data

    Returns:
        Updated progress record
    """
    try:
        # Mark as completed if 100%
        completed_at = "CURRENT_TIMESTAMP" if progress.completion_percentage == 100 else "NULL"

        query = f"""
            INSERT INTO learning_progress (
                user_id,
                chapter_id,
                module_id,
                completion_percentage,
                time_spent_seconds,
                last_position,
                status,
                completed_at
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, {completed_at})
            ON CONFLICT (user_id, chapter_id)
            DO UPDATE SET
                completion_percentage = EXCLUDED.completion_percentage,
                time_spent_seconds = learning_progress.time_spent_seconds + EXCLUDED.time_spent_seconds,
                last_position = EXCLUDED.last_position,
                status = EXCLUDED.status,
                completed_at = {completed_at},
                updated_at = CURRENT_TIMESTAMP
            RETURNING *
        """

        updated = await postgres_manager.fetch_one(
            query,
            user_id,
            progress.chapter_id,
            progress.module_id,
            progress.completion_percentage,
            progress.time_spent_seconds,
            progress.last_position,
            progress.status
        )

        return dict(updated)

    except Exception as e:
        logger.error(f"Failed to update progress: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update progress: {str(e)}"
        )


# ==========================================
# Recommendation Endpoints
# ==========================================

@router.post("/recommendations/{user_id}", response_model=List[RecommendationResponse])
async def get_recommendations(user_id: str, request: RecommendationRequest):
    """
    Get AI-powered personalized chapter recommendations.

    Uses OpenAI to analyze user profile and progress to recommend next chapters.

    Args:
        user_id: User UUID
        request: Recommendation request parameters

    Returns:
        List of recommended chapters with reasoning
    """
    try:
        recommendations = await personalization_service.generate_recommendations(
            user_id=user_id,
            count=request.count
        )

        return recommendations

    except Exception as e:
        logger.error(f"Failed to generate recommendations: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate recommendations: {str(e)}"
        )


@router.get("/skill-gaps/{user_id}", response_model=SkillGapAnalysis)
async def analyze_skill_gaps(user_id: str):
    """
    Analyze user's skill gaps using AI.

    Uses OpenAI to analyze learning progress and identify skill gaps,
    strengths, and recommended focus areas.

    Args:
        user_id: User UUID

    Returns:
        Skill gap analysis
    """
    try:
        analysis = await personalization_service.analyze_skill_gaps(user_id)

        return analysis

    except Exception as e:
        logger.error(f"Failed to analyze skill gaps: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to analyze skill gaps: {str(e)}"
        )


# ==========================================
# Translation Endpoints
# ==========================================

@router.post("/translate")
async def translate_text(request: TranslationRequest):
    """
    Translate text to Urdu using OpenAI.

    Includes caching to reduce API costs for repeated translations.

    Args:
        request: Translation request with text and parameters

    Returns:
        Translated text
    """
    try:
        translated = await translation_service.translate_to_urdu(
            text=request.text,
            preserve_formatting=request.preserve_formatting
        )

        return {
            "original_text": request.text,
            "translated_text": translated,
            "source_language": "en",
            "target_language": request.target_language,
            "timestamp": datetime.utcnow().isoformat()
        }

    except Exception as e:
        logger.error(f"Translation failed: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )


@router.get("/translation/stats")
async def get_translation_stats():
    """
    Get translation service statistics.

    Returns:
        Cache hit rate, total translations, etc.
    """
    try:
        stats = translation_service.get_stats()

        return {
            "status": "healthy",
            "service": "translation",
            **stats
        }

    except Exception as e:
        logger.error(f"Failed to get translation stats: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve stats: {str(e)}"
        )


@router.get("/health")
async def personalization_health():
    """
    Health check for personalization service.

    Returns:
        Service status and configuration
    """
    return {
        "status": "healthy",
        "service": "personalization",
        "features": {
            "recommendations": True,
            "skill_gap_analysis": True,
            "translation_urdu": True,
            "learning_progress_tracking": True,
            "ai_powered": True,
        },
        "ai_model": "gpt-4-turbo-preview via OpenRouter"
    }
