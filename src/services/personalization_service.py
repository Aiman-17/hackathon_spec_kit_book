"""
Personalization Service

Generates personalized content recommendations using OpenAI based on user profile,
learning progress, and interaction history.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from openai import AsyncOpenAI

from src.config import settings
from src.services.postgres_manager import postgres_manager

logger = logging.getLogger(__name__)


class PersonalizationService:
    """
    AI-powered personalization engine using OpenAI Chat.

    Features:
    - Personalized chapter recommendations
    - Adaptive difficulty adjustment
    - Learning path generation
    - Skill gap analysis
    """

    def __init__(self):
        self.client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

    async def get_user_context(self, user_id: str) -> Dict[str, Any]:
        """
        Gather comprehensive user context for personalization.

        Args:
            user_id: User UUID

        Returns:
            Dictionary with user profile, progress, and preferences
        """
        try:
            # Get user profile
            profile_query = """
                SELECT up.skill_level, up.technical_background, up.learning_goals,
                       up.preferred_language, u.email
                FROM user_profiles up
                JOIN users u ON up.user_id = u.id
                WHERE up.user_id = $1
            """
            profile = await postgres_manager.fetch_one(profile_query, user_id)

            # Get learning progress
            progress_query = """
                SELECT chapter_id, module_id, completion_percentage,
                       status, questions_asked, time_spent_seconds
                FROM learning_progress
                WHERE user_id = $1
                ORDER BY updated_at DESC
                LIMIT 20
            """
            progress = await postgres_manager.fetch_all(progress_query, user_id)

            # Get preferences
            prefs_query = """
                SELECT preferred_difficulty, preferred_content_types,
                       code_language_preference, show_bilingual
                FROM user_preferences
                WHERE user_id = $1
            """
            preferences = await postgres_manager.fetch_one(prefs_query, user_id)

            return {
                "profile": dict(profile) if profile else {},
                "progress": [dict(p) for p in progress] if progress else [],
                "preferences": dict(preferences) if preferences else {},
            }

        except Exception as e:
            logger.error(f"Failed to get user context: {str(e)}")
            return {"profile": {}, "progress": [], "preferences": {}}

    async def generate_recommendations(
        self,
        user_id: str,
        count: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Generate personalized chapter recommendations using OpenAI.

        Args:
            user_id: User UUID
            count: Number of recommendations to generate

        Returns:
            List of recommended chapters with reasoning
        """
        logger.info(f"Generating {count} recommendations for user {user_id}")

        try:
            # Get user context
            context = await self.get_user_context(user_id)

            # Get all available chapters
            chapters_query = """
                SELECT DISTINCT chapter_id, module_id
                FROM learning_progress
                WHERE user_id != $1
                UNION
                SELECT chapter_id, module_id
                FROM learning_progress
                WHERE user_id = $1 AND status != 'completed'
            """

            # Build prompt for OpenAI
            system_prompt = """You are an AI educational advisor specializing in Physical AI and Humanoid Robotics.
Your role is to recommend the most appropriate chapters for students based on their profile and progress.

Consider:
- Student's skill level and background
- Current progress and completion status
- Learning goals and preferences
- Logical chapter progression
- Skill gap identification

Respond with a JSON array of recommendations in this format:
[
  {
    "chapter_id": "chapter-identifier",
    "module_id": "module-X",
    "reason": "Why this chapter is recommended",
    "recommendation_type": "next_in_sequence|related_topic|skill_gap|popular",
    "relevance_score": 0.0-1.0,
    "priority": 1-5
  }
]"""

            user_prompt = f"""Based on this student profile, recommend {count} chapters:

**Student Profile:**
- Skill Level: {context['profile'].get('skill_level', 'beginner')}
- Technical Background: {context['profile'].get('technical_background', 'None')}
- Learning Goals: {context['profile'].get('learning_goals', 'None')}

**Recent Progress:**
{self._format_progress(context['progress'][:5])}

**Preferences:**
- Difficulty: {context['preferences'].get('preferred_difficulty', 'adaptive')}
- Content Types: {context['preferences'].get('preferred_content_types', ['text', 'code'])}

Recommend {count} chapters that will help this student progress most effectively."""

            response = await self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=1500,
                response_format={"type": "json_object"}
            )

            # Parse recommendations
            import json
            recommendations_data = json.loads(response.choices[0].message.content)

            # Handle both array and object responses
            if isinstance(recommendations_data, dict):
                recommendations = recommendations_data.get('recommendations', [])
            else:
                recommendations = recommendations_data

            # Store recommendations in database
            for rec in recommendations:
                await self._store_recommendation(user_id, rec)

            logger.info(f"Generated {len(recommendations)} recommendations")

            return recommendations

        except Exception as e:
            logger.error(f"Failed to generate recommendations: {str(e)}")
            return []

    def _format_progress(self, progress: List[Dict]) -> str:
        """Format progress for prompt."""
        if not progress:
            return "No progress yet"

        formatted = []
        for p in progress:
            formatted.append(
                f"- {p.get('chapter_id', 'Unknown')}: "
                f"{p.get('completion_percentage', 0)}% complete "
                f"({p.get('status', 'not_started')})"
            )
        return '\n'.join(formatted)

    async def _store_recommendation(
        self,
        user_id: str,
        recommendation: Dict[str, Any]
    ) -> None:
        """Store recommendation in database."""
        try:
            query = """
                INSERT INTO recommended_content (
                    user_id,
                    chapter_id,
                    reason,
                    recommendation_type,
                    relevance_score,
                    priority,
                    status
                ) VALUES ($1, $2, $3, $4, $5, $6, 'pending')
                ON CONFLICT DO NOTHING
            """

            await postgres_manager.execute(
                query,
                user_id,
                recommendation['chapter_id'],
                recommendation.get('reason', ''),
                recommendation.get('recommendation_type', 'related_topic'),
                recommendation.get('relevance_score', 0.5),
                recommendation.get('priority', 3)
            )

        except Exception as e:
            logger.error(f"Failed to store recommendation: {str(e)}")

    async def analyze_skill_gaps(
        self,
        user_id: str
    ) -> Dict[str, Any]:
        """
        Analyze user's skill gaps using OpenAI.

        Args:
            user_id: User UUID

        Returns:
            Dictionary with skill gap analysis and recommendations
        """
        logger.info(f"Analyzing skill gaps for user {user_id}")

        try:
            context = await self.get_user_context(user_id)

            system_prompt = """You are an AI educational analyst specializing in robotics education.
Analyze the student's progress and identify skill gaps or areas that need reinforcement.

Respond with JSON:
{
  "skill_gaps": ["list of identified skill gaps"],
  "strengths": ["list of student strengths"],
  "recommended_focus_areas": ["areas to focus on next"],
  "learning_pace": "slow|moderate|fast",
  "engagement_level": "low|medium|high"
}"""

            user_prompt = f"""Analyze this student's skill gaps:

**Profile:**
- Skill Level: {context['profile'].get('skill_level', 'beginner')}
- Background: {context['profile'].get('technical_background', 'None')}
- Goals: {context['profile'].get('learning_goals', 'None')}

**Progress Summary:**
- Completed chapters: {sum(1 for p in context['progress'] if p.get('status') == 'completed')}
- In progress: {sum(1 for p in context['progress'] if p.get('status') == 'in_progress')}
- Average completion: {sum(p.get('completion_percentage', 0) for p in context['progress']) / max(len(context['progress']), 1):.1f}%
- Total questions asked: {sum(p.get('questions_asked', 0) for p in context['progress'])}

Provide skill gap analysis."""

            response = await self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.6,
                max_tokens=1000,
                response_format={"type": "json_object"}
            )

            import json
            analysis = json.loads(response.choices[0].message.content)

            logger.info(f"Skill gap analysis complete")

            return analysis

        except Exception as e:
            logger.error(f"Skill gap analysis failed: {str(e)}")
            return {
                "skill_gaps": [],
                "strengths": [],
                "recommended_focus_areas": [],
                "learning_pace": "unknown",
                "engagement_level": "unknown"
            }


# Global instance
personalization_service = PersonalizationService()
