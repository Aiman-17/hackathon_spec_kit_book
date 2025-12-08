"""
Translation Service

Provides English-to-Urdu translation for textbook content using OpenAI.
Includes caching to reduce API costs.
"""

import logging
import hashlib
from typing import Optional, Dict, Any
from datetime import datetime
from openai import AsyncOpenAI

from src.config import settings
from src.services.postgres_manager import postgres_manager

logger = logging.getLogger(__name__)


class TranslationService:
    """
    Handles translation of textbook content to Urdu using OpenAI.

    Features:
    - English to Urdu translation
    - Database caching to reduce API calls
    - Batch translation support
    - Quality control
    """

    def __init__(self):
        self.client = AsyncOpenAI(
            api_key=settings.openai_api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        self.total_translations = 0
        self.cache_hits = 0
        self.cache_misses = 0

    @staticmethod
    def _generate_content_hash(text: str) -> str:
        """Generate SHA256 hash of text for cache lookup."""
        return hashlib.sha256(text.encode('utf-8')).hexdigest()

    async def _check_cache(
        self,
        source_text: str,
        source_lang: str = "en",
        target_lang: str = "ur"
    ) -> Optional[str]:
        """
        Check if translation exists in cache.

        Args:
            source_text: Text to translate
            source_lang: Source language code
            target_lang: Target language code

        Returns:
            Cached translation if found, None otherwise
        """
        content_hash = self._generate_content_hash(source_text)

        try:
            query = """
                SELECT translated_text, access_count
                FROM translation_cache
                WHERE source_language = $1
                  AND target_language = $2
                  AND content_hash = $3
            """

            result = await postgres_manager.fetch_one(
                query,
                source_lang,
                target_lang,
                content_hash
            )

            if result:
                # Update access count and last_accessed
                update_query = """
                    UPDATE translation_cache
                    SET access_count = access_count + 1,
                        last_accessed = CURRENT_TIMESTAMP
                    WHERE content_hash = $1
                """
                await postgres_manager.execute(update_query, content_hash)

                self.cache_hits += 1
                logger.debug(f"Cache hit for translation (hash: {content_hash[:8]}...)")
                return result['translated_text']

            self.cache_misses += 1
            return None

        except Exception as e:
            logger.error(f"Cache lookup failed: {str(e)}")
            return None

    async def _store_in_cache(
        self,
        source_text: str,
        translated_text: str,
        source_lang: str = "en",
        target_lang: str = "ur",
        service: str = "openai"
    ) -> None:
        """
        Store translation in cache.

        Args:
            source_text: Original text
            translated_text: Translated text
            source_lang: Source language code
            target_lang: Target language code
            service: Translation service used
        """
        content_hash = self._generate_content_hash(source_text)

        try:
            query = """
                INSERT INTO translation_cache (
                    source_language,
                    target_language,
                    content_hash,
                    source_text,
                    translated_text,
                    translation_service,
                    access_count
                ) VALUES ($1, $2, $3, $4, $5, $6, 1)
                ON CONFLICT (source_language, target_language, content_hash)
                DO UPDATE SET
                    translated_text = EXCLUDED.translated_text,
                    access_count = translation_cache.access_count + 1,
                    last_accessed = CURRENT_TIMESTAMP
            """

            await postgres_manager.execute(
                query,
                source_lang,
                target_lang,
                content_hash,
                source_text,
                translated_text,
                service
            )

            logger.debug(f"Stored translation in cache (hash: {content_hash[:8]}...)")

        except Exception as e:
            logger.error(f"Failed to cache translation: {str(e)}")

    async def translate_to_urdu(
        self,
        text: str,
        preserve_formatting: bool = True,
        preserve_code: bool = True
    ) -> str:
        """
        Translate English text to Urdu using OpenAI.

        Args:
            text: English text to translate
            preserve_formatting: Keep markdown formatting
            preserve_code: Don't translate code blocks

        Returns:
            Urdu translation
        """
        # Check cache first
        cached = await self._check_cache(text, "en", "ur")
        if cached:
            return cached

        try:
            # Construct prompt for high-quality technical translation
            system_prompt = """You are an expert translator specializing in technical and educational content from English to Urdu.

Translation Guidelines:
- Translate technical content about Physical AI, Robotics, and Programming
- Preserve technical terminology where appropriate (e.g., "ROS 2", "Python", "sensor")
- Maintain professional academic tone
- Keep markdown formatting intact (headings, lists, code blocks, links)
- DO NOT translate code blocks (content between ``` markers)
- DO NOT translate URLs or file paths
- Translate naturally and idiomatically for Urdu speakers
- Use proper Urdu script (not Roman Urdu)

Output ONLY the translation, no explanations."""

            user_prompt = f"""Translate this technical textbook content to Urdu:

{text}"""

            response = await self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,  # Lower temperature for consistent translation
                max_tokens=2000,
            )

            translated_text = response.choices[0].message.content.strip()

            # Store in cache
            await self._store_in_cache(text, translated_text, "en", "ur", "openai")

            self.total_translations += 1

            logger.info(f"Translated {len(text)} chars to Urdu ({response.usage.total_tokens} tokens)")

            return translated_text

        except Exception as e:
            logger.error(f"Translation failed: {str(e)}")
            raise

    async def translate_chapter(
        self,
        chapter_content: str,
        chapter_id: str
    ) -> Dict[str, Any]:
        """
        Translate an entire chapter to Urdu.

        Args:
            chapter_content: Full chapter markdown content
            chapter_id: Chapter identifier

        Returns:
            Dictionary with translated content and metadata
        """
        logger.info(f"Translating chapter: {chapter_id}")

        try:
            # Split chapter into sections for better translation quality
            # Simple split by ## headers
            sections = []
            current_section = []

            for line in chapter_content.split('\n'):
                if line.startswith('## ') and current_section:
                    sections.append('\n'.join(current_section))
                    current_section = [line]
                else:
                    current_section.append(line)

            if current_section:
                sections.append('\n'.join(current_section))

            # Translate each section
            translated_sections = []
            for i, section in enumerate(sections, 1):
                logger.debug(f"Translating section {i}/{len(sections)}")
                translated = await self.translate_to_urdu(section)
                translated_sections.append(translated)

            translated_content = '\n\n'.join(translated_sections)

            return {
                "chapter_id": chapter_id,
                "original_length": len(chapter_content),
                "translated_length": len(translated_content),
                "sections_count": len(sections),
                "translated_content": translated_content,
                "cache_hits": self.cache_hits,
                "cache_misses": self.cache_misses,
            }

        except Exception as e:
            logger.error(f"Chapter translation failed for {chapter_id}: {str(e)}")
            raise

    def get_stats(self) -> Dict[str, Any]:
        """
        Get translation service statistics.

        Returns:
            Dictionary with usage stats
        """
        total_requests = self.cache_hits + self.cache_misses
        cache_hit_rate = (self.cache_hits / total_requests * 100) if total_requests > 0 else 0

        return {
            "total_translations": self.total_translations,
            "cache_hits": self.cache_hits,
            "cache_misses": self.cache_misses,
            "cache_hit_rate": round(cache_hit_rate, 2),
            "total_requests": total_requests,
        }


# Global instance
translation_service = TranslationService()
