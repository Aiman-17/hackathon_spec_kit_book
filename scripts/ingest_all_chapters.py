#!/usr/bin/env python3
"""
Ingest all textbook chapters into Qdrant.
"""

import asyncio
import logging
from pathlib import Path
import sys

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.document_ingestion import DocumentIngestionService
from src.services.embedding_pipeline import EmbeddingPipelineService
from src.services.qdrant_manager import qdrant_manager
from src.models.chapter import ChapterMetadata

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def ingest_all_chapters():
    """Ingest all chapters from the frontend/docs directory."""

    # Initialize services
    doc_service = DocumentIngestionService()
    embedding_service = EmbeddingPipelineService()

    # Initialize Qdrant
    logger.info("Initializing Qdrant...")
    qdrant_manager.initialize()

    # Find all markdown files
    docs_dir = Path(__file__).parent.parent.parent / 'frontend' / 'docs'

    # Get all module directories
    module_dirs = sorted([d for d in docs_dir.iterdir() if d.is_dir() and d.name.startswith('module-')])

    total_chunks = 0
    total_tokens = 0

    # Skip intro.md (it's just the landing page with hero section, not course content)
    logger.info("Skipping intro.md (landing page only)")

    # Process all module chapters
    for module_dir in module_dirs:
        module_num = module_dir.name.split('-')[1]
        module_id = f'module-{module_num}'
        module_title = f'Module {module_num}'

        # Get all markdown files in this module
        chapter_files = sorted(module_dir.glob('*.md'))

        for chapter_file in chapter_files:
            logger.info(f"\n{'='*60}")
            logger.info(f"Processing: {module_dir.name}/{chapter_file.name}")
            logger.info(f"{'='*60}")

            # Read file
            with open(chapter_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract chapter number and title
            filename = chapter_file.stem
            parts = filename.split('-', 1)

            if len(parts) == 2 and parts[0].isdigit():
                chapter_number = int(parts[0])
                chapter_id = filename
            else:
                chapter_number = 1
                chapter_id = filename

            # Extract title from first H1
            import re
            title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
            chapter_title = title_match.group(1) if title_match else filename.replace('-', ' ').title()

            # Create metadata
            chapter_metadata = ChapterMetadata(
                module_id=module_id,
                module_title=module_title,
                chapter_id=chapter_id,
                chapter_title=chapter_title,
                chapter_number=chapter_number
            )

            try:
                # Process document
                chunks = doc_service.process_document(content, chapter_metadata)

                if not chunks:
                    logger.warning(f"No chunks generated for {chapter_file.name}")
                    continue

                # Store in Qdrant
                stored_count = await embedding_service.embed_and_store_chunks(chunks)

                total_chunks += stored_count
                total_tokens += sum(c.chunk_metadata.token_count for c in chunks)

                logger.info(f"✓ Stored {stored_count} chunks from {chapter_file.name}")

            except Exception as e:
                logger.error(f"Failed to process {chapter_file.name}: {str(e)}")
                continue

    # Get final stats
    stats = embedding_service.get_usage_stats()

    logger.info(f"\n{'='*60}")
    logger.info("INGESTION COMPLETE")
    logger.info(f"{'='*60}")
    logger.info(f"Total chunks stored: {total_chunks}")
    logger.info(f"Total tokens: {total_tokens}")
    logger.info(f"API requests: {stats['total_requests']}")
    logger.info(f"Tokens used: {stats['total_tokens']}")
    logger.info(f"Estimated cost: ${stats['estimated_cost_usd']:.4f}")
    logger.info(f"{'='*60}")

    # Cleanup
    qdrant_manager.close()


if __name__ == '__main__':
    asyncio.run(ingest_all_chapters())
