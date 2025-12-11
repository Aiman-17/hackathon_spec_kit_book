"""
Textbook Content Ingestion Script

Ingests markdown files from frontend/docs/ into Qdrant vector database.
Creates semantic chunks with embeddings for RAG-powered chatbot.
"""

import sys
import asyncio
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.document_ingestion import DocumentIngestionService
from src.services.embedding_pipeline import EmbeddingPipelineService
from src.services.qdrant_manager import qdrant_manager
from src.models.chapter import ChapterMetadata

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def ingest_intro():
    """Ingest intro.md file"""
    logger.info("="*60)
    logger.info("Starting Textbook Content Ingestion")
    logger.info("="*60)

    # Initialize services
    doc_service = DocumentIngestionService()
    embedding_service = EmbeddingPipelineService()

    # Initialize Qdrant
    logger.info("Connecting to Qdrant...")
    qdrant_manager.initialize()
    logger.info("✓ Qdrant connected")

    # Process intro.md
    intro_file = Path(__file__).parent.parent.parent / 'frontend' / 'docs' / 'intro.md'

    if not intro_file.exists():
        logger.error(f"File not found: {intro_file}")
        return

    logger.info(f"\nProcessing: {intro_file.name}")
    logger.info("-"*60)

    # Read content
    with open(intro_file, 'r', encoding='utf-8') as f:
        content = f.read()

    logger.info(f"Content length: {len(content)} characters")

    # Create chapter metadata
    # Note: intro.md is treated as module-1, chapter 1 to satisfy validation
    chapter_metadata = ChapterMetadata(
        module_id='module-1',
        module_title='Introduction',
        chapter_id='intro',
        chapter_title='Welcome to Physical AI & Humanoid Robotics',
        chapter_number=1
    )

    # Process into chunks
    logger.info("\nChunking document...")
    chunks = doc_service.process_document(content, chapter_metadata)

    logger.info(f"✓ Created {len(chunks)} chunks")

    # Show chunk statistics
    if chunks:
        total_tokens = sum(chunk.chunk_metadata.token_count for chunk in chunks)
        avg_tokens = total_tokens / len(chunks)

        logger.info(f"\nChunk Statistics:")
        logger.info(f"  Total chunks: {len(chunks)}")
        logger.info(f"  Total tokens: {total_tokens}")
        logger.info(f"  Average tokens/chunk: {avg_tokens:.0f}")
        logger.info(f"  Min tokens: {min(c.chunk_metadata.token_count for c in chunks)}")
        logger.info(f"  Max tokens: {max(c.chunk_metadata.token_count for c in chunks)}")

        logger.info(f"\nFirst chunk preview:")
        logger.info(f"  Heading: {chunks[0].chunk_metadata.heading}")
        logger.info(f"  Tokens: {chunks[0].chunk_metadata.token_count}")
        logger.info(f"  Content: {chunks[0].content[:150]}...")

    # Generate embeddings and store in Qdrant
    logger.info(f"\nGenerating embeddings and storing in Qdrant...")
    logger.info("(This may take a minute...)")

    stored_count = await embedding_service.embed_and_store_chunks(chunks)

    logger.info(f"✓ Stored {stored_count} vectors in Qdrant")

    # Get embedding statistics
    stats = embedding_service.get_usage_stats()

    logger.info(f"\n" + "="*60)
    logger.info("Ingestion Complete!")
    logger.info("="*60)
    logger.info(f"Embedding Stats:")
    logger.info(f"  Total requests: {stats['total_requests']}")
    logger.info(f"  Total tokens: {stats['total_tokens']}")
    logger.info(f"  Estimated cost: ${stats['estimated_cost_usd']:.4f}")
    logger.info(f"\nQdrant Collection:")
    logger.info(f"  Collection: textbook_chapters")
    logger.info(f"  Vectors: {stored_count}")
    logger.info(f"\n✓ Chatbot is now ready to answer questions about the intro!")

    # Cleanup
    qdrant_manager.close()


async def ingest_all_modules():
    """Ingest all textbook modules"""
    logger.info("="*60)
    logger.info("Ingesting All Textbook Modules")
    logger.info("="*60)

    # Initialize services
    doc_service = DocumentIngestionService()
    embedding_service = EmbeddingPipelineService()

    # Initialize Qdrant
    logger.info("Connecting to Qdrant...")
    qdrant_manager.initialize()
    logger.info("✓ Qdrant connected\n")

    # Find all markdown files in frontend/docs/
    docs_dir = Path(__file__).parent.parent.parent / 'frontend' / 'docs'

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return

    # Get all .md files (excluding intro.md if already processed)
    md_files = list(docs_dir.rglob('*.md'))
    logger.info(f"Found {len(md_files)} markdown files")

    total_chunks = 0
    total_tokens = 0

    for md_file in md_files:
        logger.info(f"\nProcessing: {md_file.relative_to(docs_dir)}")
        logger.info("-"*60)

        try:
            # Extract module and chapter ID from path
            parts = md_file.relative_to(docs_dir).parts

            if len(parts) == 1:
                # Top-level file (like intro.md)
                module_id = 'introduction'
                chapter_id = md_file.stem
            else:
                # Module file (like module-1/chapter-01.md)
                module_id = parts[0]
                chapter_id = md_file.stem

            # Read content
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title from first H1
            import re
            title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
            chapter_title = title_match.group(1) if title_match else chapter_id

            # Create metadata
            chapter_metadata = ChapterMetadata(
                module_id=module_id,
                module_title=f"Module {module_id.split('-')[1]}" if 'module-' in module_id else "Introduction",
                chapter_id=chapter_id,
                chapter_title=chapter_title,
                chapter_number=int(chapter_id.split('-')[1]) if '-' in chapter_id else 0
            )

            # Process document
            chunks = doc_service.process_document(content, chapter_metadata)

            # Store chunks
            stored_count = await embedding_service.embed_and_store_chunks(chunks)

            total_chunks += stored_count
            total_tokens += sum(c.chunk_metadata.token_count for c in chunks)

            logger.info(f"✓ Stored {stored_count} chunks ({sum(c.chunk_metadata.token_count for c in chunks)} tokens)")

        except Exception as e:
            logger.error(f"✗ Failed to process {md_file.name}: {str(e)}")
            continue

    # Final statistics
    stats = embedding_service.get_usage_stats()

    logger.info(f"\n" + "="*60)
    logger.info("All Modules Ingested!")
    logger.info("="*60)
    logger.info(f"Total Statistics:")
    logger.info(f"  Files processed: {len(md_files)}")
    logger.info(f"  Total chunks: {total_chunks}")
    logger.info(f"  Total tokens: {total_tokens}")
    logger.info(f"  Embedding requests: {stats['total_requests']}")
    logger.info(f"  Estimated cost: ${stats['estimated_cost_usd']:.4f}")
    logger.info(f"\n✓ Chatbot is ready to answer questions about the entire textbook!")

    # Cleanup
    qdrant_manager.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ingest textbook content into Qdrant")
    parser.add_argument(
        '--all',
        action='store_true',
        help='Ingest all modules (default: only intro.md)'
    )

    args = parser.parse_args()

    if args.all:
        asyncio.run(ingest_all_modules())
    else:
        asyncio.run(ingest_intro())
