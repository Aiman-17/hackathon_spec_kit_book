#!/usr/bin/env python3
"""
Qdrant Vector Database Ingestion Pipeline

Processes textbook markdown files into semantic chunks and ingests them
into Qdrant Cloud with OpenAI embeddings for RAG-powered Q&A.

Features:
- Semantic chunking by H2/H3 headers
- 500-1000 token overlap between chunks
- OpenAI text-embedding-3-small (1536 dimensions)
- Batch processing with progress tracking
- Metadata extraction (module, chapter, section, keywords)

Usage:
    python scripts/ingest-embeddings.py --all
    python scripts/ingest-embeddings.py --module 1
    python scripts/ingest-embeddings.py --chapter-id module-1/01-introduction-to-physical-ai
    python scripts/ingest-embeddings.py --reset  # Reset collection
"""

import argparse
import asyncio
import logging
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from datetime import datetime

import tiktoken
from dotenv import load_dotenv
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue
)

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ==========================================
# Configuration
# ==========================================

# API Keys
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Validate environment
if not all([OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY]):
    logger.error("Missing required environment variables")
    logger.error("Required: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY")
    sys.exit(1)

# Initialize clients
openai_client = AsyncOpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Project paths
REPO_ROOT = Path(__file__).parent.parent
DOCS_DIR = REPO_ROOT / "frontend" / "docs"

# Embedding configuration
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSIONS = 1536
COLLECTION_NAME = "textbook_chapters"

# Chunking configuration
MIN_CHUNK_SIZE = 500  # tokens
MAX_CHUNK_SIZE = 1000  # tokens
OVERLAP_SIZE = 200  # tokens

# Tiktoken encoder
encoder = tiktoken.encoding_for_model("gpt-3.5-turbo")


# ==========================================
# Data Models
# ==========================================

@dataclass
class TextChunk:
    """Represents a semantic chunk of text"""
    content: str
    heading: str
    heading_level: int
    module_id: str
    chapter_id: str
    chapter_title: str
    section_path: List[str]
    token_count: int
    metadata: Dict


# ==========================================
# Text Chunking Functions
# ==========================================

def count_tokens(text: str) -> int:
    """Count tokens in text using tiktoken"""
    return len(encoder.encode(text))


def parse_markdown_sections(content: str, filepath: Path) -> List[Dict]:
    """
    Parse markdown into sections based on H2/H3 headers

    Args:
        content: Markdown content
        filepath: Path to markdown file

    Returns:
        List of section dictionaries
    """
    sections = []
    current_h2 = None
    current_h3 = None
    current_content = []

    # Extract module and chapter IDs from filepath
    parts = filepath.relative_to(DOCS_DIR).parts
    module_id = parts[0] if len(parts) > 0 else "unknown"
    chapter_id = filepath.stem

    # Extract chapter title from first H1
    title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    chapter_title = title_match.group(1) if title_match else chapter_id

    lines = content.split("\n")

    for line in lines:
        # H2 heading
        h2_match = re.match(r'^##\s+(.+)$', line)
        if h2_match:
            # Save previous section if exists
            if current_content:
                sections.append({
                    "heading": current_h3 or current_h2 or "Introduction",
                    "heading_level": 3 if current_h3 else 2,
                    "content": "\n".join(current_content).strip(),
                    "section_path": [h for h in [current_h2, current_h3] if h],
                })

            current_h2 = h2_match.group(1)
            current_h3 = None
            current_content = []
            continue

        # H3 heading
        h3_match = re.match(r'^###\s+(.+)$', line)
        if h3_match:
            # Save previous subsection if exists
            if current_content and current_h2:
                sections.append({
                    "heading": current_h3 or current_h2,
                    "heading_level": 3 if current_h3 else 2,
                    "content": "\n".join(current_content).strip(),
                    "section_path": [h for h in [current_h2, current_h3] if h],
                })

            current_h3 = h3_match.group(1)
            current_content = []
            continue

        # Regular content line
        current_content.append(line)

    # Save final section
    if current_content:
        sections.append({
            "heading": current_h3 or current_h2 or "Introduction",
            "heading_level": 3 if current_h3 else 2,
            "content": "\n".join(current_content).strip(),
            "section_path": [h for h in [current_h2, current_h3] if h],
        })

    # Add metadata to all sections
    for section in sections:
        section["module_id"] = module_id
        section["chapter_id"] = chapter_id
        section["chapter_title"] = chapter_title

    return sections


def create_overlapping_chunks(sections: List[Dict]) -> List[TextChunk]:
    """
    Create overlapping chunks from sections

    Args:
        sections: List of parsed sections

    Returns:
        List of TextChunk objects
    """
    chunks = []

    for section in sections:
        content = section["content"]
        if not content or content.strip() == "":
            continue

        token_count = count_tokens(content)

        # If section fits in one chunk, use as-is
        if token_count <= MAX_CHUNK_SIZE:
            chunks.append(TextChunk(
                content=content,
                heading=section["heading"],
                heading_level=section["heading_level"],
                module_id=section["module_id"],
                chapter_id=section["chapter_id"],
                chapter_title=section["chapter_title"],
                section_path=section["section_path"],
                token_count=token_count,
                metadata={
                    "source": "textbook",
                    "type": "section",
                }
            ))
            continue

        # Split large sections into overlapping chunks
        sentences = re.split(r'(?<=[.!?])\s+', content)
        current_chunk = []
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = count_tokens(sentence)

            if current_tokens + sentence_tokens > MAX_CHUNK_SIZE and current_chunk:
                # Save current chunk
                chunk_content = " ".join(current_chunk)
                chunks.append(TextChunk(
                    content=chunk_content,
                    heading=section["heading"],
                    heading_level=section["heading_level"],
                    module_id=section["module_id"],
                    chapter_id=section["chapter_id"],
                    chapter_title=section["chapter_title"],
                    section_path=section["section_path"],
                    token_count=count_tokens(chunk_content),
                    metadata={
                        "source": "textbook",
                        "type": "chunk",
                        "overlap": True,
                    }
                ))

                # Keep overlap for context
                overlap_sentences = current_chunk[-3:] if len(current_chunk) >= 3 else current_chunk
                current_chunk = overlap_sentences + [sentence]
                current_tokens = count_tokens(" ".join(current_chunk))
            else:
                current_chunk.append(sentence)
                current_tokens += sentence_tokens

        # Save final chunk
        if current_chunk:
            chunk_content = " ".join(current_chunk)
            chunks.append(TextChunk(
                content=chunk_content,
                heading=section["heading"],
                heading_level=section["heading_level"],
                module_id=section["module_id"],
                chapter_id=section["chapter_id"],
                chapter_title=section["chapter_title"],
                section_path=section["section_path"],
                token_count=count_tokens(chunk_content),
                metadata={
                    "source": "textbook",
                    "type": "chunk",
                }
            ))

    return chunks


# ==========================================
# Embedding Generation
# ==========================================

async def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings using OpenAI API

    Args:
        texts: List of text strings

    Returns:
        List of embedding vectors
    """
    try:
        response = await openai_client.embeddings.create(
            model=EMBEDDING_MODEL,
            input=texts,
            dimensions=EMBEDDING_DIMENSIONS
        )

        embeddings = [item.embedding for item in response.data]
        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings

    except Exception as e:
        logger.error(f"Failed to generate embeddings: {str(e)}")
        raise


# ==========================================
# Qdrant Operations
# ==========================================

def initialize_collection():
    """Initialize or reset Qdrant collection"""
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_exists = any(c.name == COLLECTION_NAME for c in collections)

        if collection_exists:
            logger.info(f"Collection '{COLLECTION_NAME}' already exists")
            return

        # Create collection
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSIONS,
                distance=Distance.COSINE
            )
        )
        logger.info(f"✅ Created collection '{COLLECTION_NAME}'")

    except Exception as e:
        logger.error(f"Failed to initialize collection: {str(e)}")
        raise


def reset_collection():
    """Delete and recreate collection"""
    try:
        # Delete if exists
        collections = qdrant_client.get_collections().collections
        if any(c.name == COLLECTION_NAME for c in collections):
            qdrant_client.delete_collection(COLLECTION_NAME)
            logger.info(f"Deleted existing collection '{COLLECTION_NAME}'")

        # Recreate
        initialize_collection()
        logger.info("✅ Collection reset complete")

    except Exception as e:
        logger.error(f"Failed to reset collection: {str(e)}")
        raise


async def ingest_chunks(chunks: List[TextChunk], batch_size: int = 100):
    """
    Ingest chunks into Qdrant with embeddings

    Args:
        chunks: List of TextChunk objects
        batch_size: Number of chunks per batch
    """
    total_chunks = len(chunks)
    logger.info(f"Ingesting {total_chunks} chunks in batches of {batch_size}")

    for i in range(0, total_chunks, batch_size):
        batch = chunks[i:i + batch_size]
        batch_num = i // batch_size + 1
        total_batches = (total_chunks + batch_size - 1) // batch_size

        logger.info(f"Processing batch {batch_num}/{total_batches} ({len(batch)} chunks)")

        # Generate embeddings
        texts = [chunk.content for chunk in batch]
        embeddings = await generate_embeddings(texts)

        # Prepare points
        points = []
        for idx, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            point_id = i + idx
            payload = {
                "content": chunk.content,
                "heading": chunk.heading,
                "heading_level": chunk.heading_level,
                "module_id": chunk.module_id,
                "chapter_id": chunk.chapter_id,
                "chapter_title": chunk.chapter_title,
                "section_path": chunk.section_path,
                "token_count": chunk.token_count,
                **chunk.metadata,
                "ingested_at": datetime.utcnow().isoformat(),
            }

            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            ))

        # Upload to Qdrant
        try:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            logger.info(f"✅ Uploaded batch {batch_num}/{total_batches}")

            # Rate limiting
            await asyncio.sleep(1)

        except Exception as e:
            logger.error(f"Failed to upload batch {batch_num}: {str(e)}")
            raise

    logger.info(f"✅ Ingestion complete: {total_chunks} chunks uploaded")


# ==========================================
# Main Ingestion Pipeline
# ==========================================

async def process_chapter_file(filepath: Path) -> List[TextChunk]:
    """Process a single chapter file into chunks"""
    logger.info(f"Processing: {filepath.relative_to(DOCS_DIR)}")

    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    # Parse sections
    sections = parse_markdown_sections(content, filepath)
    logger.info(f"  Found {len(sections)} sections")

    # Create chunks
    chunks = create_overlapping_chunks(sections)
    logger.info(f"  Created {len(chunks)} chunks")

    return chunks


async def process_module(module_number: int):
    """Process all chapters in a module"""
    module_dir = DOCS_DIR / f"module-{module_number}"

    if not module_dir.exists():
        logger.error(f"Module directory not found: {module_dir}")
        return

    chapter_files = sorted(module_dir.glob("*.md"))
    if not chapter_files:
        logger.warning(f"No markdown files in {module_dir}")
        return

    logger.info(f"\n📚 Processing Module {module_number} ({len(chapter_files)} chapters)")

    all_chunks = []
    for chapter_file in chapter_files:
        chunks = await process_chapter_file(chapter_file)
        all_chunks.extend(chunks)

    logger.info(f"Module {module_number}: Total {len(all_chunks)} chunks")
    return all_chunks


async def process_all_modules():
    """Process all modules and ingest into Qdrant"""
    logger.info("\n🚀 Starting full textbook ingestion pipeline\n")

    initialize_collection()

    all_chunks = []
    for module_number in range(1, 5):  # Modules 1-4
        chunks = await process_module(module_number)
        if chunks:
            all_chunks.extend(chunks)

    if all_chunks:
        logger.info(f"\n📊 Total chunks across all modules: {len(all_chunks)}")
        await ingest_chunks(all_chunks)
    else:
        logger.warning("No chunks generated - check if markdown files exist")


# ==========================================
# CLI Interface
# ==========================================

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Ingest textbook content into Qdrant vector database"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Process all modules"
    )
    parser.add_argument(
        "--module",
        type=int,
        choices=[1, 2, 3, 4],
        help="Process specific module"
    )
    parser.add_argument(
        "--chapter-id",
        type=str,
        help="Process specific chapter (e.g., module-1/01-introduction)"
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset collection before ingestion"
    )

    args = parser.parse_args()

    # Reset collection if requested
    if args.reset:
        reset_collection()
        if not (args.all or args.module or args.chapter_id):
            return

    # Process based on arguments
    if args.all:
        asyncio.run(process_all_modules())
    elif args.module:
        chunks = asyncio.run(process_module(args.module))
        if chunks:
            initialize_collection()
            asyncio.run(ingest_chunks(chunks))
    elif args.chapter_id:
        # Parse chapter ID
        parts = args.chapter_id.split("/")
        if len(parts) != 2:
            logger.error("Invalid chapter ID format. Use: module-N/chapter-id")
            sys.exit(1)

        module_id, chapter_id = parts
        chapter_file = DOCS_DIR / module_id / f"{chapter_id}.md"

        if not chapter_file.exists():
            logger.error(f"Chapter file not found: {chapter_file}")
            sys.exit(1)

        chunks = asyncio.run(process_chapter_file(chapter_file))
        if chunks:
            initialize_collection()
            asyncio.run(ingest_chunks(chunks))
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
