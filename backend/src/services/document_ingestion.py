"""
Document Ingestion Service

Handles semantic chunking of textbook chapters by H2/H3 headers
with 500-1000 token overlap for optimal RAG performance.
"""

import re
import logging
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import tiktoken

from src.models.chapter import (
    ChapterMetadata,
    ChunkMetadata,
    ChapterChunk,
    ChapterChunkCreate,
    DocumentIngestionResult
)

logger = logging.getLogger(__name__)


# ==========================================
# Document Ingestion Service
# ==========================================

class DocumentIngestionService:
    """
    Processes markdown documents into semantic chunks.

    Features:
    - H2/H3 header-based semantic chunking
    - 500-1000 token overlap between chunks
    - Metadata extraction (module, chapter, section)
    - Token counting using tiktoken
    """

    # Chunking configuration
    MIN_CHUNK_SIZE = 500  # tokens
    MAX_CHUNK_SIZE = 1000  # tokens
    OVERLAP_SIZE = 200  # tokens

    def __init__(self):
        # Initialize tiktoken encoder for GPT-3.5/4
        self.encoder = tiktoken.encoding_for_model("gpt-3.5-turbo")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken"""
        return len(self.encoder.encode(text))

    # ==========================================
    # Section Parsing
    # ==========================================

    def parse_sections(
        self,
        content: str,
        chapter_metadata: ChapterMetadata
    ) -> List[Dict]:
        """
        Parse markdown into sections based on H2/H3 headers.

        Args:
            content: Markdown content
            chapter_metadata: Chapter metadata

        Returns:
            List of section dictionaries
        """
        sections = []
        current_h2 = None
        current_h3 = None
        current_content = []

        lines = content.split("\n")

        for line in lines:
            # H2 heading
            h2_match = re.match(r'^##\s+(.+)$', line)
            if h2_match:
                # Save previous section
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
                # Save previous subsection
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
            section["module_id"] = chapter_metadata.module_id
            section["chapter_id"] = chapter_metadata.chapter_id
            section["chapter_title"] = chapter_metadata.chapter_title

        logger.debug(f"Parsed {len(sections)} sections from chapter")

        return sections

    # ==========================================
    # Chunking
    # ==========================================

    def create_overlapping_chunks(
        self,
        sections: List[Dict],
        chapter_metadata: ChapterMetadata
    ) -> List[ChapterChunkCreate]:
        """
        Create overlapping chunks from sections.

        Args:
            sections: List of parsed sections
            chapter_metadata: Chapter metadata

        Returns:
            List of ChapterChunkCreate objects
        """
        chunks = []
        chunk_index = 0

        for section in sections:
            content = section["content"]
            if not content or content.strip() == "":
                continue

            # Skip sections that are too short (< 100 characters)
            if len(content.strip()) < 100:
                logger.debug(f"Skipping short section: {section['heading']} ({len(content)} chars)")
                continue

            token_count = self.count_tokens(content)

            # Check for code blocks and diagrams
            has_code_blocks = bool(re.search(r'```\w+', content))
            has_diagrams = bool(re.search(r'```mermaid', content))

            # Extract keywords (simple approach - top words)
            keywords = self._extract_keywords(content)

            # Create chunk metadata
            chunk_metadata = ChunkMetadata(
                heading=section["heading"],
                heading_level=section["heading_level"],
                section_path=section["section_path"],
                token_count=token_count,
                chunk_index=chunk_index,
                has_code_blocks=has_code_blocks,
                has_diagrams=has_diagrams,
                keywords=keywords
            )

            # If section fits in one chunk, use as-is
            if token_count <= self.MAX_CHUNK_SIZE:
                chunks.append(ChapterChunkCreate(
                    content=content,
                    chunk_metadata=chunk_metadata,
                    chapter_metadata=chapter_metadata
                ))
                chunk_index += 1
                continue

            # Split large sections into overlapping chunks
            sentences = re.split(r'(?<=[.!?])\s+', content)
            current_chunk = []
            current_tokens = 0

            for sentence in sentences:
                sentence_tokens = self.count_tokens(sentence)

                if current_tokens + sentence_tokens > self.MAX_CHUNK_SIZE and current_chunk:
                    # Save current chunk
                    chunk_content = " ".join(current_chunk)
                    chunk_metadata.token_count = self.count_tokens(chunk_content)
                    chunk_metadata.chunk_index = chunk_index

                    chunks.append(ChapterChunkCreate(
                        content=chunk_content,
                        chunk_metadata=chunk_metadata.model_copy(),
                        chapter_metadata=chapter_metadata
                    ))
                    chunk_index += 1

                    # Keep overlap for context
                    overlap_sentences = current_chunk[-3:] if len(current_chunk) >= 3 else current_chunk
                    current_chunk = overlap_sentences + [sentence]
                    current_tokens = self.count_tokens(" ".join(current_chunk))
                else:
                    current_chunk.append(sentence)
                    current_tokens += sentence_tokens

            # Save final chunk
            if current_chunk:
                chunk_content = " ".join(current_chunk)
                chunk_metadata.token_count = self.count_tokens(chunk_content)
                chunk_metadata.chunk_index = chunk_index

                chunks.append(ChapterChunkCreate(
                    content=chunk_content,
                    chunk_metadata=chunk_metadata.model_copy(),
                    chapter_metadata=chapter_metadata
                ))
                chunk_index += 1

        logger.info(f"Created {len(chunks)} chunks from {len(sections)} sections")

        return chunks

    def _extract_keywords(self, text: str, max_keywords: int = 5) -> List[str]:
        """Extract simple keywords from text"""
        # Remove code blocks and markdown
        text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
        text = re.sub(r'[#*`_]', '', text)

        # Extract words (simple approach)
        words = re.findall(r'\b[a-zA-Z]{4,}\b', text.lower())

        # Common stop words to exclude
        stop_words = {'this', 'that', 'with', 'from', 'will', 'have', 'your', 'more', 'when', 'than', 'they', 'were', 'been', 'their', 'which', 'about'}

        # Count frequencies
        word_freq = {}
        for word in words:
            if word not in stop_words:
                word_freq[word] = word_freq.get(word, 0) + 1

        # Return top keywords
        sorted_words = sorted(word_freq.items(), key=lambda x: x[1], reverse=True)
        return [word for word, freq in sorted_words[:max_keywords]]

    # ==========================================
    # Document Processing
    # ==========================================

    def process_document(
        self,
        content: str,
        chapter_metadata: ChapterMetadata
    ) -> List[ChapterChunkCreate]:
        """
        Process a markdown document into chunks.

        Args:
            content: Markdown content
            chapter_metadata: Chapter metadata

        Returns:
            List of chunks ready for embedding
        """
        logger.info(f"Processing document: {chapter_metadata.chapter_id}")

        # Parse sections
        sections = self.parse_sections(content, chapter_metadata)

        # Create chunks
        chunks = self.create_overlapping_chunks(sections, chapter_metadata)

        # Log statistics
        total_tokens = sum(chunk.chunk_metadata.token_count for chunk in chunks)
        avg_tokens = total_tokens / len(chunks) if chunks else 0

        logger.info(f"Document processing complete:")
        logger.info(f"  Sections: {len(sections)}")
        logger.info(f"  Chunks: {len(chunks)}")
        logger.info(f"  Total tokens: {total_tokens}")
        logger.info(f"  Avg tokens/chunk: {avg_tokens:.0f}")

        return chunks

    def process_file(
        self,
        file_path: Path,
        module_id: str,
        chapter_id: str
    ) -> Tuple[List[ChapterChunkCreate], DocumentIngestionResult]:
        """
        Process a markdown file.

        Args:
            file_path: Path to markdown file
            module_id: Module identifier
            chapter_id: Chapter identifier

        Returns:
            Tuple of (chunks, ingestion_result)
        """
        import time
        start_time = time.time()

        # Read file
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Extract chapter title from first H1
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        chapter_title = title_match.group(1) if title_match else chapter_id

        # Create metadata
        chapter_metadata = ChapterMetadata(
            module_id=module_id,
            module_title=f"Module {module_id.split('-')[1]}",
            chapter_id=chapter_id,
            chapter_title=chapter_title,
            chapter_number=int(chapter_id.split('-')[0]) if '-' in chapter_id else 0
        )

        # Process document
        chunks = self.process_document(content, chapter_metadata)

        # Create result
        total_tokens = sum(chunk.chunk_metadata.token_count for chunk in chunks)
        processing_time = int((time.time() - start_time) * 1000)

        result = DocumentIngestionResult(
            module_id=module_id,
            chapter_id=chapter_id,
            chunks_created=len(chunks),
            total_tokens=total_tokens,
            status="completed",
            processing_time_ms=processing_time,
            errors=[]
        )

        return chunks, result


# ==========================================
# Global Instance
# ==========================================

document_ingestion_service = DocumentIngestionService()


# ==========================================
# CLI Interface
# ==========================================

async def main():
    """CLI for testing document ingestion"""
    import argparse

    parser = argparse.ArgumentParser(description="Document Ingestion Test")
    parser.add_argument(
        "--file",
        type=Path,
        required=True,
        help="Markdown file to process"
    )
    parser.add_argument(
        "--module-id",
        type=str,
        required=True,
        help="Module ID (e.g., module-1)"
    )
    parser.add_argument(
        "--chapter-id",
        type=str,
        required=True,
        help="Chapter ID (e.g., 01-introduction)"
    )

    args = parser.parse_args()

    if not args.file.exists():
        logger.error(f"File not found: {args.file}")
        return

    logger.info(f"Processing file: {args.file}")

    chunks, result = document_ingestion_service.process_file(
        args.file,
        args.module_id,
        args.chapter_id
    )

    logger.info(f"\nIngestion Result:")
    logger.info(f"  Status: {result.status}")
    logger.info(f"  Chunks: {result.chunks_created}")
    logger.info(f"  Tokens: {result.total_tokens}")
    logger.info(f"  Time: {result.processing_time_ms}ms")

    # Show first chunk
    if chunks:
        logger.info(f"\nFirst chunk:")
        logger.info(f"  Heading: {chunks[0].chunk_metadata.heading}")
        logger.info(f"  Tokens: {chunks[0].chunk_metadata.token_count}")
        logger.info(f"  Content preview: {chunks[0].content[:200]}...")


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
