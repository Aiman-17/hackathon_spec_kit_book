"""
Ingestion API Router

Endpoints for ingesting textbook chapters into the vector database.
"""

import logging
from pathlib import Path
from typing import List, Dict, Any
from fastapi import APIRouter, HTTPException, BackgroundTasks, status
from pydantic import BaseModel

from src.services.document_ingestion import DocumentIngestionService
from src.services.embedding_pipeline import EmbeddingPipelineService
from src.services.qdrant_manager import qdrant_manager
from src.config import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/ingestion", tags=["Ingestion"])


# ==========================================
# Request/Response Models
# ==========================================

class IngestionRequest(BaseModel):
    """Request model for ingesting documents"""
    source_path: str = "frontend/docs"
    force_reindex: bool = False


class IngestionStatus(BaseModel):
    """Status of ingestion process"""
    status: str
    message: str
    chapters_processed: int = 0
    chunks_created: int = 0
    vectors_stored: int = 0
    errors: List[str] = []


class IngestionResult(BaseModel):
    """Result of ingestion process"""
    success: bool
    total_chapters: int
    total_chunks: int
    total_vectors: int
    failed_chapters: List[str] = []
    processing_time_seconds: float
    details: Dict[str, Any] = {}


# ==========================================
# Ingestion State (for background tasks)
# ==========================================

ingestion_state = {
    "status": "idle",  # idle, running, completed, failed
    "current_chapter": None,
    "chapters_processed": 0,
    "chunks_created": 0,
    "vectors_stored": 0,
    "errors": [],
    "start_time": None,
    "end_time": None,
}


# ==========================================
# Background Ingestion Task
# ==========================================

async def run_ingestion_task(source_path: str, force_reindex: bool):
    """
    Background task to ingest all chapters.

    Args:
        source_path: Path to the docs directory
        force_reindex: Whether to reindex existing documents
    """
    import time
    from datetime import datetime

    global ingestion_state

    ingestion_state["status"] = "running"
    ingestion_state["start_time"] = datetime.utcnow()
    ingestion_state["chapters_processed"] = 0
    ingestion_state["chunks_created"] = 0
    ingestion_state["vectors_stored"] = 0
    ingestion_state["errors"] = []

    try:
        # Initialize services
        doc_service = DocumentIngestionService()
        embedding_service = EmbeddingPipelineService()

        # Find all markdown files
        docs_path = Path(source_path)
        chapter_files = list(docs_path.glob("module-*/*.md"))

        logger.info(f"Found {len(chapter_files)} chapters to ingest")

        for chapter_file in chapter_files:
            try:
                ingestion_state["current_chapter"] = str(chapter_file)
                logger.info(f"Processing: {chapter_file}")

                # Read chapter content
                with open(chapter_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract metadata from path (module-X/YY-chapter-name.md)
                module_name = chapter_file.parent.name  # e.g., "module-1"
                chapter_filename = chapter_file.stem  # e.g., "01-introduction-to-physical-ai"

                # Parse frontmatter to get metadata
                import yaml
                import re

                # Extract YAML frontmatter
                frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
                frontmatter_data = {}
                if frontmatter_match:
                    try:
                        frontmatter_data = yaml.safe_load(frontmatter_match.group(1))
                    except Exception as e:
                        logger.warning(f"Failed to parse frontmatter for {chapter_file}: {e}")

                # Create ChapterMetadata
                from src.models.chapter import ChapterMetadata

                # Extract module number from module_name (e.g., "module-1" -> 1)
                module_num = int(module_name.split('-')[1]) if '-' in module_name else 1

                # Get chapter number from frontmatter or filename
                chapter_num = frontmatter_data.get('sidebar_position', 0)
                if chapter_num == 0:
                    # Try to extract from filename (e.g., "01-..." -> 1)
                    num_match = re.match(r'^(\d+)-', chapter_filename)
                    if num_match:
                        chapter_num = int(num_match.group(1))
                    else:
                        chapter_num = 1

                metadata = ChapterMetadata(
                    module_id=module_name,
                    module_title=f"Module {module_num}",
                    chapter_id=chapter_filename,
                    chapter_title=frontmatter_data.get('title', chapter_filename),
                    chapter_number=chapter_num,
                    tags=frontmatter_data.get('tags', []),
                    description=frontmatter_data.get('description'),
                    sidebar_position=frontmatter_data.get('sidebar_position')
                )

                # Process document into chunks
                chunks = doc_service.process_document(
                    content=content,
                    chapter_metadata=metadata
                )

                ingestion_state["chunks_created"] += len(chunks)

                # Convert Pydantic models to dictionaries for embedding service
                chunk_dicts = [chunk.model_dump() for chunk in chunks]

                # Generate embeddings and store in Qdrant
                vectors_count = await embedding_service.embed_and_store_chunks(chunk_dicts)
                ingestion_state["vectors_stored"] += vectors_count

                ingestion_state["chapters_processed"] += 1

                logger.info(f"✅ Processed {chapter_file.name}: {len(chunks)} chunks, {vectors_count} vectors")

            except Exception as e:
                error_msg = f"Failed to process {chapter_file}: {str(e)}"
                logger.error(error_msg)
                ingestion_state["errors"].append(error_msg)

        ingestion_state["status"] = "completed"
        ingestion_state["end_time"] = datetime.utcnow()

        logger.info(f"✅ Ingestion complete: {ingestion_state['chapters_processed']} chapters, "
                   f"{ingestion_state['chunks_created']} chunks, {ingestion_state['vectors_stored']} vectors")

    except Exception as e:
        ingestion_state["status"] = "failed"
        ingestion_state["end_time"] = datetime.utcnow()
        error_msg = f"Ingestion failed: {str(e)}"
        logger.error(error_msg)
        ingestion_state["errors"].append(error_msg)


# ==========================================
# API Endpoints
# ==========================================

@router.post("/start", response_model=Dict[str, Any], status_code=status.HTTP_202_ACCEPTED)
async def start_ingestion(
    request: IngestionRequest,
    background_tasks: BackgroundTasks
):
    """
    Start ingestion process in the background.

    This endpoint initiates the ingestion of all textbook chapters into the vector database.
    The process runs in the background and can be monitored via the /status endpoint.

    Args:
        request: Ingestion configuration
        background_tasks: FastAPI background tasks manager

    Returns:
        Confirmation message with instructions to check status
    """
    global ingestion_state

    if ingestion_state["status"] == "running":
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Ingestion is already running. Check /api/ingestion/status for progress."
        )

    # Reset state
    ingestion_state = {
        "status": "idle",
        "current_chapter": None,
        "chapters_processed": 0,
        "chunks_created": 0,
        "vectors_stored": 0,
        "errors": [],
        "start_time": None,
        "end_time": None,
    }

    # Add background task
    background_tasks.add_task(run_ingestion_task, request.source_path, request.force_reindex)

    return {
        "message": "Ingestion started successfully",
        "status": "running",
        "check_status_at": "/api/ingestion/status"
    }


@router.get("/status", response_model=IngestionStatus)
async def get_ingestion_status():
    """
    Get current status of ingestion process.

    Returns:
        Current ingestion status including progress and errors
    """
    global ingestion_state

    return IngestionStatus(
        status=ingestion_state["status"],
        message=f"Current chapter: {ingestion_state.get('current_chapter', 'N/A')}"
                if ingestion_state["status"] == "running"
                else f"Ingestion {ingestion_state['status']}",
        chapters_processed=ingestion_state["chapters_processed"],
        chunks_created=ingestion_state["chunks_created"],
        vectors_stored=ingestion_state["vectors_stored"],
        errors=ingestion_state["errors"]
    )


@router.post("/stop", response_model=Dict[str, Any])
async def stop_ingestion():
    """
    Stop the current ingestion process.

    Note: This is a soft stop - currently running chapter will complete.
    """
    global ingestion_state

    if ingestion_state["status"] != "running":
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="No ingestion is currently running"
        )

    # TODO: Implement graceful shutdown mechanism
    return {
        "message": "Ingestion stop requested (will complete current chapter)",
        "status": ingestion_state["status"]
    }
