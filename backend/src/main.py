"""
FastAPI Application Entry Point
AI-Native Textbook & RAG System Backend
"""

import logging
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Dict, Any

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException

from src.config import settings, get_settings
from src.api import setup_middleware
from src.services import postgres_manager, qdrant_manager


# ==========================================
# Logging Configuration
# ==========================================
logging.basicConfig(
    level=getattr(logging, settings.log_level),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    if settings.log_format == "text"
    else '{"time":"%(asctime)s","name":"%(name)s","level":"%(levelname)s","message":"%(message)s"}'
)
logger = logging.getLogger(__name__)


# ==========================================
# Application Lifespan Events
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager for startup and shutdown events.

    Handles:
    - Database connection initialization
    - Vector database connection
    - Redis cache connection
    - Resource cleanup on shutdown
    """
    logger.info("🚀 Starting AI-Native Textbook Backend...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Debug Mode: {settings.debug}")

    # Initialize Postgres connection pool
    try:
        await postgres_manager.initialize()
        logger.info("✅ Postgres connection pool initialized")
    except Exception as e:
        logger.error(f"Failed to initialize Postgres: {str(e)}")
        # Continue without blocking startup

    # Initialize Qdrant client
    try:
        qdrant_manager.initialize()
        logger.info("✅ Qdrant client initialized")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {str(e)}")
        # Continue without blocking startup

    # TODO: Initialize Redis client (Phase 5)

    logger.info("✅ Application startup complete")

    yield  # Application runs here

    # Shutdown
    logger.info("🛑 Shutting down AI-Native Textbook Backend...")

    # Close Postgres connection pool
    await postgres_manager.close()

    # Close Qdrant client
    qdrant_manager.close()

    # TODO: Close Redis connection (Phase 5)

    logger.info("✅ Application shutdown complete")


# ==========================================
# FastAPI Application Instance
# ==========================================
app = FastAPI(
    title="AI-Native Textbook & RAG System",
    description="Backend API for Physical AI & Humanoid Robotics Textbook with RAG-powered chatbot, personalization, and intelligent agent features",
    version="1.0.0",
    docs_url="/api/docs" if settings.debug else None,
    redoc_url="/api/redoc" if settings.debug else None,
    openapi_url="/api/openapi.json" if settings.debug else None,
    lifespan=lifespan,
)


# ==========================================
# Middleware Configuration
# ==========================================

# Setup all middleware (CORS, rate limiting, logging, security headers, etc.)
setup_middleware(app)

# Trusted Host Middleware - Additional security for production
if settings.environment == "production":
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["*.github.io", "localhost", "127.0.0.1"]
    )


# ==========================================
# Exception Handlers
# ==========================================

@app.exception_handler(StarletteHTTPException)
async def http_exception_handler(request: Request, exc: StarletteHTTPException):
    """Handle HTTP exceptions with consistent JSON response"""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": True,
            "message": exc.detail,
            "status_code": exc.status_code,
            "timestamp": datetime.utcnow().isoformat(),
        }
    )


@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Handle request validation errors"""
    return JSONResponse(
        status_code=422,
        content={
            "error": True,
            "message": "Validation error",
            "details": exc.errors(),
            "status_code": 422,
            "timestamp": datetime.utcnow().isoformat(),
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle unexpected errors"""
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": True,
            "message": "Internal server error" if settings.environment == "production" else str(exc),
            "status_code": 500,
            "timestamp": datetime.utcnow().isoformat(),
        }
    )


# ==========================================
# Health Check & System Endpoints
# ==========================================

@app.get("/", tags=["System"])
async def root() -> Dict[str, Any]:
    """
    Root endpoint - API information
    """
    return {
        "service": "AI-Native Textbook & RAG System",
        "version": "1.0.0",
        "status": "operational",
        "environment": settings.environment,
        "docs": "/api/docs" if settings.debug else "disabled",
        "timestamp": datetime.utcnow().isoformat(),
    }


@app.get("/health", tags=["System"])
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint for monitoring and load balancers

    Returns:
        Dict with health status and system information
    """
    return {
        "status": "healthy",
        "service": "ai-textbook-backend",
        "version": "1.0.0",
        "environment": settings.environment,
        "timestamp": datetime.utcnow().isoformat(),
        "features": {
            "personalization": settings.enable_personalization,
            "translation": settings.enable_translation,
            "agent_skills": settings.enable_agent_skills,
        }
    }


@app.get("/api/config", tags=["System"])
async def get_config() -> Dict[str, Any]:
    """
    Get public configuration (non-sensitive settings only)

    Returns:
        Dict with public configuration values
    """
    return {
        "environment": settings.environment,
        "features": {
            "personalization": settings.enable_personalization,
            "translation": settings.enable_translation,
            "agent_skills": settings.enable_agent_skills,
        },
        "rate_limit": settings.rate_limit_per_minute,
        "cache_ttl": settings.cache_ttl,
    }


# ==========================================
# API Routes
# ==========================================

# Import routers
from src.api.routers import ingestion, chat, personalization

# Include routers
app.include_router(ingestion.router)
app.include_router(chat.router)
app.include_router(personalization.router)  # Phase 4: Personalization & Translation

# TODO: Phase 5 - Agent intelligence endpoints (/api/agent)


# ==========================================
# Application Entry Point
# ==========================================

if __name__ == "__main__":
    import uvicorn

    logger.info(f"Starting Uvicorn server on {settings.api_host}:{settings.api_port}")

    uvicorn.run(
        "src.main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=settings.api_reload and settings.environment == "development",
        log_level=settings.log_level.lower(),
        access_log=settings.debug,
    )
