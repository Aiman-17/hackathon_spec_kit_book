"""
API Middleware

Custom middleware for CORS, rate limiting, logging, and error handling.
"""

import time
import logging
from typing import Callable
from collections import defaultdict
from datetime import datetime, timedelta

from fastapi import Request, Response, HTTPException, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from src.config import settings

logger = logging.getLogger(__name__)


# ==========================================
# Rate Limiting Middleware
# ==========================================

class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Rate limiting middleware using sliding window algorithm.

    Limits requests per IP address per minute.
    """

    def __init__(self, app: ASGIApp, requests_per_minute: int = 100):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.request_counts = defaultdict(list)  # {ip: [timestamps]}

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process request with rate limiting"""
        # Skip rate limiting for health check endpoints
        if request.url.path in ["/health", "/", "/api/config"]:
            return await call_next(request)

        # Get client IP
        client_ip = request.client.host

        # Clean old timestamps
        current_time = datetime.now()
        one_minute_ago = current_time - timedelta(minutes=1)

        self.request_counts[client_ip] = [
            timestamp for timestamp in self.request_counts[client_ip]
            if timestamp > one_minute_ago
        ]

        # Check rate limit
        if len(self.request_counts[client_ip]) >= self.requests_per_minute:
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "error": True,
                    "message": "Rate limit exceeded. Please try again later.",
                    "retry_after": 60,
                    "limit": self.requests_per_minute,
                }
            )

        # Add current request timestamp
        self.request_counts[client_ip].append(current_time)

        # Process request
        response = await call_next(request)

        # Add rate limit headers
        response.headers["X-RateLimit-Limit"] = str(self.requests_per_minute)
        response.headers["X-RateLimit-Remaining"] = str(
            self.requests_per_minute - len(self.request_counts[client_ip])
        )
        response.headers["X-RateLimit-Reset"] = str(int((current_time + timedelta(minutes=1)).timestamp()))

        return response


# ==========================================
# Request Logging Middleware
# ==========================================

class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """
    Logs all API requests with timing information.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process and log request"""
        start_time = time.time()

        # Log request
        logger.info(f"→ {request.method} {request.url.path} from {request.client.host}")

        # Process request
        try:
            response = await call_next(request)

            # Calculate duration
            duration_ms = int((time.time() - start_time) * 1000)

            # Log response
            logger.info(
                f"← {request.method} {request.url.path} "
                f"[{response.status_code}] {duration_ms}ms"
            )

            # Add timing header
            response.headers["X-Process-Time"] = f"{duration_ms}ms"

            return response

        except Exception as e:
            duration_ms = int((time.time() - start_time) * 1000)
            logger.error(
                f"✗ {request.method} {request.url.path} "
                f"[ERROR] {duration_ms}ms: {str(e)}"
            )
            raise


# ==========================================
# Error Handling Middleware
# ==========================================

class ErrorHandlingMiddleware(BaseHTTPMiddleware):
    """
    Catches and formats all unhandled exceptions.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process request with error handling"""
        try:
            return await call_next(request)

        except HTTPException:
            # Let FastAPI handle HTTP exceptions
            raise

        except ValueError as e:
            # Validation errors
            logger.error(f"Validation error: {str(e)}")
            return JSONResponse(
                status_code=status.HTTP_400_BAD_REQUEST,
                content={
                    "error": True,
                    "message": "Invalid request data",
                    "details": str(e),
                    "timestamp": datetime.utcnow().isoformat(),
                }
            )

        except Exception as e:
            # Unexpected errors
            logger.error(f"Unhandled exception: {str(e)}", exc_info=True)

            # Hide internal errors in production
            error_message = str(e) if settings.debug else "Internal server error"

            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content={
                    "error": True,
                    "message": error_message,
                    "timestamp": datetime.utcnow().isoformat(),
                }
            )


# ==========================================
# Security Headers Middleware
# ==========================================

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Adds security headers to all responses.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Add security headers"""
        response = await call_next(request)

        # Security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"

        # Content Security Policy (adjust as needed)
        if settings.environment == "production":
            response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"

        return response


# ==========================================
# Request ID Middleware
# ==========================================

class RequestIDMiddleware(BaseHTTPMiddleware):
    """
    Adds unique request ID to each request for tracing.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Add request ID"""
        import uuid

        request_id = str(uuid.uuid4())

        # Add to request state
        request.state.request_id = request_id

        # Process request
        response = await call_next(request)

        # Add to response headers
        response.headers["X-Request-ID"] = request_id

        return response


# ==========================================
# CORS Middleware Configuration
# ==========================================

def configure_cors(app):
    """
    Configure CORS middleware for the application.

    This is called in main.py during app initialization.
    """
    from fastapi.middleware.cors import CORSMiddleware

    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["X-Total-Count", "X-Page-Count", "X-Request-ID", "X-Process-Time"],
        max_age=3600,  # Cache preflight requests for 1 hour
    )

    logger.info(f"CORS configured with origins: {settings.cors_origins}")


# ==========================================
# Middleware Setup Function
# ==========================================

def setup_middleware(app):
    """
    Setup all custom middleware for the application.

    Call this function in main.py after app creation.
    """
    # Add middleware in reverse order (last added = first executed)

    # Security headers (last = first)
    app.add_middleware(SecurityHeadersMiddleware)

    # Request ID for tracing
    app.add_middleware(RequestIDMiddleware)

    # Error handling
    app.add_middleware(ErrorHandlingMiddleware)

    # Request logging
    if settings.debug or settings.environment != "production":
        app.add_middleware(RequestLoggingMiddleware)

    # Rate limiting
    app.add_middleware(
        RateLimitMiddleware,
        requests_per_minute=settings.rate_limit_per_minute
    )

    # CORS (first = last)
    configure_cors(app)

    logger.info("All middleware configured successfully")


# ==========================================
# Dependency Injection Helpers
# ==========================================

async def get_request_id(request: Request) -> str:
    """Get request ID from request state (for dependency injection)"""
    return getattr(request.state, "request_id", "unknown")


async def get_client_ip(request: Request) -> str:
    """Get client IP address (for dependency injection)"""
    return request.client.host
