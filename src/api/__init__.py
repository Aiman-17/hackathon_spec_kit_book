"""
API Package

FastAPI routes and middleware.
"""

from .middleware import (
    setup_middleware,
    configure_cors,
    RateLimitMiddleware,
    RequestLoggingMiddleware,
    ErrorHandlingMiddleware,
    SecurityHeadersMiddleware,
    RequestIDMiddleware,
    get_request_id,
    get_client_ip,
)

__all__ = [
    "setup_middleware",
    "configure_cors",
    "RateLimitMiddleware",
    "RequestLoggingMiddleware",
    "ErrorHandlingMiddleware",
    "SecurityHeadersMiddleware",
    "RequestIDMiddleware",
    "get_request_id",
    "get_client_ip",
]
