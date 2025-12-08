"""
Configuration management for AI-Native Textbook Backend
Loads environment variables using Pydantic BaseSettings
"""

from typing import List
from pydantic_settings import BaseSettings
from pydantic import Field, PostgresDsn, HttpUrl


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.

    All settings can be configured via .env file or environment variables.
    See .env.example for required values.
    """

    # ==========================================
    # Database Configuration
    # ==========================================
    neon_db_url: PostgresDsn = Field(
        ...,
        description="Neon Postgres connection string",
        validation_alias="NEON_DB_URL"
    )

    # ==========================================
    # Vector Database Configuration
    # ==========================================
    qdrant_url: HttpUrl = Field(
        ...,
        description="Qdrant Cloud URL",
        validation_alias="QDRANT_URL"
    )
    qdrant_api_key: str = Field(
        ...,
        min_length=1,
        description="Qdrant API key",
        validation_alias="QDRANT_API_KEY"
    )

    # ==========================================
    # AI/ML API Keys
    # ==========================================
    openai_api_key: str = Field(
        ...,
        min_length=20,
        description="OpenAI API key for embeddings and RAG",
        validation_alias="OPENAI_API_KEY"
    )

    # ==========================================
    # Authentication
    # ==========================================
    better_auth_secret: str = Field(
        ...,
        min_length=32,
        description="Better-Auth secret key (min 32 characters)",
        validation_alias="BETTER_AUTH_SECRET"
    )
    jwt_secret_key: str = Field(
        ...,
        min_length=16,
        description="JWT secret key",
        validation_alias="JWT_SECRET_KEY"
    )
    jwt_algorithm: str = Field(
        default="HS256",
        description="JWT signing algorithm",
        validation_alias="JWT_ALGORITHM"
    )
    jwt_access_token_expire_minutes: int = Field(
        default=30,
        ge=1,
        le=1440,
        description="JWT access token expiration in minutes",
        validation_alias="JWT_ACCESS_TOKEN_EXPIRE_MINUTES"
    )

    # ==========================================
    # Caching
    # ==========================================
    redis_url: str = Field(
        default="redis://localhost:6379",
        description="Redis connection URL for caching",
        validation_alias="REDIS_URL"
    )
    cache_ttl: int = Field(
        default=3600,
        ge=60,
        le=86400,
        description="Cache TTL in seconds (default: 1 hour)",
        validation_alias="CACHE_TTL"
    )

    # ==========================================
    # Application Settings
    # ==========================================
    environment: str = Field(
        default="development",
        pattern="^(development|staging|production)$",
        description="Application environment",
        validation_alias="ENVIRONMENT"
    )
    api_host: str = Field(
        default="0.0.0.0",
        description="API server host",
        validation_alias="API_HOST"
    )
    api_port: int = Field(
        default=8000,
        ge=1,
        le=65535,
        description="API server port",
        validation_alias="API_PORT"
    )
    api_reload: bool = Field(
        default=True,
        description="Enable auto-reload in development",
        validation_alias="API_RELOAD"
    )

    # ==========================================
    # CORS Configuration
    # ==========================================
    cors_origins: List[str] = Field(
        default=[
            "http://localhost:3000",
            "https://aiman-17.github.io",
            "https://huggingface.co"
        ],
        description="Allowed CORS origins (comma-separated in .env)",
        validation_alias="CORS_ORIGINS"
    )

    # ==========================================
    # Rate Limiting
    # ==========================================
    rate_limit_per_minute: int = Field(
        default=100,
        ge=1,
        le=10000,
        description="Rate limit requests per minute",
        validation_alias="RATE_LIMIT_PER_MINUTE"
    )

    # ==========================================
    # Feature Flags
    # ==========================================
    enable_personalization: bool = Field(
        default=True,
        description="Enable personalization features",
        validation_alias="ENABLE_PERSONALIZATION"
    )
    enable_translation: bool = Field(
        default=True,
        description="Enable Urdu translation",
        validation_alias="ENABLE_TRANSLATION"
    )
    enable_agent_skills: bool = Field(
        default=True,
        description="Enable agent skill system (BONUS)",
        validation_alias="ENABLE_AGENT_SKILLS"
    )

    # ==========================================
    # Logging
    # ==========================================
    log_level: str = Field(
        default="INFO",
        pattern="^(DEBUG|INFO|WARNING|ERROR|CRITICAL)$",
        description="Logging level",
        validation_alias="LOG_LEVEL"
    )
    log_format: str = Field(
        default="json",
        pattern="^(json|text)$",
        description="Log output format",
        validation_alias="LOG_FORMAT"
    )

    # ==========================================
    # Development Settings
    # ==========================================
    debug: bool = Field(
        default=False,
        description="Enable debug mode",
        validation_alias="DEBUG"
    )
    auto_reload: bool = Field(
        default=True,
        description="Enable auto-reload for development",
        validation_alias="AUTO_RELOAD"
    )

    class Config:
        """Pydantic configuration"""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"  # Ignore extra fields in .env

        # Custom parsing for comma-separated lists
        @classmethod
        def parse_env_var(cls, field_name: str, raw_val: str):
            if field_name == "cors_origins":
                return [origin.strip() for origin in raw_val.split(",")]
            return raw_val


# Global settings instance
settings = Settings()


# Utility functions
def get_settings() -> Settings:
    """
    Get the global settings instance.

    This function can be used as a FastAPI dependency for injecting
    settings into route handlers.

    Returns:
        Settings: Application settings instance
    """
    return settings


def is_production() -> bool:
    """Check if running in production environment"""
    return settings.environment == "production"


def is_development() -> bool:
    """Check if running in development environment"""
    return settings.environment == "development"


def is_staging() -> bool:
    """Check if running in staging environment"""
    return settings.environment == "staging"
