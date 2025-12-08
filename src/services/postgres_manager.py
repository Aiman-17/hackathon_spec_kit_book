"""
PostgreSQL Database Manager

Manages Neon Postgres connections, schema migrations, and database operations.
Uses asyncpg for asynchronous database operations.
"""

import asyncpg
import logging
from pathlib import Path
from typing import Optional
from contextlib import asynccontextmanager

from src.config import settings

logger = logging.getLogger(__name__)


# ==========================================
# PostgreSQL Connection Pool
# ==========================================

class PostgresManager:
    """
    Manages PostgreSQL connection pool and database operations.

    Uses asyncpg for high-performance async PostgreSQL access.
    """

    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
        self.schema_file = Path(__file__).parent.parent / "models" / "schema.sql"

    async def initialize(self):
        """
        Initialize the connection pool.

        Creates a connection pool with optimal settings for production.
        """
        if self.pool:
            logger.warning("Connection pool already initialized")
            return

        try:
            logger.info("Initializing PostgreSQL connection pool...")

            # Parse Neon DB URL
            db_url = str(settings.neon_db_url)

            # Create connection pool
            self.pool = await asyncpg.create_pool(
                dsn=db_url,
                min_size=2,
                max_size=10,
                max_queries=50000,
                max_inactive_connection_lifetime=300,
                timeout=30,
                command_timeout=60,
            )

            logger.info("✅ PostgreSQL connection pool initialized")

            # Verify connection
            async with self.pool.acquire() as conn:
                version = await conn.fetchval("SELECT version()")
                logger.info(f"Connected to: {version}")

        except Exception as e:
            logger.error(f"Failed to initialize PostgreSQL pool: {str(e)}")
            raise

    async def close(self):
        """Close the connection pool"""
        if self.pool:
            await self.pool.close()
            self.pool = None
            logger.info("PostgreSQL connection pool closed")

    @asynccontextmanager
    async def acquire(self):
        """
        Context manager to acquire a connection from the pool.

        Usage:
            async with postgres_manager.acquire() as conn:
                result = await conn.fetch("SELECT * FROM users")
        """
        if not self.pool:
            raise RuntimeError("Connection pool not initialized. Call initialize() first.")

        async with self.pool.acquire() as connection:
            yield connection

    # ==========================================
    # Schema Migration
    # ==========================================

    async def create_tables(self):
        """
        Create database tables from schema.sql.

        This is idempotent - safe to run multiple times.
        """
        if not self.schema_file.exists():
            raise FileNotFoundError(f"Schema file not found: {self.schema_file}")

        logger.info("Creating database tables...")

        # Read schema SQL
        with open(self.schema_file, "r", encoding="utf-8") as f:
            schema_sql = f.read()

        try:
            async with self.acquire() as conn:
                # Execute schema SQL
                await conn.execute(schema_sql)

            logger.info("✅ Database tables created successfully")

        except Exception as e:
            logger.error(f"Failed to create tables: {str(e)}")
            raise

    async def drop_tables(self):
        """
        Drop all tables (DANGEROUS - use only in development).

        This will delete all data!
        """
        if settings.environment == "production":
            raise RuntimeError("Cannot drop tables in production!")

        logger.warning("Dropping all tables...")

        drop_sql = """
        DROP TABLE IF EXISTS chat_history CASCADE;
        DROP TABLE IF EXISTS user_profiles CASCADE;
        DROP TABLE IF EXISTS users CASCADE;
        """

        try:
            async with self.acquire() as conn:
                await conn.execute(drop_sql)

            logger.info("✅ All tables dropped")

        except Exception as e:
            logger.error(f"Failed to drop tables: {str(e)}")
            raise

    async def migrate(self, reset: bool = False):
        """
        Run database migration.

        Args:
            reset: If True, drop and recreate all tables (DANGER!)
        """
        if reset:
            if settings.environment == "production":
                raise RuntimeError("Cannot reset database in production!")
            await self.drop_tables()

        await self.create_tables()

    # ==========================================
    # Health Check
    # ==========================================

    async def health_check(self) -> dict:
        """
        Check database health and return status.

        Returns:
            dict with health status, version, and connection pool stats
        """
        if not self.pool:
            return {
                "status": "unhealthy",
                "message": "Connection pool not initialized"
            }

        try:
            async with self.acquire() as conn:
                # Test query
                version = await conn.fetchval("SELECT version()")
                tables_count = await conn.fetchval("""
                    SELECT COUNT(*)
                    FROM information_schema.tables
                    WHERE table_schema = 'public'
                """)

            return {
                "status": "healthy",
                "version": version.split(",")[0],  # PostgreSQL version
                "tables_count": tables_count,
                "pool_size": self.pool.get_size(),
                "pool_free_size": self.pool.get_idle_size(),
            }

        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }

    # ==========================================
    # Query Helpers
    # ==========================================

    async def execute(self, query: str, *args):
        """Execute a query without returning results"""
        async with self.acquire() as conn:
            return await conn.execute(query, *args)

    async def fetch(self, query: str, *args):
        """Fetch multiple rows"""
        async with self.acquire() as conn:
            return await conn.fetch(query, *args)

    async def fetchrow(self, query: str, *args):
        """Fetch a single row"""
        async with self.acquire() as conn:
            return await conn.fetchrow(query, *args)

    async def fetchval(self, query: str, *args):
        """Fetch a single value"""
        async with self.acquire() as conn:
            return await conn.fetchval(query, *args)


# ==========================================
# Global Instance
# ==========================================

postgres_manager = PostgresManager()


# ==========================================
# CLI Interface (for migrations)
# ==========================================

async def main():
    """CLI for database management"""
    import argparse

    parser = argparse.ArgumentParser(description="PostgreSQL Database Manager")
    parser.add_argument(
        "--migrate",
        action="store_true",
        help="Run database migration"
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset database (drop and recreate tables)"
    )
    parser.add_argument(
        "--health",
        action="store_true",
        help="Check database health"
    )

    args = parser.parse_args()

    # Initialize connection pool
    await postgres_manager.initialize()

    try:
        if args.migrate:
            await postgres_manager.migrate(reset=args.reset)
            logger.info("Migration complete")

        elif args.health:
            health = await postgres_manager.health_check()
            logger.info(f"Health check: {health}")

        else:
            parser.print_help()

    finally:
        await postgres_manager.close()


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
