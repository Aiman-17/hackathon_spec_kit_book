"""Verify database tables were created"""
import asyncio
from src.services.postgres_manager import postgres_manager

async def main():
    await postgres_manager.initialize()

    async with postgres_manager.acquire() as conn:
        tables = await conn.fetch(
            "SELECT tablename FROM pg_tables WHERE schemaname='public' ORDER BY tablename"
        )

        print("Database tables created:")
        for table in tables:
            print(f"  - {table['tablename']}")

        print(f"\nTotal tables: {len(tables)}")

    await postgres_manager.close()

if __name__ == "__main__":
    asyncio.run(main())
