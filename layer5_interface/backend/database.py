from sqlalchemy import inspect, text
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from models import Base
import os

# Database URL - using SQLite for simplicity
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite+aiosqlite:///./bin_collection.db")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=True,  # Set to False in production
    future=True
)

# Create async session factory
async_session = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)


def _migrate_legacy_schema(sync_conn):
    """Apply lightweight schema fixes for older local SQLite databases."""

    inspector = inspect(sync_conn)
    if 'bins' not in inspector.get_table_names():
        return

    columns = {column['name'] for column in inspector.get_columns('bins')}
    if 'pickup_marker_id' not in columns:
        sync_conn.execute(
            text(
                'ALTER TABLE bins '
                'ADD COLUMN pickup_marker_id INTEGER NOT NULL DEFAULT 0'
            )
        )
    if 'path_to_home_id' not in columns:
        sync_conn.execute(
            text(
                'ALTER TABLE bins '
                "ADD COLUMN path_to_home_id TEXT NOT NULL DEFAULT ''"
            )
        )


async def init_db():
    """Initialize database tables"""
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
        await conn.run_sync(_migrate_legacy_schema)


async def get_session() -> AsyncSession:
    """Dependency for getting database session"""
    async with async_session() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()
