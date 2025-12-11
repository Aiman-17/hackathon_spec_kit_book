"""
Hugging Face Space Entry Point
Physical AI & Humanoid Robotics Textbook - RAG Backend API
"""

import os
import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

# Import the FastAPI app
from src.main import app

# Required for Hugging Face Spaces
if __name__ == "__main__":
    import uvicorn

    # Get port from environment (HF Spaces uses PORT env var)
    port = int(os.getenv("PORT", 7860))

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )
