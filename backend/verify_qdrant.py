"""Verify Qdrant collection was created"""
from src.services.qdrant_manager import qdrant_manager

def main():
    qdrant_manager.initialize()

    try:
        # Check if collection exists
        exists = qdrant_manager.collection_exists()
        print(f"Collection exists: {exists}")

        if exists:
            # Get collection info
            info = qdrant_manager.get_collection_info()
            print(f"\nCollection: {info.get('name', 'N/A')}")
            print(f"Vector dimensions: {info.get('vector_size', 'N/A')}")
            print(f"Distance metric: {info.get('distance', 'N/A')}")
            print(f"Points count: {info.get('points_count', 0)}")
            print(f"Status: {info.get('status', 'N/A')}")
        else:
            print("ERROR: Collection does not exist!")

    finally:
        qdrant_manager.close()

if __name__ == "__main__":
    main()
