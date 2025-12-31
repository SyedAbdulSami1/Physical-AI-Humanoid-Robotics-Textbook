#!/usr/bin/env python3
"""
Debug script to check environment variable loading, especially GEMINI_API_KEY
"""
import os
from dotenv import load_dotenv
from pathlib import Path

# Load environment variables
dotenv_path = Path(__file__).resolve().parent / '.env'
print(f"Loading .env from: {dotenv_path}")
print(f"File exists: {dotenv_path.exists()}")

# Load the .env file, overriding existing environment variables
load_dotenv(dotenv_path=dotenv_path, override=True)

# Debug the GEMINI_API_KEY
key = os.getenv("GEMINI_API_KEY")
print(f"\nGEMINI_API_KEY loaded: {key is not None}")
if key:
    print(f"GEMINI_API_KEY repr: {repr(key)}")
    print(f"GEMINI_API_KEY length: {len(key)}")
    print(f"GEMINI_API_KEY first 10 chars: {key[:10]}")
    print(f"GEMINI_API_KEY last 5 chars: {key[-5:]}")
    print(f"GEMINI_API_KEY starts with 'AIzaSy': {key.startswith('AIzaSy')}")
    
    # Check for common issues
    print(f"Has leading/trailing whitespace: {key != key.strip()}")
    print(f"Has newline characters: {'\\n' in repr(key) or '\\r' in repr(key)}")
else:
    print("GEMINI_API_KEY is None - not found in environment")

# Debug other environment variables
print(f"\nQDRANT_URL: {os.getenv('QDRANT_URL') is not None}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY') is not None}")
print(f"NEON_DATABASE_URL: {os.getenv('NEON_DATABASE_URL') is not None}")
print(f"FRONTEND_URL: {os.getenv('FRONTEND_URL') is not None}")