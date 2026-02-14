"""
Test script to run the viewer with debug output
"""
import sys
import os

# Ensure we're using the venv Python
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version}")

# Add viewer to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("\n" + "="*60)
print("STARTING RPLIDAR VIEWER WITH DEBUG OUTPUT")
print("="*60 + "\n")

try:
    from viewer.app import main
    print("[TEST] Successfully imported viewer.app")
    print("[TEST] Calling main()...\n")
    main()
except Exception as e:
    print(f"\n[TEST] FATAL ERROR: {e}")
    import traceback
    traceback.print_exc()
    input("\nPress Enter to exit...")
