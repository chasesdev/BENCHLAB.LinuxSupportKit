#!/usr/bin/env python3
"""
BenchLab SDK Example - RGB LED Control

This example demonstrates how to:
- Set RGB LED to different colors and modes
- Control brightness and animation speed
- Query current RGB settings
"""

import sys
import time
from pathlib import Path

# Add SDK to path
sys.path.insert(0, str(Path(__file__).parent.parent / "benchlab_sdk"))

from benchlab_client import BenchLabClient


def main():
    if len(sys.argv) < 2:
        print("Usage: rgb_control.py <device>")
        print("\nExample:")
        print("  rgb_control.py /dev/benchlab0")
        print("  rgb_control.py benchlab0")
        return 1

    device = sys.argv[1]

    # Create client
    client = BenchLabClient()

    print("BenchLab SDK - RGB LED Control Example")
    print("=" * 50)
    print(f"Device: {device}\n")

    # Example 1: Solid red
    print("1. Setting RGB to solid red...")
    try:
        result = client.set_rgb(
            device=device,
            mode="solid",
            red=255,
            green=0,
            blue=0,
            brightness=128
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        time.sleep(2)
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 2: Solid green
    print("\n2. Setting RGB to solid green...")
    try:
        result = client.set_rgb(
            device=device,
            mode="solid",
            red=0,
            green=255,
            blue=0,
            brightness=128
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        time.sleep(2)
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 3: Solid blue
    print("\n3. Setting RGB to solid blue...")
    try:
        result = client.set_rgb(
            device=device,
            mode="solid",
            red=0,
            green=0,
            blue=255,
            brightness=128
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        time.sleep(2)
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 4: Breathing mode (purple)
    print("\n4. Setting RGB to breathing purple...")
    try:
        result = client.set_rgb(
            device=device,
            mode="breathing",
            red=128,
            green=0,
            blue=255,
            brightness=192,
            speed=128  # Medium speed
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        print("   (Watch the LED breathe for 5 seconds)")
        time.sleep(5)
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 5: Color cycle mode
    print("\n5. Setting RGB to color cycle...")
    try:
        result = client.set_rgb(
            device=device,
            mode="cycle",
            brightness=255,
            speed=200  # Faster cycling
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        print("   (Watch the LED cycle colors for 5 seconds)")
        time.sleep(5)
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 6: Temperature mode
    print("\n6. Setting RGB to temperature mode...")
    try:
        result = client.set_rgb(
            device=device,
            mode="temperature",
            brightness=255
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        print("   (LED will change color based on chip temperature)")
        print("   Blue=cool, Green=warm, Yellow=hot, Red=critical")
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 7: Turn off
    print("\n7. Turning RGB LED off...")
    try:
        result = client.set_rgb(
            device=device,
            mode="off"
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
    except Exception as e:
        print(f"   ✗ Failed: {e}")

    print("\n✓ Example completed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
