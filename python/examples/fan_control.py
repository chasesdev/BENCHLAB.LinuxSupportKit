#!/usr/bin/env python3
"""
BenchLab SDK Example - Fan Control

This example demonstrates how to:
- Set fan to manual mode with specific PWM duty
- Set fan to automatic temperature-based control
- Query fan status
"""

import sys
from pathlib import Path

# Add SDK to path
sys.path.insert(0, str(Path(__file__).parent.parent / "benchlab_sdk"))

from benchlab_client import BenchLabClient


def main():
    if len(sys.argv) < 2:
        print("Usage: fan_control.py <device> [fan_index]")
        print("\nExamples:")
        print("  fan_control.py /dev/benchlab0           # Control fan 0")
        print("  fan_control.py benchlab0 1               # Control fan 1")
        return 1

    device = sys.argv[1]
    fan_index = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    # Create client
    client = BenchLabClient()

    print("BenchLab SDK - Fan Control Example")
    print("=" * 50)
    print(f"Device: {device}")
    print(f"Fan:    {fan_index}\n")

    # Example 1: Set fan to 50% manual speed
    print("1. Setting fan to manual mode (50% duty)...")
    try:
        # Manual mode with 50% PWM (128/255)
        result = client._request('PUT', f'/devices/{device.replace("/dev/", "")}/fans/{fan_index}',
                                 json={
                                     'mode': 'manual',
                                     'manualDuty': 128
                                 })
        response = result.json()
        print(f"   ✓ Status: {response.get('status', 'ok')}")
        print(f"     Mode:   {response.get('profile', {}).get('mode', 'unknown')}")
        print(f"     Duty:   {response.get('profile', {}).get('manualDuty', 0)}/255")

    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 2: Set fan to auto mode
    print("\n2. Setting fan to automatic mode...")
    try:
        result = client.set_fan_auto(
            device=device,
            fan_index=fan_index,
            temp_threshold=60.0,    # Start ramping at 60°C
            min_duty=64,            # Minimum 25% speed
            max_duty=255,           # Maximum 100% speed
            sensor_index=0          # Use sensor 0 (chip temp)
        )
        print(f"   ✓ Status: {result.get('status', 'ok')}")
        print(f"     Mode:       {result.get('profile', {}).get('mode', 'unknown')}")
        print(f"     Threshold:  {result.get('profile', {}).get('tempThreshold', 0)}°C")
        print(f"     Duty range: {result.get('profile', {}).get('minDuty', 0)}-{result.get('profile', {}).get('maxDuty', 0)}")

    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 3: Set fan to maximum speed
    print("\n3. Setting fan to maximum speed...")
    try:
        result = client._request('PUT', f'/devices/{device.replace("/dev/", "")}/fans/{fan_index}',
                                 json={
                                     'mode': 'manual',
                                     'manualDuty': 255
                                 })
        response = result.json()
        print(f"   ✓ Status: {response.get('status', 'ok')}")
        print(f"     Duty: 100% (255/255)")

    except Exception as e:
        print(f"   ✗ Failed: {e}")

    # Example 4: Turn fan off
    print("\n4. Turning fan off...")
    try:
        result = client._request('PUT', f'/devices/{device.replace("/dev/", "")}/fans/{fan_index}',
                                 json={
                                     'mode': 'manual',
                                     'manualDuty': 0
                                 })
        response = result.json()
        print(f"   ✓ Status: {response.get('status', 'ok')}")
        print(f"     Duty: 0% (off)")

    except Exception as e:
        print(f"   ✗ Failed: {e}")

    print("\n✓ Example completed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
