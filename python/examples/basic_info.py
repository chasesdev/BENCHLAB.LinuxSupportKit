#!/usr/bin/env python3
"""
Basic BenchLab SDK Example - Device Information

This example demonstrates how to:
- Connect to the BenchLab HTTP service
- List available devices
- Query device information
"""

import sys
from pathlib import Path

# Add SDK to path
sys.path.insert(0, str(Path(__file__).parent.parent / "benchlab_sdk"))

from benchlab_client import BenchLabClient


def main():
    # Create client (default: http://127.0.0.1:8080)
    client = BenchLabClient()

    print("BenchLab SDK - Device Information Example")
    print("=" * 50)

    # Check service health
    try:
        health = client.health_check()
        print(f"✓ Service health: {health.get('status', 'unknown')}")
    except Exception as e:
        print(f"✗ Service not available: {e}")
        print("\nMake sure benchlab service is running:")
        print("  sudo systemctl start benchlab-http")
        return 1

    # List all available devices
    print("\nDiscovering devices...")
    try:
        devices = client.list_devices()
        if not devices:
            print("No devices found")
            return 1

        print(f"Found {len(devices)} device(s):")
        for dev in devices:
            device_path = dev.get('device', 'unknown')
            is_valid = dev.get('isValid', False)
            status = "✓" if is_valid else "✗"
            print(f"  {status} {device_path}")

    except Exception as e:
        print(f"✗ Failed to list devices: {e}")
        return 1

    # Get detailed info for first device
    first_device = devices[0]['device']
    print(f"\nQuerying device info: {first_device}")
    try:
        info = client.get_device_info(first_device)

        print("\nDevice Details:")
        print(f"  Name:            {info.name}")
        print(f"  Device Path:     {info.device}")
        print(f"  Vendor ID:       0x{info.vendor_id:04x}")
        print(f"  Product ID:      0x{info.product_id:04x}")
        print(f"  Firmware:        v{info.firmware_version}")
        print(f"  Valid:           {info.is_valid}")
        print(f"  Timestamp:       {info.timestamp}")

    except Exception as e:
        print(f"✗ Failed to get device info: {e}")
        return 1

    # Get calibration status
    print(f"\nChecking calibration...")
    try:
        cal = client.get_calibration(first_device)
        print(f"  Voltage offsets: {cal.get('voltageOffsets', [])[:3]}... (13 total)")
        print(f"  Temp offset:     {cal.get('tempOffset', 0)}")
        print(f"  Flags:           {cal.get('flags', 0)}")
    except Exception as e:
        print(f"  Warning: {e}")

    print("\n✓ Example completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
