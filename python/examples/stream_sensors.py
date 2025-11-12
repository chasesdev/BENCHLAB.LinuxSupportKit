#!/usr/bin/env python3
"""
BenchLab SDK Example - Sensor Streaming

This example demonstrates how to:
- Stream real-time telemetry data
- Parse sensor readings
- Display power consumption and temperatures
"""

import sys
import time
from pathlib import Path

# Add SDK to path
sys.path.insert(0, str(Path(__file__).parent.parent / "benchlab_sdk"))

from benchlab_client import BenchLabClient


def main():
    # Parse command line
    device = sys.argv[1] if len(sys.argv) > 1 else None

    # Create client
    client = BenchLabClient()

    print("BenchLab SDK - Sensor Streaming Example")
    print("=" * 50)

    if device:
        print(f"Streaming from: {device}")
    else:
        print("Streaming from: auto-discovered device")

    print("Press Ctrl+C to stop\n")

    try:
        # Stream telemetry data
        reading_count = 0

        for reading in client.stream_telemetry(device):
            reading_count += 1

            # Parse telemetry data
            chip_temp = reading.get('chipTemp', 0)
            power_rails = reading.get('power', [])
            fans = reading.get('fans', [])

            # Calculate total power
            total_power = sum(p.get('w', 0) for p in power_rails)

            # Display summary (every 10th reading)
            if reading_count % 10 == 0:
                print(f"[{reading_count:4d}] "
                      f"Chip: {chip_temp:5.1f}°C  "
                      f"Power: {total_power:6.2f}W  "
                      f"Fans: ", end="")

                # Show fan speeds
                for fan in fans[:3]:  # First 3 fans
                    if fan.get('enabled'):
                        rpm = fan.get('rpm', 0)
                        print(f"{rpm:4d}RPM ", end="")
                    else:
                        print("OFF  ", end="")

                print()  # Newline

            # Optional: Add delay to reduce output rate
            # time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n✓ Streaming stopped by user")
        return 0
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
