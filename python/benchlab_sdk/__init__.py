"""
BenchLab Python SDK

Official Python client library for BenchLab device communication.
"""

from .benchlab_client import (
    BenchLabClient,
    BenchLabError,
    DeviceNotFoundError,
    DeviceBusyError,
    AuthenticationError,
    DeviceInfo,
    SensorReading,
    connect
)

__version__ = "1.0.0"
__all__ = [
    "BenchLabClient",
    "BenchLabError",
    "DeviceNotFoundError",
    "DeviceBusyError",
    "AuthenticationError",
    "DeviceInfo",
    "SensorReading",
    "connect"
]
