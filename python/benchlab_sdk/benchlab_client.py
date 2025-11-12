"""
BenchLab Python SDK - Official client library for BenchLab HTTP Service.

This library provides a high-level interface for interacting with BenchLab devices
through the HTTP service API.
"""

import time
from typing import Optional, Iterator, Dict, Any, List
from dataclasses import dataclass
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry


@dataclass
class DeviceInfo:
    """Information about a BenchLab device."""
    device: str
    name: str
    vendor_id: str
    product_id: str
    firmware_version: int
    is_valid: bool
    timestamp: str


@dataclass
class SensorReading:
    """Sensor telemetry data from a BenchLab device."""
    device: str
    timestamp: str
    voltages: List[Dict[str, Any]]
    temperatures: Dict[str, Any]
    humidity: float
    power: List[Dict[str, float]]
    fans: List[Dict[str, Any]]


class BenchLabError(Exception):
    """Base exception for BenchLab client errors."""
    pass


class DeviceNotFoundError(BenchLabError):
    """Raised when a device is not found."""
    pass


class DeviceBusyError(BenchLabError):
    """Raised when a device is already in use."""
    pass


class AuthenticationError(BenchLabError):
    """Raised when authentication fails."""
    pass


class BenchLabClient:
    """
    Client for interacting with BenchLab HTTP Service.

    Example:
        ```python
        # Create client with API key authentication
        client = BenchLabClient("http://localhost:8080", api_key="your-secret-key")

        # List available devices
        devices = client.list_devices()

        # Get device information
        info = client.get_device_info("/dev/benchlab0")
        print(f"Device: {info.name} (FW v{info.firmware_version})")

        # Read sensor data
        sensors = client.read_sensors("/dev/benchlab0")
        print(f"Chip temperature: {sensors.temperatures['chip']}°C")

        # Stream telemetry data
        for reading in client.stream_telemetry("/dev/benchlab0"):
            print(f"Power: {sum(p['w'] for p in reading.power):.2f}W")
        ```
    """

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8080",
        api_key: Optional[str] = None,
        timeout: float = 10.0,
        retry_count: int = 3,
        verify_ssl: bool = True
    ):
        """
        Initialize the BenchLab client.

        Args:
            base_url: Base URL of the BenchLab HTTP service
            api_key: Optional API key for authentication
            timeout: Request timeout in seconds
            retry_count: Number of retries for failed requests
            verify_ssl: Whether to verify SSL certificates
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.session = requests.Session()

        # Configure retries
        retry_strategy = Retry(
            total=retry_count,
            backoff_factor=0.5,
            status_forcelist=[429, 500, 502, 503, 504],
            allowed_methods=["GET", "POST"]
        )
        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

        # Set authentication header if API key provided
        if api_key:
            self.session.headers['Authorization'] = f'Bearer {api_key}'

        self.session.verify = verify_ssl

    def _request(self, method: str, endpoint: str, **kwargs) -> requests.Response:
        """Make an HTTP request with error handling."""
        url = f"{self.base_url}{endpoint}"

        try:
            response = self.session.request(
                method,
                url,
                timeout=kwargs.pop('timeout', self.timeout),
                **kwargs
            )

            # Handle HTTP errors
            if response.status_code == 401:
                raise AuthenticationError("Invalid or missing API key")
            elif response.status_code == 404:
                raise DeviceNotFoundError(f"Device not found: {response.json().get('error', 'Unknown')}")
            elif response.status_code == 409:
                raise DeviceBusyError("Device is already in use by another client")
            elif response.status_code >= 400:
                error_msg = response.json().get('error', response.text) if response.text else f"HTTP {response.status_code}"
                raise BenchLabError(f"Request failed: {error_msg}")

            return response

        except requests.exceptions.Timeout:
            raise BenchLabError(f"Request timeout after {self.timeout}s")
        except requests.exceptions.ConnectionError as e:
            raise BenchLabError(f"Connection failed: {e}")
        except requests.exceptions.RequestException as e:
            raise BenchLabError(f"Request error: {e}")

    def get_info(self) -> Dict[str, Any]:
        """
        Get service information.

        Returns:
            Dictionary containing service metadata
        """
        response = self._request('GET', '/')
        return response.json()

    def health_check(self) -> Dict[str, Any]:
        """
        Check service health.

        Returns:
            Dictionary with health status
        """
        response = self._request('GET', '/health')
        return response.json()

    def list_devices(self) -> List[Dict[str, Any]]:
        """
        List all available devices.

        Returns:
            List of device probe results
        """
        response = self._request('GET', '/devices')
        return response.json()

    def get_device_info(self, device: str, timeout: Optional[int] = None) -> DeviceInfo:
        """
        Get detailed information about a device.

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")
            timeout: Optional timeout in milliseconds

        Returns:
            DeviceInfo object with device details
        """
        device_id = device.replace('/dev/', '')
        params = {'timeout': timeout} if timeout else {}

        response = self._request('GET', f'/devices/{device_id}/info', params=params)
        data = response.json()

        return DeviceInfo(
            device=data['device'],
            name=data['name'],
            vendor_id=data['vendorId'],
            product_id=data['productId'],
            firmware_version=data['firmwareVersion'],
            is_valid=data['isValid'],
            timestamp=data['timestamp']
        )

    def read_sensors(self, device: str, timeout: Optional[int] = None) -> SensorReading:
        """
        Read sensor telemetry from a device (one-shot).

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")
            timeout: Optional timeout in milliseconds

        Returns:
            SensorReading object with telemetry data
        """
        device_id = device.replace('/dev/', '')
        params = {'timeout': timeout} if timeout else {}

        response = self._request('GET', f'/devices/{device_id}/sensors', params=params)
        data = response.json()

        return SensorReading(
            device=data['device'],
            timestamp=data['timestamp'],
            voltages=data['voltages'],
            temperatures=data['temperatures'],
            humidity=data['humidity'],
            power=data['power'],
            fans=data['fans']
        )

    def stream_telemetry(
        self,
        device: Optional[str] = None,
        timeout: Optional[int] = None,
        max_retries: int = 3
    ) -> Iterator[Dict[str, Any]]:
        """
        Stream telemetry data from a device (continuous).

        This is a generator that yields telemetry readings as they arrive.
        The connection will automatically reconnect on errors.

        Args:
            device: Optional device path (auto-discovers if not specified)
            timeout: Optional timeout in milliseconds
            max_retries: Maximum number of reconnection attempts

        Yields:
            Dictionary containing telemetry data

        Example:
            ```python
            for reading in client.stream_telemetry("/dev/benchlab0"):
                print(f"Temp: {reading['chipTemp']}°C")
            ```
        """
        import json

        params = {}
        if device:
            params['device'] = device
        if timeout:
            params['timeout'] = timeout

        retry_count = 0

        while retry_count <= max_retries:
            try:
                with self._request('GET', '/stream', params=params, stream=True, timeout=None) as response:
                    retry_count = 0  # Reset on successful connection

                    for line in response.iter_lines():
                        if line:
                            try:
                                # Parse each NDJSON line as separate JSON object
                                line_str = line.decode('utf-8') if isinstance(line, bytes) else line
                                data = json.loads(line_str)
                                yield data
                            except json.JSONDecodeError as e:
                                # Skip malformed JSON lines but log warning
                                continue
                            except Exception as e:
                                # Skip other parsing errors
                                continue

            except (BenchLabError, requests.exceptions.RequestException) as e:
                retry_count += 1
                if retry_count > max_retries:
                    raise BenchLabError(f"Stream failed after {max_retries} retries: {e}")

                # Exponential backoff
                wait_time = min(2 ** retry_count, 30)
                time.sleep(wait_time)

    def write_data(self, device: str, data: str) -> Dict[str, Any]:
        """
        Write data to a device.

        Args:
            device: Device path (e.g., "/dev/benchlab0")
            data: Data string to write

        Returns:
            Dictionary with write result
        """
        payload = {
            'device': device,
            'data': data
        }

        response = self._request('POST', '/write', json=payload)
        return response.json()

    def get_calibration(self, device: str) -> Dict[str, Any]:
        """
        Get calibration data from device RAM.

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")

        Returns:
            Dictionary with calibration data (voltageOffsets, voltageScales, etc.)
        """
        device_id = device.replace('/dev/', '')
        response = self._request('GET', f'/devices/{device_id}/calibration')
        return response.json()

    def set_rgb(
        self,
        device: str,
        mode: str,
        red: int = 0,
        green: int = 0,
        blue: int = 0,
        brightness: int = 128,
        speed: int = 128
    ) -> Dict[str, Any]:
        """
        Set RGB LED configuration.

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")
            mode: LED mode ("off", "solid", "breathing", "cycle", "temperature")
            red: Red value (0-255)
            green: Green value (0-255)
            blue: Blue value (0-255)
            brightness: Brightness (0-255, default 128)
            speed: Animation speed (0-255, default 128)

        Returns:
            Dictionary with status and RGB settings
        """
        device_id = device.replace('/dev/', '')
        payload = {
            'mode': mode,
            'red': red,
            'green': green,
            'blue': blue,
            'brightness': brightness,
            'speed': speed
        }

        response = self._request('PUT', f'/devices/{device_id}/rgb', json=payload)
        return response.json()

    def set_fan_auto(
        self,
        device: str,
        fan_index: int,
        temp_threshold: float,
        min_duty: int,
        max_duty: int,
        sensor_index: int = 0
    ) -> Dict[str, Any]:
        """
        Set fan to automatic temperature-based control.

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")
            fan_index: Fan index (0-8)
            temp_threshold: Temperature threshold in Celsius
            min_duty: Minimum PWM duty cycle (0-255)
            max_duty: Maximum PWM duty cycle (0-255)
            sensor_index: Temperature sensor to monitor (default 0)

        Returns:
            Dictionary with status and fan profile
        """
        device_id = device.replace('/dev/', '')
        payload = {
            'mode': 'auto',
            'tempThreshold': temp_threshold,
            'minDuty': min_duty,
            'maxDuty': max_duty,
            'sensorIndex': sensor_index
        }

        response = self._request('PUT', f'/devices/{device_id}/fans/{fan_index}', json=payload)
        return response.json()

    def get_sensors(self, device: str, timeout: Optional[int] = None) -> SensorReading:
        """
        Alias for read_sensors() for compatibility.

        Args:
            device: Device path (e.g., "/dev/benchlab0" or "benchlab0")
            timeout: Optional timeout in milliseconds

        Returns:
            SensorReading object with telemetry data
        """
        return self.read_sensors(device, timeout)

    def close(self):
        """Close the HTTP session."""
        self.session.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


# Convenience function
def connect(base_url: str = "http://127.0.0.1:8080", api_key: Optional[str] = None, **kwargs) -> BenchLabClient:
    """
    Create and return a BenchLab client instance.

    Args:
        base_url: Base URL of the BenchLab HTTP service
        api_key: Optional API key for authentication
        **kwargs: Additional arguments passed to BenchLabClient

    Returns:
        BenchLabClient instance
    """
    return BenchLabClient(base_url, api_key, **kwargs)
