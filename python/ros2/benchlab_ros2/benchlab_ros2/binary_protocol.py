"""Python implementation of BenchLab binary UART protocol.

Maps to C# BinaryProtocol.cs with all 15 commands (0x00-0x0E).
Uses pyserial for communication with STM32 VCP devices.
"""

import struct
import serial
import time
from typing import Optional, Tuple
from dataclasses import dataclass


# Protocol Constants
VENDOR_ID = 0xEE
PRODUCT_ID = 0x10

SENSOR_VIN_COUNT = 13
TEMPERATURE_SENSOR_COUNT = 4
SENSOR_POWER_COUNT = 11
FAN_COUNT = 9

# UART Commands (matches C# ProtocolConstants)
CMD_WELCOME = 0x00
CMD_READ_SENSORS = 0x01
CMD_ACTION = 0x02
CMD_READ_NAME = 0x03
CMD_WRITE_NAME = 0x04
CMD_READ_FAN_PROFILE = 0x05
CMD_WRITE_FAN_PROFILE = 0x06
CMD_READ_RGB = 0x07
CMD_WRITE_RGB = 0x08
CMD_READ_CALIBRATION = 0x09
CMD_WRITE_CALIBRATION = 0x0A
CMD_LOAD_CALIBRATION = 0x0B
CMD_STORE_CALIBRATION = 0x0C
CMD_READ_UID = 0x0D
CMD_READ_VENDOR_DATA = 0x0E


@dataclass
class VendorData:
    """Vendor identification data (3 bytes)."""
    vendor_id: int  # Should be 0xEE for BenchLab
    product_id: int  # Should be 0x10 for standard device
    fw_version: int

    def is_valid(self) -> bool:
        return self.vendor_id == VENDOR_ID and self.product_id == PRODUCT_ID


@dataclass
class PowerSensor:
    """Power sensor measurements (10 bytes).
    All raw values in milli-units, converted properties in standard units.
    """
    voltage_mv: int  # millivolts
    current_ma: int  # milliamps
    power_mw: int    # milliwatts

    @property
    def voltage_v(self) -> float:
        return self.voltage_mv / 1000.0

    @property
    def current_a(self) -> float:
        return self.current_ma / 1000.0

    @property
    def power_w(self) -> float:
        return self.power_mw / 1000.0


@dataclass
class FanSensor:
    """Fan sensor and control data (4 bytes)."""
    enable: int  # 0=disabled, 1=enabled
    duty: int  # PWM duty cycle (0-255)
    tach: int  # Tachometer reading in RPM

    @property
    def is_enabled(self) -> bool:
        return self.enable != 0

    @property
    def duty_percent(self) -> float:
        return (self.duty / 255.0) * 100.0


@dataclass
class SensorData:
    """Complete sensor telemetry structure (~194 bytes).
    Maps to C# SensorStruct.
    """
    # Voltage inputs (13 channels, millivolts)
    vin: list[int]

    # Supply voltages
    vdd: int  # Digital supply voltage (millivolts)
    vref: int  # Reference voltage (millivolts)

    # Temperatures (all in degrees Celsius × 100)
    tchip: int  # Chip temperature
    ts: list[int]  # 4 temperature sensors
    tamb: int  # Ambient temperature

    # Humidity (percentage × 100)
    hum: int

    # Status bits
    fan_switch_status: int
    rgb_switch_status: int
    rgb_ext_status: int
    fan_ext_duty: int

    # Power measurements (11 channels)
    power: list[PowerSensor]

    # Fan measurements (9 channels)
    fans: list[FanSensor]

    @property
    def chip_temp_c(self) -> float:
        return self.tchip / 100.0

    @property
    def ambient_temp_c(self) -> float:
        return self.tamb / 100.0

    @property
    def humidity_percent(self) -> float:
        return self.hum / 100.0

    def get_temperatures_c(self) -> list[float]:
        """Get all temperatures in Celsius."""
        temps = [self.chip_temp_c]
        temps.extend([t / 100.0 for t in self.ts])
        temps.append(self.ambient_temp_c)
        return temps


@dataclass
class DeviceUid:
    """Device unique identifier (12 bytes, 96-bit)."""
    uid_low: int  # 8 bytes (uint64)
    uid_high: int  # 4 bytes (uint32)

    def to_hex_string(self) -> str:
        """Convert UID to hex string."""
        # Pack as little-endian and convert to hex
        low_bytes = self.uid_low.to_bytes(8, 'little')
        high_bytes = self.uid_high.to_bytes(4, 'little')
        return (low_bytes + high_bytes).hex().upper()


@dataclass
class FanProfile:
    """Fan profile configuration (8 bytes)."""
    mode: int  # 0=Manual, 1=Auto
    manual_duty: int  # Manual duty cycle (0-255) when mode=0
    temp_threshold: int  # Temperature threshold (degrees C × 100)
    min_duty: int  # Min duty in auto mode (0-255)
    max_duty: int  # Max duty in auto mode (0-255)
    sensor_index: int  # Temperature sensor index for auto mode
    reserved: int

    @property
    def is_auto_mode(self) -> bool:
        return self.mode == 1

    @property
    def temp_threshold_c(self) -> float:
        return self.temp_threshold / 100.0


@dataclass
class RgbConfig:
    """RGB LED configuration (8 bytes)."""
    mode: int  # 0=Off, 1=Solid, 2=Breathing, 3=Cycle, 4=Temperature
    red: int  # 0-255
    green: int  # 0-255
    blue: int  # 0-255
    brightness: int  # 0-255
    speed: int  # 0-255 (higher=faster)
    reserved1: int
    reserved2: int


@dataclass
class CalibrationData:
    """Calibration data structure.
    Stores offset and scaling factors for sensors.
    """
    voltage_offsets: list[int]  # 13 channels, millivolts
    voltage_scales: list[int]  # 13 channels, factor × 1000
    temp_offset: int  # degrees C × 100
    temp_scale: int  # factor × 1000
    current_offsets: list[int]  # 11 channels, milliamps
    current_scales: list[int]  # 11 channels, factor × 1000
    flags: int  # Bit field for which sensors are calibrated


class BinaryProtocol:
    """BenchLab binary UART protocol implementation.

    Provides all 15 protocol commands with timeout handling.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5):
        """Initialize protocol with serial port.

        Args:
            port: Serial port path (e.g., '/dev/benchlab0')
            baudrate: Baud rate (default: 115200)
            timeout: Command timeout in seconds (default: 0.5)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

    def open(self):
        """Open serial port connection."""
        if self._serial and self._serial.is_open:
            return

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout
        )
        # Enable DTR/RTS for STM32 VCP
        self._serial.dtr = True
        self._serial.rts = True
        time.sleep(0.1)  # Allow device to settle

    def close(self):
        """Close serial port connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def _send_command(self, cmd: int) -> bytes:
        """Send command byte and read response.

        Args:
            cmd: Command byte (0x00-0x0E)

        Returns:
            Response bytes

        Raises:
            TimeoutError: If response not received within timeout
            IOError: If serial communication fails
        """
        if not self._serial or not self._serial.is_open:
            raise IOError("Serial port not open")

        # Flush buffers
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        # Send command
        self._serial.write(bytes([cmd]))
        self._serial.flush()

        return self._serial

    def read_welcome(self) -> str:
        """Read welcome message (CMD 0x00).

        Returns:
            Welcome string ("BENCHLAB\\x00")
        """
        ser = self._send_command(CMD_WELCOME)
        data = ser.read(13)
        if len(data) != 13:
            raise TimeoutError("Welcome command timeout")
        return data.decode('ascii').rstrip('\x00')

    def read_vendor_data(self) -> VendorData:
        """Read vendor identification data (CMD 0x0E).

        Returns:
            VendorData with vendor ID, product ID, firmware version
        """
        ser = self._send_command(CMD_READ_VENDOR_DATA)
        data = ser.read(3)
        if len(data) != 3:
            raise TimeoutError("Vendor data command timeout")

        vendor_id, product_id, fw_version = struct.unpack('<BBB', data)
        return VendorData(vendor_id, product_id, fw_version)

    def read_device_name(self) -> str:
        """Read device name (CMD 0x03).

        Returns:
            Device name string (max 32 chars)
        """
        ser = self._send_command(CMD_READ_NAME)
        data = ser.read(32)
        if len(data) != 32:
            raise TimeoutError("Read name command timeout")
        return data.decode('ascii').rstrip('\x00')

    def write_device_name(self, name: str) -> int:
        """Write device name (CMD 0x04).

        Args:
            name: Device name (max 32 characters)

        Returns:
            Status code (0=success)
        """
        if len(name) > 32:
            raise ValueError("Name must be 32 characters or less")

        # Pad to 32 bytes
        name_bytes = name.encode('ascii').ljust(32, b'\x00')

        ser = self._send_command(CMD_WRITE_NAME)
        ser.write(name_bytes)
        ser.flush()

        # Read status byte
        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Write name command timeout")
        return status[0]

    def read_device_uid(self) -> DeviceUid:
        """Read device unique identifier (CMD 0x0D).

        Returns:
            DeviceUid with 96-bit unique ID
        """
        ser = self._send_command(CMD_READ_UID)
        data = ser.read(12)
        if len(data) != 12:
            raise TimeoutError("Read UID command timeout")

        uid_low, uid_high = struct.unpack('<QI', data)
        return DeviceUid(uid_low, uid_high)

    def read_sensors(self) -> SensorData:
        """Read all sensor telemetry (CMD 0x01).

        Returns:
            SensorData with ~194 bytes of telemetry
        """
        ser = self._send_command(CMD_READ_SENSORS)

        # Read fixed-size structure
        # Format: 13 shorts (vin), 2 ushorts (vdd, vref), short (tchip),
        #         4 shorts (ts), short (tamb), short (hum), 4 bytes (status),
        #         11x PowerSensor (110 bytes), 9x FanSensor (36 bytes)

        # Voltage inputs (13 × 2 bytes = 26 bytes)
        vin = list(struct.unpack('<13h', ser.read(26)))

        # Supply voltages (2 × 2 bytes = 4 bytes)
        vdd, vref = struct.unpack('<HH', ser.read(4))

        # Chip temp (2 bytes)
        tchip = struct.unpack('<h', ser.read(2))[0]

        # Temperature sensors (4 × 2 bytes = 8 bytes)
        ts = list(struct.unpack('<4h', ser.read(8)))

        # Ambient temp (2 bytes)
        tamb = struct.unpack('<h', ser.read(2))[0]

        # Humidity (2 bytes)
        hum = struct.unpack('<h', ser.read(2))[0]

        # Status bytes (4 bytes)
        fan_switch, rgb_switch, rgb_ext, fan_ext = struct.unpack('<BBBB', ser.read(4))

        # Power sensors (11 × 10 bytes = 110 bytes)
        power = []
        for _ in range(SENSOR_POWER_COUNT):
            voltage_mv, current_ma, power_mw = struct.unpack('<hii', ser.read(10))
            power.append(PowerSensor(voltage_mv, current_ma, power_mw))

        # Fan sensors (9 × 4 bytes = 36 bytes)
        fans = []
        for _ in range(FAN_COUNT):
            enable, duty, tach = struct.unpack('<BBH', ser.read(4))
            fans.append(FanSensor(enable, duty, tach))

        return SensorData(
            vin=vin, vdd=vdd, vref=vref,
            tchip=tchip, ts=ts, tamb=tamb, hum=hum,
            fan_switch_status=fan_switch,
            rgb_switch_status=rgb_switch,
            rgb_ext_status=rgb_ext,
            fan_ext_duty=fan_ext,
            power=power, fans=fans
        )

    def execute_action(self, action_id: int) -> int:
        """Execute device action (CMD 0x02).

        Args:
            action_id: Action ID (0-255)

        Returns:
            Status code (0=success)
        """
        if not 0 <= action_id <= 255:
            raise ValueError("Action ID must be 0-255")

        ser = self._send_command(CMD_ACTION)
        ser.write(bytes([action_id]))
        ser.flush()

        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Action command timeout")
        return status[0]

    def read_fan_profile(self, fan_index: int) -> FanProfile:
        """Read fan profile configuration (CMD 0x05).

        Args:
            fan_index: Fan index (0-8)

        Returns:
            FanProfile with configuration
        """
        if not 0 <= fan_index <= 8:
            raise ValueError("Fan index must be 0-8")

        ser = self._send_command(CMD_READ_FAN_PROFILE)
        ser.write(bytes([fan_index]))
        ser.flush()

        data = ser.read(8)
        if len(data) != 8:
            raise TimeoutError("Read fan profile timeout")

        mode, manual_duty, temp_threshold, min_duty, max_duty, sensor_idx, reserved = \
            struct.unpack('<BBhBBBB', data)

        return FanProfile(mode, manual_duty, temp_threshold,
                          min_duty, max_duty, sensor_idx, reserved)

    def write_fan_profile(self, fan_index: int, profile: FanProfile) -> int:
        """Write fan profile configuration (CMD 0x06).

        Args:
            fan_index: Fan index (0-8)
            profile: FanProfile to write

        Returns:
            Status code (0=success)
        """
        if not 0 <= fan_index <= 8:
            raise ValueError("Fan index must be 0-8")

        ser = self._send_command(CMD_WRITE_FAN_PROFILE)
        ser.write(bytes([fan_index]))

        # Pack profile struct (8 bytes)
        data = struct.pack('<BBhBBBB',
                           profile.mode, profile.manual_duty,
                           profile.temp_threshold,
                           profile.min_duty, profile.max_duty,
                           profile.sensor_index, profile.reserved)
        ser.write(data)
        ser.flush()

        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Write fan profile timeout")
        return status[0]

    def read_rgb(self) -> RgbConfig:
        """Read RGB LED configuration (CMD 0x07).

        Returns:
            RgbConfig with LED settings
        """
        ser = self._send_command(CMD_READ_RGB)
        data = ser.read(8)
        if len(data) != 8:
            raise TimeoutError("Read RGB timeout")

        mode, red, green, blue, brightness, speed, res1, res2 = \
            struct.unpack('<BBBBBBBB', data)

        return RgbConfig(mode, red, green, blue, brightness, speed, res1, res2)

    def write_rgb(self, config: RgbConfig) -> int:
        """Write RGB LED configuration (CMD 0x08).

        Args:
            config: RgbConfig to write

        Returns:
            Status code (0=success)
        """
        ser = self._send_command(CMD_WRITE_RGB)

        # Pack RGB struct (8 bytes)
        data = struct.pack('<BBBBBBBB',
                           config.mode, config.red, config.green, config.blue,
                           config.brightness, config.speed,
                           config.reserved1, config.reserved2)
        ser.write(data)
        ser.flush()

        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Write RGB timeout")
        return status[0]

    def read_calibration(self) -> CalibrationData:
        """Read calibration data from RAM (CMD 0x09).

        Returns:
            CalibrationData with sensor calibration
        """
        ser = self._send_command(CMD_READ_CALIBRATION)

        # Voltage offsets (13 × 2 bytes = 26 bytes)
        voltage_offsets = list(struct.unpack('<13h', ser.read(26)))

        # Voltage scales (13 × 2 bytes = 26 bytes)
        voltage_scales = list(struct.unpack('<13h', ser.read(26)))

        # Temperature offset and scale (2 + 2 = 4 bytes)
        temp_offset, temp_scale = struct.unpack('<hh', ser.read(4))

        # Current offsets (11 × 4 bytes = 44 bytes)
        current_offsets = list(struct.unpack('<11i', ser.read(44)))

        # Current scales (11 × 2 bytes = 22 bytes)
        current_scales = list(struct.unpack('<11h', ser.read(22)))

        # Flags (4 bytes)
        flags = struct.unpack('<I', ser.read(4))[0]

        return CalibrationData(
            voltage_offsets=voltage_offsets,
            voltage_scales=voltage_scales,
            temp_offset=temp_offset,
            temp_scale=temp_scale,
            current_offsets=current_offsets,
            current_scales=current_scales,
            flags=flags
        )

    def write_calibration(self, cal: CalibrationData) -> int:
        """Write calibration data to RAM (CMD 0x0A).

        Args:
            cal: CalibrationData to write

        Returns:
            Status code (0=success)
        """
        ser = self._send_command(CMD_WRITE_CALIBRATION)

        # Pack calibration struct
        data = struct.pack('<13h', *cal.voltage_offsets)
        data += struct.pack('<13h', *cal.voltage_scales)
        data += struct.pack('<hh', cal.temp_offset, cal.temp_scale)
        data += struct.pack('<11i', *cal.current_offsets)
        data += struct.pack('<11h', *cal.current_scales)
        data += struct.pack('<I', cal.flags)

        ser.write(data)
        ser.flush()

        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Write calibration timeout")
        return status[0]

    def load_calibration(self) -> int:
        """Load calibration from flash to RAM (CMD 0x0B).

        Returns:
            Status code (0=success, 1=no calibration in flash)
        """
        ser = self._send_command(CMD_LOAD_CALIBRATION)
        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Load calibration timeout")
        return status[0]

    def store_calibration(self) -> int:
        """Store calibration from RAM to flash (CMD 0x0C).

        Returns:
            Status code (0=success)
        """
        ser = self._send_command(CMD_STORE_CALIBRATION)
        status = ser.read(1)
        if len(status) != 1:
            raise TimeoutError("Store calibration timeout")
        return status[0]
