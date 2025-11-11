namespace BenchLab.Platform.Protocol;

/// <summary>
/// UART command codes for BenchLab device communication.
/// Based on BENCHLAB_Core protocol specification.
/// </summary>
public enum UartCommand : byte
{
    /// <summary>
    /// Device identification handshake.
    /// Sends: 0x00
    /// Expects: 13 bytes containing "BENCHLAB\x00"
    /// </summary>
    Welcome = 0x00,

    /// <summary>
    /// Fetch complete sensor telemetry data.
    /// Sends: 0x01
    /// Expects: ~194 bytes (SensorStruct)
    /// </summary>
    ReadSensors = 0x01,

    /// <summary>
    /// Execute device action.
    /// Sends: 0x02 + action parameters
    /// Expects: Status byte
    /// </summary>
    Action = 0x02,

    /// <summary>
    /// Get device name.
    /// Sends: 0x03
    /// Expects: 32 bytes (ASCII string)
    /// </summary>
    ReadName = 0x03,

    /// <summary>
    /// Set device name.
    /// Sends: 0x04 + 32 bytes (ASCII string)
    /// Expects: Status confirmation
    /// </summary>
    WriteName = 0x04,

    /// <summary>
    /// Retrieve fan profile settings.
    /// Sends: 0x05
    /// Expects: Binary fan profile structure
    /// </summary>
    ReadFanProfile = 0x05,

    /// <summary>
    /// Configure fan behavior.
    /// Sends: 0x06 + fan profile data
    /// Expects: Status confirmation
    /// </summary>
    WriteFanProfile = 0x06,

    /// <summary>
    /// Get RGB LED settings.
    /// Sends: 0x07
    /// Expects: Binary RGB structure
    /// </summary>
    ReadRgb = 0x07,

    /// <summary>
    /// Configure RGB LEDs.
    /// Sends: 0x08 + RGB data
    /// Expects: Status confirmation
    /// </summary>
    WriteRgb = 0x08,

    /// <summary>
    /// Retrieve calibration data.
    /// Sends: 0x09
    /// Expects: Binary calibration structure
    /// </summary>
    ReadCalibration = 0x09,

    /// <summary>
    /// Apply calibration values.
    /// Sends: 0x0A + calibration data
    /// Expects: Status confirmation
    /// </summary>
    WriteCalibration = 0x0A,

    /// <summary>
    /// Load stored calibration from flash.
    /// Sends: 0x0B
    /// Expects: Status confirmation
    /// </summary>
    LoadCalibration = 0x0B,

    /// <summary>
    /// Save calibration to flash memory.
    /// Sends: 0x0C
    /// Expects: Status confirmation
    /// </summary>
    StoreCalibration = 0x0C,

    /// <summary>
    /// Read unique device identifier.
    /// Sends: 0x0D
    /// Expects: Binary UID structure
    /// </summary>
    ReadUid = 0x0D,

    /// <summary>
    /// Get vendor information (VendorId, ProductId, FwVersion).
    /// Sends: 0x0E
    /// Expects: 3 bytes (VendorDataStruct)
    /// </summary>
    ReadVendorData = 0x0E
}

/// <summary>
/// Protocol constants for BenchLab device communication.
/// </summary>
public static class ProtocolConstants
{
    /// <summary>
    /// Expected welcome response string.
    /// </summary>
    public const string WelcomeResponse = "BENCHLAB";

    /// <summary>
    /// BenchLab vendor ID.
    /// </summary>
    public const byte VendorId = 0xEE;

    /// <summary>
    /// BenchLab product ID.
    /// </summary>
    public const byte ProductId = 0x10;

    /// <summary>
    /// USB Vendor ID (STMicroelectronics).
    /// </summary>
    public const ushort UsbVendorId = 0x0483;

    /// <summary>
    /// USB Product ID (Virtual COM Port).
    /// </summary>
    public const ushort UsbProductId = 0x5740;

    /// <summary>
    /// Number of voltage input channels.
    /// </summary>
    public const int SensorVinCount = 13;

    /// <summary>
    /// Number of power measurement channels.
    /// </summary>
    public const int SensorPowerCount = 11;

    /// <summary>
    /// Number of fan channels.
    /// </summary>
    public const int FanCount = 9;

    /// <summary>
    /// Number of temperature sensors.
    /// </summary>
    public const int TemperatureSensorCount = 4;

    /// <summary>
    /// Device name length in bytes.
    /// </summary>
    public const int DeviceNameLength = 32;

    /// <summary>
    /// Welcome response length in bytes.
    /// </summary>
    public const int WelcomeResponseLength = 13;

    /// <summary>
    /// Vendor data struct length in bytes.
    /// </summary>
    public const int VendorDataLength = 3;
}
