using System.Runtime.InteropServices;

namespace BenchLab.Platform.Protocol;

/// <summary>
/// Vendor identification data (3 bytes).
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct VendorDataStruct
{
    /// <summary>
    /// Vendor ID (should be 0xEE for BenchLab).
    /// </summary>
    public byte VendorId;

    /// <summary>
    /// Product ID (should be 0x10 for standard BenchLab device).
    /// </summary>
    public byte ProductId;

    /// <summary>
    /// Firmware version number.
    /// </summary>
    public byte FwVersion;

    /// <summary>
    /// Validates that this is a BenchLab device.
    /// </summary>
    public readonly bool IsValid =>
        VendorId == ProtocolConstants.VendorId &&
        ProductId == ProtocolConstants.ProductId;
}

/// <summary>
/// Power sensor measurements (10 bytes).
/// All values are in milli-units (millivolts, milliamps, milliwatts).
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PowerSensor
{
    /// <summary>
    /// Voltage in millivolts.
    /// </summary>
    public short Voltage;

    /// <summary>
    /// Current in milliamps.
    /// </summary>
    public int Current;

    /// <summary>
    /// Power in milliwatts.
    /// </summary>
    public int Power;

    /// <summary>
    /// Gets voltage in volts.
    /// </summary>
    public readonly double VoltageVolts => Voltage / 1000.0;

    /// <summary>
    /// Gets current in amps.
    /// </summary>
    public readonly double CurrentAmps => Current / 1000.0;

    /// <summary>
    /// Gets power in watts.
    /// </summary>
    public readonly double PowerWatts => Power / 1000.0;
}

/// <summary>
/// Fan sensor and control data (4 bytes).
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct FanSensor
{
    /// <summary>
    /// Fan enable status (0=disabled, 1=enabled).
    /// </summary>
    public byte Enable;

    /// <summary>
    /// PWM duty cycle (0-255).
    /// </summary>
    public byte Duty;

    /// <summary>
    /// Tachometer reading in RPM.
    /// </summary>
    public ushort Tach;

    /// <summary>
    /// Gets whether the fan is enabled.
    /// </summary>
    public readonly bool IsEnabled => Enable != 0;

    /// <summary>
    /// Gets duty cycle as percentage (0-100).
    /// </summary>
    public readonly double DutyPercent => (Duty / 255.0) * 100.0;
}

/// <summary>
/// Complete sensor telemetry structure (~194 bytes).
/// This is the primary data payload returned by UART_CMD_READ_SENSORS.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public unsafe struct SensorStruct
{
    /// <summary>
    /// Voltage input channels in millivolts (13 channels).
    /// Typical channels: ATX 24V, 12V, 5V, 5VSB, 3.3V rails.
    /// </summary>
    public fixed short Vin[ProtocolConstants.SensorVinCount];

    /// <summary>
    /// Digital supply voltage in millivolts.
    /// </summary>
    public ushort Vdd;

    /// <summary>
    /// Reference voltage in millivolts.
    /// </summary>
    public ushort Vref;

    /// <summary>
    /// Chip temperature in degrees Celsius × 100.
    /// </summary>
    public short Tchip;

    /// <summary>
    /// Additional temperature sensors in degrees Celsius × 100 (4 sensors).
    /// </summary>
    public fixed short Ts[ProtocolConstants.TemperatureSensorCount];

    /// <summary>
    /// Ambient temperature in degrees Celsius × 100.
    /// </summary>
    public short Tamb;

    /// <summary>
    /// Humidity in percentage × 100.
    /// </summary>
    public short Hum;

    /// <summary>
    /// Fan switch status bits.
    /// </summary>
    public byte FanSwitchStatus;

    /// <summary>
    /// RGB switch status bits.
    /// </summary>
    public byte RGBSwitchStatus;

    /// <summary>
    /// External RGB status.
    /// </summary>
    public byte RGBExtStatus;

    /// <summary>
    /// External fan duty cycle.
    /// </summary>
    public byte FanExtDuty;

    // Note: The following arrays cannot use 'fixed' because they are not primitive types.
    // We need to use a workaround with sequential layout.

    /// <summary>
    /// Power measurements for all power channels (11 channels, 110 bytes total).
    /// </summary>
    public PowerSensor Power0;
    public PowerSensor Power1;
    public PowerSensor Power2;
    public PowerSensor Power3;
    public PowerSensor Power4;
    public PowerSensor Power5;
    public PowerSensor Power6;
    public PowerSensor Power7;
    public PowerSensor Power8;
    public PowerSensor Power9;
    public PowerSensor Power10;

    /// <summary>
    /// Fan sensor data for all fan channels (9 channels, 36 bytes total).
    /// </summary>
    public FanSensor Fan0;
    public FanSensor Fan1;
    public FanSensor Fan2;
    public FanSensor Fan3;
    public FanSensor Fan4;
    public FanSensor Fan5;
    public FanSensor Fan6;
    public FanSensor Fan7;
    public FanSensor Fan8;

    /// <summary>
    /// Gets power readings as an array.
    /// </summary>
    public readonly PowerSensor[] GetPowerReadings()
    {
        return new[]
        {
            Power0, Power1, Power2, Power3, Power4, Power5,
            Power6, Power7, Power8, Power9, Power10
        };
    }

    /// <summary>
    /// Gets fan readings as an array.
    /// </summary>
    public readonly FanSensor[] GetFans()
    {
        return new[] { Fan0, Fan1, Fan2, Fan3, Fan4, Fan5, Fan6, Fan7, Fan8 };
    }

    /// <summary>
    /// Gets voltage inputs as an array (in millivolts).
    /// </summary>
    public readonly short[] GetVoltages()
    {
        var voltages = new short[ProtocolConstants.SensorVinCount];
        fixed (short* ptr = Vin)
        {
            for (int i = 0; i < ProtocolConstants.SensorVinCount; i++)
            {
                voltages[i] = ptr[i];
            }
        }
        return voltages;
    }

    /// <summary>
    /// Gets temperature sensors as an array (in degrees Celsius).
    /// </summary>
    public readonly double[] GetTemperatures()
    {
        var temps = new double[ProtocolConstants.TemperatureSensorCount + 2]; // Ts + Tchip + Tamb
        temps[0] = Tchip / 100.0;
        fixed (short* ptr = Ts)
        {
            for (int i = 0; i < ProtocolConstants.TemperatureSensorCount; i++)
            {
                temps[i + 1] = ptr[i] / 100.0;
            }
        }
        temps[ProtocolConstants.TemperatureSensorCount + 1] = Tamb / 100.0;
        return temps;
    }

    /// <summary>
    /// Gets humidity as percentage.
    /// </summary>
    public readonly double HumidityPercent => Hum / 100.0;

    /// <summary>
    /// Gets chip temperature in degrees Celsius.
    /// </summary>
    public readonly double ChipTemperature => Tchip / 100.0;

    /// <summary>
    /// Gets ambient temperature in degrees Celsius.
    /// </summary>
    public readonly double AmbientTemperature => Tamb / 100.0;
}

/// <summary>
/// Device unique identifier structure.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DeviceUidStruct
{
    /// <summary>
    /// 96-bit (12 bytes) unique device identifier from STM32.
    /// </summary>
    public ulong UidLow;   // First 8 bytes
    public uint UidHigh;    // Last 4 bytes

    /// <summary>
    /// Gets UID as hexadecimal string.
    /// </summary>
    public readonly string ToHexString()
    {
        var bytes = new byte[12];
        BitConverter.GetBytes(UidLow).CopyTo(bytes, 0);
        BitConverter.GetBytes(UidHigh).CopyTo(bytes, 8);
        return BitConverter.ToString(bytes).Replace("-", "");
    }
}

/// <summary>
/// Fan profile configuration structure.
/// Controls how a fan responds to temperature or manual control.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct FanProfileStruct
{
    /// <summary>
    /// Fan mode: 0=Manual, 1=Auto (temperature-based).
    /// </summary>
    public byte Mode;

    /// <summary>
    /// Manual duty cycle (0-255) when Mode=0.
    /// </summary>
    public byte ManualDuty;

    /// <summary>
    /// Temperature threshold for auto mode (degrees C × 100).
    /// </summary>
    public short TempThreshold;

    /// <summary>
    /// Minimum duty cycle in auto mode (0-255).
    /// </summary>
    public byte MinDuty;

    /// <summary>
    /// Maximum duty cycle in auto mode (0-255).
    /// </summary>
    public byte MaxDuty;

    /// <summary>
    /// Temperature sensor index to use for auto mode.
    /// </summary>
    public byte SensorIndex;

    /// <summary>
    /// Reserved for future use.
    /// </summary>
    public byte Reserved;

    /// <summary>
    /// Gets whether fan is in auto mode.
    /// </summary>
    public readonly bool IsAutoMode => Mode == 1;
}

/// <summary>
/// RGB LED configuration structure.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct RgbStruct
{
    /// <summary>
    /// RGB mode: 0=Off, 1=Solid, 2=Breathing, 3=Cycle, 4=Temperature.
    /// </summary>
    public byte Mode;

    /// <summary>
    /// Red channel (0-255).
    /// </summary>
    public byte Red;

    /// <summary>
    /// Green channel (0-255).
    /// </summary>
    public byte Green;

    /// <summary>
    /// Blue channel (0-255).
    /// </summary>
    public byte Blue;

    /// <summary>
    /// Brightness (0-255).
    /// </summary>
    public byte Brightness;

    /// <summary>
    /// Animation speed for modes 2-4 (0-255, higher=faster).
    /// </summary>
    public byte Speed;

    /// <summary>
    /// Reserved for future use.
    /// </summary>
    public byte Reserved1;
    public byte Reserved2;
}

/// <summary>
/// Calibration data structure.
/// Stores offset and scaling factors for sensors.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public unsafe struct CalibrationStruct
{
    /// <summary>
    /// Voltage calibration offsets (13 channels, millivolts).
    /// </summary>
    public fixed short VoltageOffsets[ProtocolConstants.SensorVinCount];

    /// <summary>
    /// Voltage calibration scales (13 channels, factor × 1000).
    /// </summary>
    public fixed short VoltageScales[ProtocolConstants.SensorVinCount];

    /// <summary>
    /// Temperature calibration offset (degrees C × 100).
    /// </summary>
    public short TempOffset;

    /// <summary>
    /// Temperature calibration scale (factor × 1000).
    /// </summary>
    public short TempScale;

    /// <summary>
    /// Current calibration offsets (11 channels, milliamps).
    /// </summary>
    public fixed int CurrentOffsets[ProtocolConstants.SensorPowerCount];

    /// <summary>
    /// Current calibration scales (11 channels, factor × 1000).
    /// </summary>
    public fixed short CurrentScales[ProtocolConstants.SensorPowerCount];

    /// <summary>
    /// Calibration flags (bit field for which sensors are calibrated).
    /// </summary>
    public uint Flags;
}
