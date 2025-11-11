using System.Runtime.InteropServices;
using System.Text;
using Xunit;
using FluentAssertions;
using Moq;
using BenchLab.Platform.Protocol;
using BenchLab.Platform.Ports;

namespace BenchLab.Platform.Tests.Protocol;

/// <summary>
/// Comprehensive tests for all 15 UART protocol commands.
/// Tests valid inputs, boundary conditions, error scenarios, and struct marshalling.
/// </summary>
public class CommandTests
{
    private readonly TimeSpan _timeout = TimeSpan.FromSeconds(1);

    #region Test Helpers

    private Mock<ISerialPort> CreateMockPort()
    {
        var mock = new Mock<ISerialPort>();
        mock.Setup(p => p.PortName).Returns("/dev/test");
        return mock;
    }

    private void SetupReadResponse(Mock<ISerialPort> mockPort, byte[] response)
    {
        var position = 0;
        mockPort.Setup(p => p.BytesToRead).Returns(() => response.Length - position);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                var bytesToCopy = Math.Min(count, response.Length - position);
                Array.Copy(response, position, buffer, offset, bytesToCopy);
                position += bytesToCopy;
            })
            .Returns<byte[], int, int>((buffer, offset, count) =>
            {
                return Math.Min(count, response.Length - position);
            });
    }

    private byte[] StructToBytes<T>(T structure) where T : struct
    {
        var size = Marshal.SizeOf(structure);
        var bytes = new byte[size];
        var ptr = Marshal.AllocHGlobal(size);
        try
        {
            Marshal.StructureToPtr(structure, ptr, false);
            Marshal.Copy(ptr, bytes, 0, size);
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }
        return bytes;
    }

    #endregion

    #region Command 0x00: Welcome

    [Fact]
    public void Welcome_ValidResponse_ReturnsTrue()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var welcomeBytes = new byte[13];
        Encoding.ASCII.GetBytes("BENCHLAB").CopyTo(welcomeBytes, 0);
        SetupReadResponse(mockPort, welcomeBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.PerformHandshake();

        // Assert
        result.Should().BeTrue();
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.Welcome), 0, 1), Times.Once);
    }

    [Fact]
    public void Welcome_InvalidResponse_ReturnsFalse()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var invalidBytes = new byte[13];
        Encoding.ASCII.GetBytes("WRONGDEV").CopyTo(invalidBytes, 0);
        SetupReadResponse(mockPort, invalidBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.PerformHandshake();

        // Assert
        result.Should().BeFalse();
    }

    [Fact]
    public void Welcome_EmptyResponse_ReturnsFalse()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var emptyBytes = new byte[13];
        SetupReadResponse(mockPort, emptyBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.PerformHandshake();

        // Assert
        result.Should().BeFalse();
    }

    #endregion

    #region Command 0x01: ReadSensors

    [Fact]
    public unsafe void ReadSensors_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var sensorData = new SensorStruct
        {
            Tchip = 4520,  // 45.20°C
            Tamb = 2315,   // 23.15°C
            Hum = 5500     // 55.00%
        };

        // Set some voltage/power/fan data
        fixed (short* v = sensorData.Voltages)
        {
            v[0] = 5000;   // 5V
            v[1] = 12000;  // 12V
        }

        var responseBytes = StructToBytes(sensorData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadSensors();

        // Assert
        result.ChipTemperature.Should().Be(45.2);
        result.AmbientTemperature.Should().Be(23.15);
        result.HumidityPercent.Should().Be(55.0);
        result.GetVoltages()[0].Should().Be(5000);
        result.GetVoltages()[1].Should().Be(12000);

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadSensors), 0, 1), Times.Once);
    }

    [Fact]
    public unsafe void ReadSensors_BoundaryTemperatures_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var sensorData = new SensorStruct
        {
            Tchip = -1000,  // -10.00°C
            Tamb = 12500,   // 125.00°C (max typical)
            Hum = 0
        };

        var responseBytes = StructToBytes(sensorData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadSensors();

        // Assert
        result.ChipTemperature.Should().Be(-10.0);
        result.AmbientTemperature.Should().Be(125.0);
        result.HumidityPercent.Should().Be(0.0);
    }

    [Fact]
    public unsafe void ReadSensors_AllPowerChannels_ReturnsCorrectReadings()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var sensorData = new SensorStruct();

        fixed (PowerSensor* p = sensorData.Power)
        {
            p[0] = new PowerSensor { Voltage = 5000, Current = 1000, Power = 5000 };    // 5V, 1A, 5W
            p[1] = new PowerSensor { Voltage = 12000, Current = 3000, Power = 36000 };  // 12V, 3A, 36W
            p[2] = new PowerSensor { Voltage = 3300, Current = 500, Power = 1650 };     // 3.3V, 0.5A, 1.65W
        }

        var responseBytes = StructToBytes(sensorData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadSensors();
        var powerReadings = result.GetPowerReadings();

        // Assert
        powerReadings[0].VoltageVolts.Should().Be(5.0);
        powerReadings[0].CurrentAmps.Should().Be(1.0);
        powerReadings[0].PowerWatts.Should().Be(5.0);

        powerReadings[1].VoltageVolts.Should().Be(12.0);
        powerReadings[1].CurrentAmps.Should().Be(3.0);
        powerReadings[1].PowerWatts.Should().Be(36.0);

        powerReadings[2].VoltageVolts.Should().Be(3.3);
        powerReadings[2].CurrentAmps.Should().Be(0.5);
        powerReadings[2].PowerWatts.Should().Be(1.65);
    }

    [Fact]
    public unsafe void ReadSensors_AllFans_ReturnsCorrectData()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var sensorData = new SensorStruct();

        fixed (FanSensor* f = sensorData.Fans)
        {
            f[0] = new FanSensor { Enable = 1, Duty = 127, Tach = 1200 };  // ~50% duty, 1200 RPM
            f[1] = new FanSensor { Enable = 1, Duty = 255, Tach = 2400 };  // 100% duty, 2400 RPM
            f[2] = new FanSensor { Enable = 0, Duty = 0, Tach = 0 };       // Disabled
        }

        var responseBytes = StructToBytes(sensorData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadSensors();
        var fans = result.GetFans();

        // Assert
        fans[0].IsEnabled.Should().BeTrue();
        fans[0].DutyPercent.Should().BeApproximately(49.8, 0.5);
        fans[0].Tach.Should().Be(1200);

        fans[1].IsEnabled.Should().BeTrue();
        fans[1].DutyPercent.Should().Be(100.0);
        fans[1].Tach.Should().Be(2400);

        fans[2].IsEnabled.Should().BeFalse();
        fans[2].DutyPercent.Should().Be(0.0);
    }

    #endregion

    #region Command 0x02: Action

    [Fact]
    public void Action_SendCommand_WritesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()));

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        protocol.SendAction(42);

        // Assert
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.Action && b[1] == 42), 0, 2), Times.Once);
    }

    [Fact]
    public void Action_BoundaryValues_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()));

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act & Assert - Min value
        protocol.SendAction(0);
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[1] == 0), 0, 2), Times.Once);

        // Act & Assert - Max value
        protocol.SendAction(255);
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[1] == 255), 0, 2), Times.Once);
    }

    #endregion

    #region Command 0x03: ReadDeviceName

    [Fact]
    public void ReadDeviceName_ValidName_ReturnsCorrectString()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var nameBytes = new byte[32];
        Encoding.UTF8.GetBytes("My BenchLab Device").CopyTo(nameBytes, 0);
        SetupReadResponse(mockPort, nameBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceName();

        // Assert
        result.Should().Be("My BenchLab Device");
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadDeviceName), 0, 1), Times.Once);
    }

    [Fact]
    public void ReadDeviceName_EmptyName_ReturnsEmptyString()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var nameBytes = new byte[32];
        SetupReadResponse(mockPort, nameBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceName();

        // Assert
        result.Should().BeEmpty();
    }

    [Fact]
    public void ReadDeviceName_MaxLength_ReturnsFullString()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var nameBytes = new byte[32];
        var longName = new string('X', 31); // Max 31 chars + null terminator
        Encoding.UTF8.GetBytes(longName).CopyTo(nameBytes, 0);
        SetupReadResponse(mockPort, nameBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceName();

        // Assert
        result.Should().Be(longName);
    }

    [Fact]
    public void ReadDeviceName_SpecialCharacters_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var nameBytes = new byte[32];
        var specialName = "Test-Device_123!@#";
        Encoding.UTF8.GetBytes(specialName).CopyTo(nameBytes, 0);
        SetupReadResponse(mockPort, nameBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceName();

        // Assert
        result.Should().Be(specialName);
    }

    #endregion

    #region Command 0x04: WriteDeviceName

    [Fact]
    public void WriteDeviceName_ValidName_WritesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        protocol.WriteDeviceName("TestDevice");

        // Assert
        capturedBytes[0].Should().Be((byte)UartCommand.WriteDeviceName);
        var writtenName = Encoding.UTF8.GetString(capturedBytes.Skip(1).Take(32).ToArray()).TrimEnd('\0');
        writtenName.Should().Be("TestDevice");
    }

    [Fact]
    public void WriteDeviceName_EmptyName_WritesZeros()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        protocol.WriteDeviceName("");

        // Assert
        capturedBytes.Skip(1).Take(32).Should().AllBeEquivalentTo((byte)0);
    }

    [Fact]
    public void WriteDeviceName_LongName_TruncatesTo31Chars()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);
        var longName = new string('A', 50);

        // Act
        protocol.WriteDeviceName(longName);

        // Assert
        var writtenName = Encoding.UTF8.GetString(capturedBytes.Skip(1).Take(32).ToArray()).TrimEnd('\0');
        writtenName.Length.Should().BeLessOrEqualTo(31);
    }

    #endregion

    #region Command 0x05: ReadFanProfile

    [Fact]
    public void ReadFanProfile_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var profileData = new FanProfileStruct
        {
            Mode = 1,           // Auto mode
            ManualDuty = 0,
            TempThreshold = 5000,  // 50°C
            MinDuty = 30,
            MaxDuty = 100,
            SensorIndex = 0,
            Reserved = 0
        };

        var responseBytes = StructToBytes(profileData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadFanProfile(2); // Fan index 2

        // Assert
        result.IsAutoMode.Should().BeTrue();
        result.TempThreshold.Should().Be(5000);
        result.MinDuty.Should().Be(30);
        result.MaxDuty.Should().Be(100);
        result.SensorIndex.Should().Be(0);

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadFanProfile && b[1] == 2), 0, 2), Times.Once);
    }

    [Fact]
    public void ReadFanProfile_ManualMode_ReturnsCorrectData()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var profileData = new FanProfileStruct
        {
            Mode = 0,           // Manual mode
            ManualDuty = 75,
            TempThreshold = 0,
            MinDuty = 0,
            MaxDuty = 0,
            SensorIndex = 0,
            Reserved = 0
        };

        var responseBytes = StructToBytes(profileData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadFanProfile(0);

        // Assert
        result.IsAutoMode.Should().BeFalse();
        result.ManualDuty.Should().Be(75);
    }

    [Fact]
    public void ReadFanProfile_BoundaryFanIndex_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var profileData = new FanProfileStruct();
        var responseBytes = StructToBytes(profileData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act & Assert - Min index
        protocol.ReadFanProfile(0);
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[1] == 0), 0, 2), Times.Once);

        // Act & Assert - Max index (8 fans, 0-8 = 9 total)
        protocol.ReadFanProfile(8);
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[1] == 8), 0, 2), Times.Once);
    }

    #endregion

    #region Command 0x06: WriteFanProfile

    [Fact]
    public void WriteFanProfile_ValidData_WritesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);
        var profile = new FanProfileStruct
        {
            Mode = 1,
            ManualDuty = 0,
            TempThreshold = 4500,
            MinDuty = 25,
            MaxDuty = 100,
            SensorIndex = 1,
            Reserved = 0
        };

        // Act
        protocol.WriteFanProfile(3, profile);

        // Assert
        capturedBytes[0].Should().Be((byte)UartCommand.WriteFanProfile);
        capturedBytes[1].Should().Be(3); // Fan index
        var writtenProfile = MemoryMarshal.Read<FanProfileStruct>(capturedBytes.Skip(2).ToArray().AsSpan());
        writtenProfile.Mode.Should().Be(1);
        writtenProfile.TempThreshold.Should().Be(4500);
    }

    #endregion

    #region Command 0x07: ReadRgb

    [Fact]
    public void ReadRgb_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var rgbData = new RgbStruct
        {
            Mode = 2,    // Pattern mode
            Red = 255,
            Green = 128,
            Blue = 64,
            Speed = 50,
            Brightness = 200
        };

        var responseBytes = StructToBytes(rgbData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadRgb();

        // Assert
        result.Mode.Should().Be(2);
        result.Red.Should().Be(255);
        result.Green.Should().Be(128);
        result.Blue.Should().Be(64);
        result.Speed.Should().Be(50);
        result.Brightness.Should().Be(200);

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadRgb), 0, 1), Times.Once);
    }

    [Fact]
    public void ReadRgb_AllColors_ReturnsCorrectValues()
    {
        // Arrange
        var mockPort = CreateMockPort();

        // Test pure colors
        var testCases = new[]
        {
            (r: (byte)255, g: (byte)0, b: (byte)0, name: "Red"),
            (r: (byte)0, g: (byte)255, b: (byte)0, name: "Green"),
            (r: (byte)0, g: (byte)0, b: (byte)255, name: "Blue"),
            (r: (byte)255, g: (byte)255, b: (byte)255, name: "White"),
            (r: (byte)0, g: (byte)0, b: (byte)0, name: "Black")
        };

        foreach (var testCase in testCases)
        {
            var rgbData = new RgbStruct
            {
                Mode = 1,
                Red = testCase.r,
                Green = testCase.g,
                Blue = testCase.b,
                Speed = 0,
                Brightness = 255
            };

            var responseBytes = StructToBytes(rgbData);
            SetupReadResponse(mockPort, responseBytes);

            var protocol = new BinaryProtocol(mockPort.Object, _timeout);

            // Act
            var result = protocol.ReadRgb();

            // Assert
            result.Red.Should().Be(testCase.r, $"because testing {testCase.name}");
            result.Green.Should().Be(testCase.g, $"because testing {testCase.name}");
            result.Blue.Should().Be(testCase.b, $"because testing {testCase.name}");
        }
    }

    #endregion

    #region Command 0x08: WriteRgb

    [Fact]
    public void WriteRgb_ValidData_WritesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);
        var rgb = new RgbStruct
        {
            Mode = 3,
            Red = 200,
            Green = 150,
            Blue = 100,
            Speed = 25,
            Brightness = 180
        };

        // Act
        protocol.WriteRgb(rgb);

        // Assert
        capturedBytes[0].Should().Be((byte)UartCommand.WriteRgb);
        var writtenRgb = MemoryMarshal.Read<RgbStruct>(capturedBytes.Skip(1).ToArray().AsSpan());
        writtenRgb.Red.Should().Be(200);
        writtenRgb.Green.Should().Be(150);
        writtenRgb.Blue.Should().Be(100);
    }

    #endregion

    #region Command 0x09: ReadCalibration

    [Fact]
    public void ReadCalibration_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var calData = new CalibrationStruct
        {
            VoltageOffset = 100,    // +100 mV offset
            CurrentOffset = -50,    // -50 mA offset
            TempOffset = 25,        // +2.5°C offset
            ChecksumOrFlags = 0xAB
        };

        var responseBytes = StructToBytes(calData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadCalibration();

        // Assert
        result.VoltageOffset.Should().Be(100);
        result.CurrentOffset.Should().Be(-50);
        result.TempOffset.Should().Be(25);
        result.ChecksumOrFlags.Should().Be(0xAB);

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadCalibration), 0, 1), Times.Once);
    }

    [Fact]
    public void ReadCalibration_NegativeOffsets_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var calData = new CalibrationStruct
        {
            VoltageOffset = -1000,
            CurrentOffset = -500,
            TempOffset = -100,
            ChecksumOrFlags = 0
        };

        var responseBytes = StructToBytes(calData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadCalibration();

        // Assert
        result.VoltageOffset.Should().Be(-1000);
        result.CurrentOffset.Should().Be(-500);
        result.TempOffset.Should().Be(-100);
    }

    #endregion

    #region Command 0x0A: WriteCalibration

    [Fact]
    public void WriteCalibration_ValidData_WritesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var capturedBytes = new List<byte>();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                capturedBytes.AddRange(buffer.Skip(offset).Take(count));
            });

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);
        var cal = new CalibrationStruct
        {
            VoltageOffset = 50,
            CurrentOffset = -25,
            TempOffset = 10,
            ChecksumOrFlags = 0xCD
        };

        // Act
        protocol.WriteCalibration(cal);

        // Assert
        capturedBytes[0].Should().Be((byte)UartCommand.WriteCalibration);
        var writtenCal = MemoryMarshal.Read<CalibrationStruct>(capturedBytes.Skip(1).ToArray().AsSpan());
        writtenCal.VoltageOffset.Should().Be(50);
        writtenCal.CurrentOffset.Should().Be(-25);
    }

    #endregion

    #region Command 0x0B: LoadCalibration

    [Fact]
    public void LoadCalibration_SendsCorrectCommand()
    {
        // Arrange
        var mockPort = CreateMockPort();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()));

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        protocol.LoadCalibration();

        // Assert
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.LoadCalibration), 0, 1), Times.Once);
    }

    #endregion

    #region Command 0x0C: StoreCalibration

    [Fact]
    public void StoreCalibration_SendsCorrectCommand()
    {
        // Arrange
        var mockPort = CreateMockPort();
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()));

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        protocol.StoreCalibration();

        // Assert
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.StoreCalibration), 0, 1), Times.Once);
    }

    #endregion

    #region Command 0x0D: ReadDeviceUid

    [Fact]
    public void ReadDeviceUid_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var uidData = new DeviceUidStruct
        {
            Uid0 = 0x12345678,
            Uid1 = 0x9ABCDEF0,
            Uid2 = 0x11223344
        };

        var responseBytes = StructToBytes(uidData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceUid();

        // Assert
        result.Uid0.Should().Be(0x12345678);
        result.Uid1.Should().Be(0x9ABCDEF0);
        result.Uid2.Should().Be(0x11223344);

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadDeviceUid), 0, 1), Times.Once);
    }

    [Fact]
    public void ReadDeviceUid_ZeroUid_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var uidData = new DeviceUidStruct
        {
            Uid0 = 0,
            Uid1 = 0,
            Uid2 = 0
        };

        var responseBytes = StructToBytes(uidData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceUid();

        // Assert
        result.Uid0.Should().Be(0);
        result.Uid1.Should().Be(0);
        result.Uid2.Should().Be(0);
    }

    [Fact]
    public void ReadDeviceUid_MaxValues_HandlesCorrectly()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var uidData = new DeviceUidStruct
        {
            Uid0 = 0xFFFFFFFF,
            Uid1 = 0xFFFFFFFF,
            Uid2 = 0xFFFFFFFF
        };

        var responseBytes = StructToBytes(uidData);
        SetupReadResponse(mockPort, responseBytes);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadDeviceUid();

        // Assert
        result.Uid0.Should().Be(0xFFFFFFFF);
        result.Uid1.Should().Be(0xFFFFFFFF);
        result.Uid2.Should().Be(0xFFFFFFFF);
    }

    #endregion

    #region Command 0x0E: ReadVendorData

    [Fact]
    public void ReadVendorData_ValidData_ReturnsCorrectStruct()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var vendorData = new byte[] { 0xEE, 0x10, 0x02 }; // VID=0xEE, PID=0x10, FW=2
        SetupReadResponse(mockPort, vendorData);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadVendorData();

        // Assert
        result.VendorId.Should().Be(0xEE);
        result.ProductId.Should().Be(0x10);
        result.FwVersion.Should().Be(0x02);
        result.IsValid.Should().BeTrue();

        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadVendorData), 0, 1), Times.Once);
    }

    [Fact]
    public void ReadVendorData_InvalidVendorId_IsValidFalse()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var vendorData = new byte[] { 0xFF, 0x10, 0x01 }; // Wrong VID
        SetupReadResponse(mockPort, vendorData);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadVendorData();

        // Assert
        result.VendorId.Should().Be(0xFF);
        result.IsValid.Should().BeFalse();
    }

    [Fact]
    public void ReadVendorData_InvalidProductId_IsValidFalse()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var vendorData = new byte[] { 0xEE, 0xFF, 0x01 }; // Wrong PID
        SetupReadResponse(mockPort, vendorData);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act
        var result = protocol.ReadVendorData();

        // Assert
        result.ProductId.Should().Be(0xFF);
        result.IsValid.Should().BeFalse();
    }

    [Fact]
    public void ReadVendorData_DifferentFirmwareVersions_AllValid()
    {
        // Arrange & Act & Assert
        for (byte fwVersion = 0; fwVersion <= 10; fwVersion++)
        {
            var mockPort = CreateMockPort();
            var vendorData = new byte[] { 0xEE, 0x10, fwVersion };
            SetupReadResponse(mockPort, vendorData);

            var protocol = new BinaryProtocol(mockPort.Object, _timeout);
            var result = protocol.ReadVendorData();

            result.FwVersion.Should().Be(fwVersion);
            result.IsValid.Should().BeTrue();
        }
    }

    #endregion

    #region Error Scenarios

    [Fact]
    public void SendCommand_Timeout_ThrowsTimeoutException()
    {
        // Arrange
        var mockPort = CreateMockPort();
        mockPort.Setup(p => p.BytesToRead).Returns(0); // No data available
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Returns(0);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromMilliseconds(100));

        // Act & Assert
        var act = () => protocol.SendCommand(UartCommand.ReadVendorData, 3);
        act.Should().Throw<TimeoutException>()
            .WithMessage("Timeout waiting for response*");
    }

    [Fact]
    public void SendCommand_PartialRead_ThrowsIOException()
    {
        // Arrange
        var mockPort = CreateMockPort();
        var partialData = new byte[5]; // Expected more data
        SetupReadResponse(mockPort, partialData);

        var protocol = new BinaryProtocol(mockPort.Object, _timeout);

        // Act & Assert
        var act = () => protocol.SendCommand(UartCommand.ReadSensors, 100); // Expecting 100 bytes
        act.Should().Throw<IOException>()
            .WithMessage("Stream ended after reading*");
    }

    #endregion
}
