using Xunit;
using FluentAssertions;
using Moq;
using BenchLab.Platform.Protocol;
using BenchLab.Platform.Ports;

namespace BenchLab.Platform.Tests.Protocol;

public class BinaryProtocolTests
{
    [Fact]
    public void PerformHandshake_WithValidResponse_ReturnsTrue()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var welcomeResponse = new byte[13];
        System.Text.Encoding.ASCII.GetBytes("BENCHLAB").CopyTo(welcomeResponse, 0);

        mockPort.Setup(p => p.BytesToRead).Returns(13);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(welcomeResponse, 0, buffer, offset, Math.Min(count, welcomeResponse.Length));
            })
            .Returns(welcomeResponse.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var result = protocol.PerformHandshake();

        // Assert
        result.Should().BeTrue();
        mockPort.Verify(p => p.Write(It.Is<byte[]>(b => b[0] == 0x00), 0, 1), Times.Once);
    }

    [Fact]
    public void PerformHandshake_WithInvalidResponse_ReturnsFalse()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var invalidResponse = new byte[13];
        System.Text.Encoding.ASCII.GetBytes("INVALID").CopyTo(invalidResponse, 0);

        mockPort.Setup(p => p.BytesToRead).Returns(13);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(invalidResponse, 0, buffer, offset, Math.Min(count, invalidResponse.Length));
            })
            .Returns(invalidResponse.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var result = protocol.PerformHandshake();

        // Assert
        result.Should().BeFalse();
    }

    [Fact]
    public void ReadVendorData_WithValidDevice_ReturnsCorrectData()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var vendorData = new byte[] { 0xEE, 0x10, 0x01 }; // VID, PID, FW version

        mockPort.Setup(p => p.BytesToRead).Returns(3);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(vendorData, 0, buffer, offset, Math.Min(count, vendorData.Length));
            })
            .Returns(vendorData.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var result = protocol.ReadVendorData();

        // Assert
        result.VendorId.Should().Be(0xEE);
        result.ProductId.Should().Be(0x10);
        result.FwVersion.Should().Be(0x01);
        result.IsValid.Should().BeTrue();
    }

    [Fact]
    public void SendCommand_WithTimeout_ThrowsTimeoutException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        mockPort.Setup(p => p.BytesToRead).Returns(0); // No data available

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromMilliseconds(100));

        // Act & Assert
        var act = () => protocol.SendCommand(UartCommand.Welcome, 13);
        act.Should().Throw<TimeoutException>();
    }

    [Fact]
    public void SendCommand_WithPartialResponse_ThrowsTimeoutException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var partialData = new byte[5]; // Only 5 bytes, but expecting 13

        var callCount = 0;
        mockPort.Setup(p => p.BytesToRead)
            .Returns(() => callCount++ == 0 ? 5 : 0);

        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(partialData, 0, buffer, offset, Math.Min(count, partialData.Length));
            })
            .Returns(5);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromMilliseconds(100));

        // Act & Assert
        var act = () => protocol.SendCommand(UartCommand.Welcome, 13);
        act.Should().Throw<TimeoutException>()
            .WithMessage("*received 5 bytes*");
    }

    [Fact]
    public void SendCommand_DiscardsInputBufferBeforeSending()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[13];
        System.Text.Encoding.ASCII.GetBytes("BENCHLAB").CopyTo(response, 0);

        mockPort.Setup(p => p.BytesToRead).Returns(13);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        protocol.SendCommand(UartCommand.Welcome, 13);

        // Assert
        mockPort.Verify(p => p.DiscardInBuffer(), Times.Once);
    }

    [Fact]
    public void SendCommand_WritesCommandByteFirst()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[3] { 0xEE, 0x10, 0x01 };
        byte[] writtenData = null!;

        mockPort.Setup(p => p.BytesToRead).Returns(3);
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                writtenData = new byte[count];
                Array.Copy(buffer, offset, writtenData, 0, count);
            });

        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        protocol.SendCommand(UartCommand.ReadVendorData, 3);

        // Assert
        writtenData.Should().NotBeNull();
        writtenData[0].Should().Be((byte)UartCommand.ReadVendorData);
    }

    [Fact]
    public void SendCommand_WithAdditionalData_AppendsAfterCommand()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 };
        byte[] writtenData = null!;

        mockPort.Setup(p => p.BytesToRead).Returns(1);
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                writtenData = new byte[count];
                Array.Copy(buffer, offset, writtenData, 0, count);
            });

        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));
        var additionalData = new byte[] { 0xAA, 0xBB, 0xCC };

        // Act
        protocol.SendCommand(UartCommand.Action, 1, additionalData);

        // Assert
        writtenData.Should().NotBeNull();
        writtenData.Should().HaveCount(4); // 1 command + 3 additional
        writtenData[0].Should().Be((byte)UartCommand.Action);
        writtenData[1].Should().Be(0xAA);
        writtenData[2].Should().Be(0xBB);
        writtenData[3].Should().Be(0xCC);
    }

    [Fact]
    public async Task SendCommandAsync_WithCancellation_ThrowsOperationCanceledException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var cts = new CancellationTokenSource();
        cts.Cancel(); // Cancel immediately

        mockPort.Setup(p => p.WriteAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
            .ThrowsAsync(new OperationCanceledException());

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act & Assert
        await FluentActions.Awaiting(async () =>
                await protocol.SendCommandAsync(UartCommand.Welcome, 13, null, cts.Token))
            .Should().ThrowAsync<OperationCanceledException>();
    }

    [Fact]
    public async Task ReadSensorsAsync_ReturnsValidSensorStruct()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();

        unsafe
        {
            var expectedSize = sizeof(SensorStruct);
            var sensorData = new byte[expectedSize];

            // Fill with test data (simplified - real struct would have proper layout)
            Array.Fill<byte>(sensorData, 0x42);

            mockPort.Setup(p => p.WriteAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
                .Returns(Task.CompletedTask);

            var readCount = 0;
            mockPort.Setup(p => p.ReadAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
                .Callback<byte[], int, int, CancellationToken>((buffer, offset, count, _) =>
                {
                    var toCopy = Math.Min(count, sensorData.Length - readCount);
                    Array.Copy(sensorData, readCount, buffer, offset, toCopy);
                    readCount += toCopy;
                })
                .ReturnsAsync((byte[] buffer, int offset, int count, CancellationToken _) =>
                {
                    return Math.Min(count, sensorData.Length);
                });

            var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

            // Act
            var result = await protocol.ReadSensorsAsync();

            // Assert
            result.Should().NotBeNull();
            mockPort.Verify(p => p.WriteAsync(
                It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadSensors),
                0,
                1,
                It.IsAny<CancellationToken>()
            ), Times.Once);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(8)]
    public void ReadFanProfile_WithValidIndex_SendsCorrectCommand(byte fanIndex)
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();

        unsafe
        {
            var expectedSize = sizeof(FanProfileStruct);
            var response = new byte[expectedSize];
            byte[] writtenData = null!;

            mockPort.Setup(p => p.BytesToRead).Returns(expectedSize);
            mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
                .Callback<byte[], int, int>((buffer, offset, count) =>
                {
                    writtenData = new byte[count];
                    Array.Copy(buffer, offset, writtenData, 0, count);
                });

            mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
                .Callback<byte[], int, int>((buffer, offset, count) =>
                {
                    Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
                })
                .Returns(response.Length);

            var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

            // Act
            protocol.ReadFanProfile(fanIndex);

            // Assert
            writtenData.Should().NotBeNull();
            writtenData.Should().HaveCount(2);
            writtenData[0].Should().Be((byte)UartCommand.ReadFanProfile);
            writtenData[1].Should().Be(fanIndex);
        }
    }

    [Fact]
    public void ReadFanProfile_WithInvalidIndex_ThrowsArgumentException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act & Assert
        var act = () => protocol.ReadFanProfile(9); // Max is 8
        act.Should().Throw<ArgumentException>()
            .WithMessage("*Fan index must be 0-8*");
    }

    [Fact]
    public void WriteDeviceName_WithTooLongName_ThrowsArgumentException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));
        var tooLongName = new string('X', 33); // Max is 32

        // Act & Assert
        var act = () => protocol.WriteDeviceName(tooLongName);
        act.Should().Throw<ArgumentException>()
            .WithMessage("*32 characters or less*");
    }

    [Fact]
    public void LoadCalibration_ReturnsStatusByte()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 }; // Success status

        mockPort.Setup(p => p.BytesToRead).Returns(1);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var status = protocol.LoadCalibration();

        // Assert
        status.Should().Be(0x00);
        mockPort.Verify(p => p.Write(
            It.Is<byte[]>(b => b[0] == (byte)UartCommand.LoadCalibration),
            0,
            1
        ), Times.Once);
    }

    [Fact]
    public void LoadCalibration_WithNoCalibrationStored_ReturnsStatusOne()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x01 }; // No calibration stored

        mockPort.Setup(p => p.BytesToRead).Returns(1);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var status = protocol.LoadCalibration();

        // Assert
        status.Should().Be(0x01);
    }

    [Fact]
    public void StoreCalibration_ReturnsStatusByte()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 }; // Success

        mockPort.Setup(p => p.BytesToRead).Returns(1);
        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var status = protocol.StoreCalibration();

        // Assert
        status.Should().Be(0x00);
        mockPort.Verify(p => p.Write(
            It.Is<byte[]>(b => b[0] == (byte)UartCommand.StoreCalibration),
            0,
            1
        ), Times.Once);
    }

    [Fact]
    public void ReadCalibration_ReturnsCalibrationStruct()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();

        unsafe
        {
            var expectedSize = sizeof(CalibrationStruct);
            var calibrationData = new byte[expectedSize];
            Array.Fill<byte>(calibrationData, 0xFF);

            mockPort.Setup(p => p.BytesToRead).Returns(expectedSize);
            mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
                .Callback<byte[], int, int>((buffer, offset, count) =>
                {
                    Array.Copy(calibrationData, 0, buffer, offset, Math.Min(count, calibrationData.Length));
                })
                .Returns(calibrationData.Length);

            var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

            // Act
            var result = protocol.ReadCalibration();

            // Assert
            result.Should().NotBeNull();
            mockPort.Verify(p => p.Write(
                It.Is<byte[]>(b => b[0] == (byte)UartCommand.ReadCalibration),
                0,
                1
            ), Times.Once);
        }
    }

    [Fact]
    public void WriteCalibration_SendsCalibrationData_ReturnsStatus()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 }; // Success
        byte[] writtenData = null!;

        unsafe
        {
            var expectedSize = sizeof(CalibrationStruct);

            mockPort.Setup(p => p.BytesToRead).Returns(1);
            mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
                .Callback<byte[], int, int>((buffer, offset, count) =>
                {
                    writtenData = new byte[count];
                    Array.Copy(buffer, offset, writtenData, 0, count);
                });

            mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
                .Callback<byte[], int, int>((buffer, offset, count) =>
                {
                    Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
                })
                .Returns(response.Length);

            var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));
            var calibration = new CalibrationStruct();

            // Act
            var status = protocol.WriteCalibration(calibration);

            // Assert
            status.Should().Be(0x00);
            writtenData.Should().NotBeNull();
            writtenData[0].Should().Be((byte)UartCommand.WriteCalibration);
            writtenData.Length.Should().Be(1 + expectedSize); // Command + calibration data
        }
    }

    [Fact]
    public async Task LoadCalibrationAsync_ReturnsStatusByte()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 };

        mockPort.Setup(p => p.WriteAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
            .Returns(Task.CompletedTask);

        mockPort.Setup(p => p.ReadAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
            .Callback<byte[], int, int, CancellationToken>((buffer, offset, count, _) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .ReturnsAsync(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var status = await protocol.LoadCalibrationAsync();

        // Assert
        status.Should().Be(0x00);
    }

    [Fact]
    public async Task StoreCalibrationAsync_ReturnsStatusByte()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 };

        mockPort.Setup(p => p.WriteAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
            .Returns(Task.CompletedTask);

        mockPort.Setup(p => p.ReadAsync(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<CancellationToken>()))
            .Callback<byte[], int, int, CancellationToken>((buffer, offset, count, _) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .ReturnsAsync(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));

        // Act
        var status = await protocol.StoreCalibrationAsync();

        // Assert
        status.Should().Be(0x00);
    }

    [Fact]
    public void WriteFanProfile_WithInvalidIndex_ThrowsArgumentException()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));
        var profile = new FanProfileStruct();

        // Act & Assert
        var act = () => protocol.WriteFanProfile(9, profile); // Max is 8
        act.Should().Throw<ArgumentException>()
            .WithMessage("*Fan index must be 0-8*");
    }

    [Fact]
    public void ExecuteAction_SendsActionIdAndReturnsStatus()
    {
        // Arrange
        var mockPort = new Mock<ISerialPort>();
        var response = new byte[1] { 0x00 }; // Success
        byte[] writtenData = null!;

        mockPort.Setup(p => p.BytesToRead).Returns(1);
        mockPort.Setup(p => p.Write(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                writtenData = new byte[count];
                Array.Copy(buffer, offset, writtenData, 0, count);
            });

        mockPort.Setup(p => p.Read(It.IsAny<byte[]>(), It.IsAny<int>(), It.IsAny<int>()))
            .Callback<byte[], int, int>((buffer, offset, count) =>
            {
                Array.Copy(response, 0, buffer, offset, Math.Min(count, response.Length));
            })
            .Returns(response.Length);

        var protocol = new BinaryProtocol(mockPort.Object, TimeSpan.FromSeconds(1));
        byte actionId = 0x42;

        // Act
        var status = protocol.ExecuteAction(actionId);

        // Assert
        status.Should().Be(0x00);
        writtenData.Should().NotBeNull();
        writtenData.Should().HaveCount(2);
        writtenData[0].Should().Be((byte)UartCommand.Action);
        writtenData[1].Should().Be(actionId);
    }
}
