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
}
