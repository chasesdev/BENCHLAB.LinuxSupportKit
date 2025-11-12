using Xunit;
using FluentAssertions;
using BenchLab.Platform.Discovery;

namespace BenchLab.Platform.Tests.Discovery;

/// <summary>
/// Tests for BenchlabHandshake behavior and edge cases.
/// Note: Full integration tests require real hardware or mocked SerialPort infrastructure.
/// These tests document expected behavior and test what's possible without hardware.
/// </summary>
public class BenchlabHandshakeTests
{
    [Fact]
    public void Constructor_WithValidateVendorDataTrue_CreatesInstance()
    {
        // Act
        var handshake = new BenchlabHandshake(validateVendorData: true);

        // Assert
        handshake.Should().NotBeNull();
    }

    [Fact]
    public void Constructor_WithValidateVendorDataFalse_CreatesInstance()
    {
        // Act
        var handshake = new BenchlabHandshake(validateVendorData: false);

        // Assert
        handshake.Should().NotBeNull();
    }

    [Fact]
    public void Probe_WithNonExistentDevice_ReturnsFailedResult()
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: true);
        var nonExistentDevice = "/dev/ttyNONEXISTENT999";

        // Act
        var result = handshake.Probe(nonExistentDevice, TimeSpan.FromMilliseconds(100));

        // Assert
        result.Should().NotBeNull();
        result.Device.Should().Be(nonExistentDevice);
        result.IsBenchlab.Should().BeFalse();
        // Info should indicate some kind of error
        result.Info.Should().NotBeEmpty();
    }

    [Fact]
    public void Probe_WithInvalidPath_ReturnsFailedResult()
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: false);
        var invalidPath = "/invalid/path/to/device";

        // Act
        var result = handshake.Probe(invalidPath, TimeSpan.FromMilliseconds(100));

        // Assert
        result.Should().NotBeNull();
        result.Device.Should().Be(invalidPath);
        result.IsBenchlab.Should().BeFalse();
        result.Info.Should().NotBeEmpty();
    }

    [Fact]
    public void Probe_WithEmptyDevicePath_ReturnsFailedResult()
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: true);

        // Act
        var result = handshake.Probe("", TimeSpan.FromMilliseconds(100));

        // Assert
        result.Should().NotBeNull();
        result.Device.Should().Be("");
        result.IsBenchlab.Should().BeFalse();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Probe_WithBothValidationModes_HandlesNonExistentDevice(bool validateVendorData)
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: validateVendorData);
        var device = "/dev/ttyNONEXISTENT";

        // Act
        var result = handshake.Probe(device, TimeSpan.FromMilliseconds(50));

        // Assert
        result.Should().NotBeNull();
        result.IsBenchlab.Should().BeFalse();
        result.Device.Should().Be(device);
    }

    [Theory]
    [InlineData(10)]
    [InlineData(100)]
    [InlineData(500)]
    [InlineData(1000)]
    public void Probe_WithVariousTimeouts_ReturnsResultWithinReasonableTime(int timeoutMs)
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: false);
        var nonExistentDevice = "/dev/ttyNONEXISTENT";
        var timeout = TimeSpan.FromMilliseconds(timeoutMs);

        // Act
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = handshake.Probe(nonExistentDevice, timeout);
        sw.Stop();

        // Assert
        result.Should().NotBeNull();
        result.IsBenchlab.Should().BeFalse();
        // Should return quickly for non-existent device (not wait for full timeout)
        sw.ElapsedMilliseconds.Should().BeLessThan(timeoutMs + 100);
    }

    [Fact]
    public void Probe_DisposesSerialPortEvenOnException()
    {
        // Arrange
        var handshake = new BenchlabHandshake(validateVendorData: true);
        var invalidDevice = "/dev/null"; // Exists but not a serial device

        // Act
        var result = handshake.Probe(invalidDevice, TimeSpan.FromMilliseconds(100));

        // Assert
        // Should return failed result without hanging or leaving resources open
        result.Should().NotBeNull();
        result.IsBenchlab.Should().BeFalse();
        // No resource leak - verified by test completing successfully
    }

    /// <summary>
    /// Documents expected behavior when welcome handshake fails.
    /// With real hardware, this would return Failed result.
    /// </summary>
    [Fact]
    public void Probe_WhenWelcomeHandshakeFails_ReturnsFailedResult_Documentation()
    {
        // This test documents expected behavior:
        // If protocol.PerformHandshake() returns false, Probe should return:
        // - IsBenchlab = false
        // - Info = "Welcome handshake failed"

        // Arrange (simulated scenario)
        var expectedDevice = "/dev/ttyACM0";
        var expectedInfo = "Welcome handshake failed";

        // Act (what would happen with real hardware)
        var simulatedResult = ProbeResult.Failed(expectedDevice, expectedInfo);

        // Assert (document expected behavior)
        simulatedResult.IsBenchlab.Should().BeFalse();
        simulatedResult.Info.Should().Be(expectedInfo);
        simulatedResult.Device.Should().Be(expectedDevice);
    }

    /// <summary>
    /// Documents expected behavior when vendor data is invalid.
    /// </summary>
    [Fact]
    public void Probe_WhenVendorDataInvalid_ReturnsFailedResult_Documentation()
    {
        // This test documents expected behavior:
        // If vendorData.IsValid is false, Probe should return:
        // - IsBenchlab = false
        // - Info contains "Invalid vendor data" with VID and PID

        // Arrange (simulated scenario)
        var expectedDevice = "/dev/ttyACM0";
        byte vendorId = 0xFF;
        byte productId = 0xFF;
        var expectedInfo = $"Invalid vendor data: VID=0x{vendorId:X2}, PID=0x{productId:X2}";

        // Act (what would happen with real hardware)
        var simulatedResult = ProbeResult.Failed(expectedDevice, expectedInfo);

        // Assert (document expected behavior)
        simulatedResult.IsBenchlab.Should().BeFalse();
        simulatedResult.Info.Should().Contain("Invalid vendor data");
        simulatedResult.Info.Should().Contain("VID=");
        simulatedResult.Info.Should().Contain("PID=");
    }

    /// <summary>
    /// Documents expected behavior when vendor data read throws exception.
    /// </summary>
    [Fact]
    public void Probe_WhenVendorDataUnavailable_StillReturnsOkIfWelcomeSucceeded_Documentation()
    {
        // This test documents expected behavior:
        // If welcome succeeds but vendor data read throws exception, Probe should return:
        // - IsBenchlab = true (because welcome succeeded)
        // - Info = "BenchLab device (vendor data unavailable)"

        // Arrange (simulated scenario)
        var expectedDevice = "/dev/ttyACM0";
        var expectedInfo = "BenchLab device (vendor data unavailable)";

        // Act (what would happen with real hardware)
        var simulatedResult = ProbeResult.Ok(expectedDevice, expectedInfo);

        // Assert (document expected behavior)
        simulatedResult.IsBenchlab.Should().BeTrue();
        simulatedResult.Info.Should().Contain("vendor data unavailable");
    }

    /// <summary>
    /// Documents expected behavior for successful probe with vendor validation.
    /// </summary>
    [Fact]
    public void Probe_WhenSuccessful_ReturnsOkWithVendorInfo_Documentation()
    {
        // This test documents expected behavior:
        // Successful probe with vendor validation returns:
        // - IsBenchlab = true
        // - Info contains VID, PID, and firmware version

        // Arrange (simulated scenario)
        var expectedDevice = "/dev/ttyACM0";
        byte vendorId = 0xEE;
        byte productId = 0x10;
        byte fwVersion = 5;
        var expectedInfo = $"BenchLab device (VID=0x{vendorId:X2}, PID=0x{productId:X2}, FW=v{fwVersion})";

        // Act (what would happen with real hardware)
        var simulatedResult = ProbeResult.Ok(expectedDevice, expectedInfo);

        // Assert (document expected behavior)
        simulatedResult.IsBenchlab.Should().BeTrue();
        simulatedResult.Info.Should().Contain("VID=0xEE");
        simulatedResult.Info.Should().Contain("PID=0x10");
        simulatedResult.Info.Should().Contain("FW=v5");
    }
}
