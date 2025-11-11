using Xunit;
using FluentAssertions;
using BenchLab.Platform.Protocol;

namespace BenchLab.Platform.Tests.Protocol;

public class StructTests
{
    [Fact]
    public void VendorDataStruct_IsValid_WithCorrectIds_ReturnsTrue()
    {
        // Arrange
        var vendorData = new VendorDataStruct
        {
            VendorId = 0xEE,
            ProductId = 0x10,
            FwVersion = 1
        };

        // Act & Assert
        vendorData.IsValid.Should().BeTrue();
    }

    [Fact]
    public void VendorDataStruct_IsValid_WithIncorrectIds_ReturnsFalse()
    {
        // Arrange
        var vendorData = new VendorDataStruct
        {
            VendorId = 0xFF,
            ProductId = 0x10,
            FwVersion = 1
        };

        // Act & Assert
        vendorData.IsValid.Should().BeFalse();
    }

    [Fact]
    public void PowerSensor_Conversions_CalculateCorrectly()
    {
        // Arrange
        var sensor = new PowerSensor
        {
            Voltage = 12000,  // 12000 mV = 12V
            Current = 5000,   // 5000 mA = 5A
            Power = 60000     // 60000 mW = 60W
        };

        // Act & Assert
        sensor.VoltageVolts.Should().Be(12.0);
        sensor.CurrentAmps.Should().Be(5.0);
        sensor.PowerWatts.Should().Be(60.0);
    }

    [Fact]
    public void FanSensor_IsEnabled_WithZero_ReturnsFalse()
    {
        // Arrange
        var fan = new FanSensor { Enable = 0 };

        // Act & Assert
        fan.IsEnabled.Should().BeFalse();
    }

    [Fact]
    public void FanSensor_IsEnabled_WithNonZero_ReturnsTrue()
    {
        // Arrange
        var fan = new FanSensor { Enable = 1 };

        // Act & Assert
        fan.IsEnabled.Should().BeTrue();
    }

    [Fact]
    public void FanSensor_DutyPercent_CalculatesCorrectly()
    {
        // Arrange
        var fan = new FanSensor { Duty = 127 }; // ~50%

        // Act
        var percent = fan.DutyPercent;

        // Assert
        percent.Should().BeApproximately(49.8, 0.5);
    }

    [Fact]
    public unsafe void SensorStruct_GetTemperatures_ReturnsCorrectValues()
    {
        // Arrange
        var sensors = new SensorStruct
        {
            Tchip = 4500,  // 45.00°C
            Tamb = 2300    // 23.00°C
        };

        // Act
        var chipTemp = sensors.ChipTemperature;
        var ambientTemp = sensors.AmbientTemperature;

        // Assert
        chipTemp.Should().Be(45.0);
        ambientTemp.Should().Be(23.0);
    }

    [Fact]
    public unsafe void SensorStruct_HumidityPercent_CalculatesCorrectly()
    {
        // Arrange
        var sensors = new SensorStruct
        {
            Hum = 5500  // 55.00%
        };

        // Act
        var humidity = sensors.HumidityPercent;

        // Assert
        humidity.Should().Be(55.0);
    }
}
