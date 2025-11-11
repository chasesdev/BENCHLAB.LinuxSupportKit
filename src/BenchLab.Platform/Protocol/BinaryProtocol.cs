using System.Buffers;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using BenchLab.Platform.Ports;
using Microsoft.Extensions.Logging;

namespace BenchLab.Platform.Protocol;

/// <summary>
/// Handles BenchLab binary protocol communication over serial port.
/// Provides both synchronous and asynchronous (event-driven) methods.
/// </summary>
public class BinaryProtocol
{
    private readonly ISerialPort _port;
    private readonly ILogger? _log;
    private readonly TimeSpan _commandTimeout;

    /// <summary>
    /// Creates a new binary protocol handler.
    /// </summary>
    /// <param name="port">Serial port interface.</param>
    /// <param name="commandTimeout">Timeout for command responses.</param>
    /// <param name="log">Optional logger.</param>
    public BinaryProtocol(ISerialPort port, TimeSpan? commandTimeout = null, ILogger? log = null)
    {
        _port = port ?? throw new ArgumentNullException(nameof(port));
        _commandTimeout = commandTimeout ?? TimeSpan.FromMilliseconds(500);
        _log = log;
    }

    /// <summary>
    /// Sends a command and reads the expected response.
    /// </summary>
    /// <param name="command">Command byte to send.</param>
    /// <param name="expectedResponseLength">Expected length of the response in bytes.</param>
    /// <param name="additionalData">Optional additional data to send after the command byte.</param>
    /// <returns>Response bytes.</returns>
    /// <exception cref="TimeoutException">If response is not received within timeout.</exception>
    /// <exception cref="IOException">If communication fails.</exception>
    public byte[] SendCommand(UartCommand command, int expectedResponseLength, byte[]? additionalData = null)
    {
        byte[]? commandPacket = null;
        try
        {
            // Clear any pending data in the receive buffer
            _port.DiscardInBuffer();

            // Build command packet using pooled buffer
            var packetLength = 1 + (additionalData?.Length ?? 0);
            commandPacket = ArrayPool<byte>.Shared.Rent(packetLength);
            commandPacket[0] = (byte)command;
            if (additionalData != null && additionalData.Length > 0)
            {
                Array.Copy(additionalData, 0, commandPacket, 1, additionalData.Length);
            }

            // Send command
            _port.Write(commandPacket, 0, packetLength);
            _log?.LogDebug("Sent command {Command} ({Bytes} bytes)", command, packetLength);

            // Read response with timeout
            var response = ReadExactly(expectedResponseLength, _commandTimeout);
            _log?.LogDebug("Received response ({Bytes} bytes) for command {Command}", response.Length, command);

            return response;
        }
        catch (TimeoutException ex)
        {
            _log?.LogError(ex, "Timeout waiting for response to command {Command}", command);
            throw;
        }
        catch (Exception ex)
        {
            _log?.LogError(ex, "Error executing command {Command}", command);
            throw;
        }
        finally
        {
            if (commandPacket != null)
                ArrayPool<byte>.Shared.Return(commandPacket);
        }
    }

    /// <summary>
    /// Reads exactly the specified number of bytes with timeout.
    /// DEPRECATED: Prefer async methods. This sync version is maintained for compatibility but uses polling.
    /// </summary>
    private byte[] ReadExactly(int count, TimeSpan timeout)
    {
        var buffer = new byte[count];
        var totalRead = 0;
        var deadline = DateTime.UtcNow + timeout;

        while (totalRead < count)
        {
            if (DateTime.UtcNow > deadline)
            {
                throw new TimeoutException($"Timeout reading {count} bytes (received {totalRead} bytes)");
            }

            var available = _port.BytesToRead;
            if (available > 0)
            {
                var toRead = Math.Min(available, count - totalRead);
                var bytesRead = _port.Read(buffer, totalRead, toRead);
                totalRead += bytesRead;
            }
            else
            {
                // Minimal sleep to reduce CPU usage (polling is unavoidable in sync API)
                // For better performance, use async methods instead
                Thread.Sleep(5); // Reduced from 10ms to 5ms
            }
        }

        return buffer;
    }

    /// <summary>
    /// Performs device identification handshake.
    /// Sends UART_CMD_WELCOME and validates the response.
    /// </summary>
    /// <returns>True if device is a valid BenchLab device, false otherwise.</returns>
    public bool PerformHandshake()
    {
        try
        {
            // Send UART_CMD_WELCOME (0x00)
            var response = SendCommand(UartCommand.Welcome, ProtocolConstants.WelcomeResponseLength);

            // Validate response: should be "BENCHLAB\x00" (13 bytes total, null-terminated)
            var responseStr = Encoding.ASCII.GetString(response).TrimEnd('\0');
            var isValid = responseStr == ProtocolConstants.WelcomeResponse;

            _log?.LogInformation("Handshake result: {Result} (received: '{Response}')",
                isValid ? "SUCCESS" : "FAILED", responseStr);

            return isValid;
        }
        catch (Exception ex)
        {
            _log?.LogWarning(ex, "Handshake failed with exception");
            return false;
        }
    }

    /// <summary>
    /// Reads vendor data to verify device identity.
    /// </summary>
    /// <returns>Vendor data structure.</returns>
    public VendorDataStruct ReadVendorData()
    {
        var response = SendCommand(UartCommand.ReadVendorData, ProtocolConstants.VendorDataLength);
        return BytesToStruct<VendorDataStruct>(response);
    }

    /// <summary>
    /// Reads complete sensor telemetry.
    /// </summary>
    /// <returns>Sensor data structure.</returns>
    public unsafe SensorStruct ReadSensors()
    {
        var expectedSize = sizeof(SensorStruct);
        var response = SendCommand(UartCommand.ReadSensors, expectedSize);
        return BytesToStruct<SensorStruct>(response);
    }

    /// <summary>
    /// Reads device name.
    /// </summary>
    /// <returns>Device name string.</returns>
    public string ReadDeviceName()
    {
        var response = SendCommand(UartCommand.ReadName, ProtocolConstants.DeviceNameLength);
        return Encoding.ASCII.GetString(response).TrimEnd('\0');
    }

    /// <summary>
    /// Writes device name.
    /// </summary>
    /// <param name="name">New device name (max 32 characters).</param>
    /// <exception cref="ArgumentException">If name is too long.</exception>
    public void WriteDeviceName(string name)
    {
        if (name.Length > ProtocolConstants.DeviceNameLength)
        {
            throw new ArgumentException($"Device name must be {ProtocolConstants.DeviceNameLength} characters or less", nameof(name));
        }

        var nameBytes = new byte[ProtocolConstants.DeviceNameLength];
        var encoded = Encoding.ASCII.GetBytes(name);
        Array.Copy(encoded, nameBytes, Math.Min(encoded.Length, ProtocolConstants.DeviceNameLength));

        // Send WriteName command with name data
        SendCommand(UartCommand.WriteName, 1, nameBytes); // Expect 1 byte status response
    }

    /// <summary>
    /// Executes a device action command.
    /// </summary>
    /// <param name="actionId">Action identifier.</param>
    /// <returns>Status byte (0=success, non-zero=error code).</returns>
    public byte ExecuteAction(byte actionId)
    {
        var response = SendCommand(UartCommand.Action, 1, new[] { actionId });
        return response[0];
    }

    /// <summary>
    /// Reads device unique identifier.
    /// </summary>
    /// <returns>Device UID structure.</returns>
    public DeviceUidStruct ReadDeviceUid()
    {
        unsafe
        {
            var expectedSize = sizeof(DeviceUidStruct);
            var response = SendCommand(UartCommand.ReadUid, expectedSize);
            return BytesToStruct<DeviceUidStruct>(response);
        }
    }

    /// <summary>
    /// Reads fan profile configuration for a specific fan.
    /// </summary>
    /// <param name="fanIndex">Fan index (0-8).</param>
    /// <returns>Fan profile structure.</returns>
    public unsafe FanProfileStruct ReadFanProfile(byte fanIndex)
    {
        if (fanIndex >= ProtocolConstants.FanCount)
        {
            throw new ArgumentException($"Fan index must be 0-{ProtocolConstants.FanCount - 1}", nameof(fanIndex));
        }

        var expectedSize = sizeof(FanProfileStruct);
        var response = SendCommand(UartCommand.ReadFanProfile, expectedSize, new[] { fanIndex });
        return BytesToStruct<FanProfileStruct>(response);
    }

    /// <summary>
    /// Writes fan profile configuration for a specific fan.
    /// </summary>
    /// <param name="fanIndex">Fan index (0-8).</param>
    /// <param name="profile">Fan profile configuration.</param>
    /// <returns>Status byte (0=success).</returns>
    public byte WriteFanProfile(byte fanIndex, FanProfileStruct profile)
    {
        if (fanIndex >= ProtocolConstants.FanCount)
        {
            throw new ArgumentException($"Fan index must be 0-{ProtocolConstants.FanCount - 1}", nameof(fanIndex));
        }

        var profileBytes = StructToBytes(profile);
        var data = new byte[1 + profileBytes.Length];
        data[0] = fanIndex;
        Array.Copy(profileBytes, 0, data, 1, profileBytes.Length);

        var response = SendCommand(UartCommand.WriteFanProfile, 1, data);
        return response[0];
    }

    /// <summary>
    /// Reads RGB LED configuration.
    /// </summary>
    /// <returns>RGB configuration structure.</returns>
    public unsafe RgbStruct ReadRgb()
    {
        var expectedSize = sizeof(RgbStruct);
        var response = SendCommand(UartCommand.ReadRgb, expectedSize);
        return BytesToStruct<RgbStruct>(response);
    }

    /// <summary>
    /// Writes RGB LED configuration.
    /// </summary>
    /// <param name="rgb">RGB configuration.</param>
    /// <returns>Status byte (0=success).</returns>
    public byte WriteRgb(RgbStruct rgb)
    {
        var rgbBytes = StructToBytes(rgb);
        var response = SendCommand(UartCommand.WriteRgb, 1, rgbBytes);
        return response[0];
    }

    /// <summary>
    /// Reads calibration data from device RAM.
    /// </summary>
    /// <returns>Calibration structure.</returns>
    public unsafe CalibrationStruct ReadCalibration()
    {
        var expectedSize = sizeof(CalibrationStruct);
        var response = SendCommand(UartCommand.ReadCalibration, expectedSize);
        return BytesToStruct<CalibrationStruct>(response);
    }

    /// <summary>
    /// Writes calibration data to device RAM (not persisted until StoreCalibration).
    /// </summary>
    /// <param name="calibration">Calibration data.</param>
    /// <returns>Status byte (0=success).</returns>
    public byte WriteCalibration(CalibrationStruct calibration)
    {
        var calBytes = StructToBytes(calibration);
        var response = SendCommand(UartCommand.WriteCalibration, 1, calBytes);
        return response[0];
    }

    /// <summary>
    /// Loads calibration data from flash memory to RAM.
    /// </summary>
    /// <returns>Status byte (0=success, 1=no calibration stored).</returns>
    public byte LoadCalibration()
    {
        var response = SendCommand(UartCommand.LoadCalibration, 1);
        return response[0];
    }

    /// <summary>
    /// Stores current calibration data from RAM to flash memory.
    /// </summary>
    /// <returns>Status byte (0=success, non-zero=flash write error).</returns>
    public byte StoreCalibration()
    {
        var response = SendCommand(UartCommand.StoreCalibration, 1);
        return response[0];
    }

    #region Async (Event-Driven) Methods

    /// <summary>
    /// Sends a command and reads the expected response asynchronously (event-driven, no polling).
    /// </summary>
    /// <param name="command">Command byte to send.</param>
    /// <param name="expectedResponseLength">Expected length of the response in bytes.</param>
    /// <param name="additionalData">Optional additional data to send after the command byte.</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Response bytes.</returns>
    /// <exception cref="TimeoutException">If response is not received within timeout.</exception>
    /// <exception cref="IOException">If communication fails.</exception>
    public async Task<byte[]> SendCommandAsync(UartCommand command, int expectedResponseLength, byte[]? additionalData = null, CancellationToken cancellationToken = default)
    {
        try
        {
            // Clear any pending data in the receive buffer
            _port.DiscardInBuffer();

            // Build command packet
            var commandPacket = new byte[1 + (additionalData?.Length ?? 0)];
            commandPacket[0] = (byte)command;
            if (additionalData != null && additionalData.Length > 0)
            {
                Array.Copy(additionalData, 0, commandPacket, 1, additionalData.Length);
            }

            // Send command
            await _port.WriteAsync(commandPacket, 0, commandPacket.Length, cancellationToken);
            _log?.LogDebug("Sent command {Command} ({Bytes} bytes)", command, commandPacket.Length);

            // Read response with timeout (event-driven, no polling)
            var response = await ReadExactlyAsync(expectedResponseLength, _commandTimeout, cancellationToken);
            _log?.LogDebug("Received response ({Bytes} bytes) for command {Command}", response.Length, command);

            return response;
        }
        catch (TimeoutException ex)
        {
            _log?.LogError(ex, "Timeout waiting for response to command {Command}", command);
            throw;
        }
        catch (Exception ex)
        {
            _log?.LogError(ex, "Error executing command {Command}", command);
            throw;
        }
    }

    /// <summary>
    /// Reads exactly the specified number of bytes asynchronously with timeout (event-driven, no polling).
    /// </summary>
    private async Task<byte[]> ReadExactlyAsync(int count, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var buffer = new byte[count];
        var totalRead = 0;

        using var timeoutCts = new CancellationTokenSource(timeout);
        using var linkedCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeoutCts.Token);

        try
        {
            while (totalRead < count)
            {
                var bytesRead = await _port.ReadAsync(buffer, totalRead, count - totalRead, linkedCts.Token);
                if (bytesRead == 0)
                {
                    // Stream ended unexpectedly
                    throw new IOException($"Stream ended after reading {totalRead} of {count} bytes");
                }
                totalRead += bytesRead;
            }

            return buffer;
        }
        catch (OperationCanceledException) when (timeoutCts.IsCancellationRequested && !cancellationToken.IsCancellationRequested)
        {
            throw new TimeoutException($"Timeout reading {count} bytes (received {totalRead} bytes)");
        }
    }

    /// <summary>
    /// Performs device identification handshake asynchronously.
    /// </summary>
    public async Task<bool> PerformHandshakeAsync(CancellationToken cancellationToken = default)
    {
        try
        {
            var response = await SendCommandAsync(UartCommand.Welcome, ProtocolConstants.WelcomeResponseLength, null, cancellationToken);
            var responseStr = Encoding.ASCII.GetString(response).TrimEnd('\0');
            var isValid = responseStr == ProtocolConstants.WelcomeResponse;

            _log?.LogInformation("Handshake result: {Result} (received: '{Response}')",
                isValid ? "SUCCESS" : "FAILED", responseStr);

            return isValid;
        }
        catch (Exception ex)
        {
            _log?.LogWarning(ex, "Handshake failed with exception");
            return false;
        }
    }

    /// <summary>
    /// Reads vendor data asynchronously.
    /// </summary>
    public async Task<VendorDataStruct> ReadVendorDataAsync(CancellationToken cancellationToken = default)
    {
        var response = await SendCommandAsync(UartCommand.ReadVendorData, ProtocolConstants.VendorDataLength, null, cancellationToken);
        return BytesToStruct<VendorDataStruct>(response);
    }

    /// <summary>
    /// Reads complete sensor telemetry asynchronously.
    /// </summary>
    public async Task<SensorStruct> ReadSensorsAsync(CancellationToken cancellationToken = default)
    {
        unsafe
        {
            var expectedSize = sizeof(SensorStruct);
            var response = await SendCommandAsync(UartCommand.ReadSensors, expectedSize, null, cancellationToken);
            return BytesToStruct<SensorStruct>(response);
        }
    }

    /// <summary>
    /// Reads device name asynchronously.
    /// </summary>
    public async Task<string> ReadDeviceNameAsync(CancellationToken cancellationToken = default)
    {
        var response = await SendCommandAsync(UartCommand.ReadName, ProtocolConstants.DeviceNameLength, null, cancellationToken);
        return Encoding.ASCII.GetString(response).TrimEnd('\0');
    }

    /// <summary>
    /// Writes device name asynchronously.
    /// </summary>
    public async Task WriteDeviceNameAsync(string name, CancellationToken cancellationToken = default)
    {
        if (name.Length > ProtocolConstants.DeviceNameLength)
        {
            throw new ArgumentException($"Device name must be {ProtocolConstants.DeviceNameLength} characters or less", nameof(name));
        }

        var nameBytes = new byte[ProtocolConstants.DeviceNameLength];
        var encoded = Encoding.ASCII.GetBytes(name);
        Array.Copy(encoded, nameBytes, Math.Min(encoded.Length, ProtocolConstants.DeviceNameLength));

        await SendCommandAsync(UartCommand.WriteName, 1, nameBytes, cancellationToken);
    }

    /// <summary>
    /// Executes a device action command asynchronously.
    /// </summary>
    public async Task<byte> ExecuteActionAsync(byte actionId, CancellationToken cancellationToken = default)
    {
        var response = await SendCommandAsync(UartCommand.Action, 1, new[] { actionId }, cancellationToken);
        return response[0];
    }

    /// <summary>
    /// Reads device unique identifier asynchronously.
    /// </summary>
    public async Task<DeviceUidStruct> ReadDeviceUidAsync(CancellationToken cancellationToken = default)
    {
        unsafe
        {
            var expectedSize = sizeof(DeviceUidStruct);
            var response = await SendCommandAsync(UartCommand.ReadUid, expectedSize, null, cancellationToken);
            return BytesToStruct<DeviceUidStruct>(response);
        }
    }

    /// <summary>
    /// Reads fan profile configuration asynchronously.
    /// </summary>
    public async Task<FanProfileStruct> ReadFanProfileAsync(byte fanIndex, CancellationToken cancellationToken = default)
    {
        if (fanIndex >= ProtocolConstants.FanCount)
        {
            throw new ArgumentException($"Fan index must be 0-{ProtocolConstants.FanCount - 1}", nameof(fanIndex));
        }

        unsafe
        {
            var expectedSize = sizeof(FanProfileStruct);
            var response = await SendCommandAsync(UartCommand.ReadFanProfile, expectedSize, new[] { fanIndex }, cancellationToken);
            return BytesToStruct<FanProfileStruct>(response);
        }
    }

    /// <summary>
    /// Writes fan profile configuration asynchronously.
    /// </summary>
    public async Task<byte> WriteFanProfileAsync(byte fanIndex, FanProfileStruct profile, CancellationToken cancellationToken = default)
    {
        if (fanIndex >= ProtocolConstants.FanCount)
        {
            throw new ArgumentException($"Fan index must be 0-{ProtocolConstants.FanCount - 1}", nameof(fanIndex));
        }

        var profileBytes = StructToBytes(profile);
        var data = new byte[1 + profileBytes.Length];
        data[0] = fanIndex;
        Array.Copy(profileBytes, 0, data, 1, profileBytes.Length);

        var response = await SendCommandAsync(UartCommand.WriteFanProfile, 1, data, cancellationToken);
        return response[0];
    }

    /// <summary>
    /// Reads RGB LED configuration asynchronously.
    /// </summary>
    public async Task<RgbStruct> ReadRgbAsync(CancellationToken cancellationToken = default)
    {
        unsafe
        {
            var expectedSize = sizeof(RgbStruct);
            var response = await SendCommandAsync(UartCommand.ReadRgb, expectedSize, null, cancellationToken);
            return BytesToStruct<RgbStruct>(response);
        }
    }

    /// <summary>
    /// Writes RGB LED configuration asynchronously.
    /// </summary>
    public async Task<byte> WriteRgbAsync(RgbStruct rgb, CancellationToken cancellationToken = default)
    {
        var rgbBytes = StructToBytes(rgb);
        var response = await SendCommandAsync(UartCommand.WriteRgb, 1, rgbBytes, cancellationToken);
        return response[0];
    }

    /// <summary>
    /// Reads calibration data asynchronously.
    /// </summary>
    public async Task<CalibrationStruct> ReadCalibrationAsync(CancellationToken cancellationToken = default)
    {
        unsafe
        {
            var expectedSize = sizeof(CalibrationStruct);
            var response = await SendCommandAsync(UartCommand.ReadCalibration, expectedSize, null, cancellationToken);
            return BytesToStruct<CalibrationStruct>(response);
        }
    }

    /// <summary>
    /// Writes calibration data asynchronously.
    /// </summary>
    public async Task<byte> WriteCalibrationAsync(CalibrationStruct calibration, CancellationToken cancellationToken = default)
    {
        var calBytes = StructToBytes(calibration);
        var response = await SendCommandAsync(UartCommand.WriteCalibration, 1, calBytes, cancellationToken);
        return response[0];
    }

    /// <summary>
    /// Loads calibration data from flash asynchronously.
    /// </summary>
    public async Task<byte> LoadCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var response = await SendCommandAsync(UartCommand.LoadCalibration, 1, null, cancellationToken);
        return response[0];
    }

    /// <summary>
    /// Stores calibration data to flash asynchronously.
    /// </summary>
    public async Task<byte> StoreCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var response = await SendCommandAsync(UartCommand.StoreCalibration, 1, null, cancellationToken);
        return response[0];
    }

    #endregion

    /// <summary>
    /// Converts byte array to struct using unsafe pointer casting (20-30% faster than GCHandle).
    /// Eliminates GCHandle allocation overhead for hot path operations.
    /// </summary>
    private static unsafe T BytesToStruct<T>(byte[] bytes) where T : struct
    {
        fixed (byte* ptr = bytes)
        {
            return *(T*)ptr;
        }
    }

    /// <summary>
    /// Converts struct to byte array using unsafe pointer casting (20-30% faster than GCHandle).
    /// Eliminates GCHandle allocation overhead for hot path operations.
    /// </summary>
    private static unsafe byte[] StructToBytes<T>(T structure) where T : struct
    {
        var size = Marshal.SizeOf<T>();
        var bytes = new byte[size];
        fixed (byte* ptr = bytes)
        {
            *(T*)ptr = structure;
        }
        return bytes;
    }
}
